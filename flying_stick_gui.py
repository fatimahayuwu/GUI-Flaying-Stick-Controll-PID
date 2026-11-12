"""
Flying Stick PID Monitor
3D viewer (kiri) + grafik, status, dan input PID/setpoint (kanan) dalam satu jendela.

Install (sekali saja):
    py -3.12 -m pip install PySide6 pyvistaqt pyqtgraph pyvista pyserial numpy

Jalankan:
    py -3.12 flying_stick_gui.py                 # model default: .\\3Dmodel\\iterasi2.obj
    py -3.12 flying_stick_gui.py --demo          # tanpa ESP32 (data simulasi)
    py -3.12 flying_stick_gui.py --model "path\\ke\\model.obj" --axis x --invert

Telemetri ESP32 -> PC (115200 baud, satu baris per sampel, disarankan 50 Hz):
    T,<ms>,<angle_deg>,<gyro_dps>,<setpoint_deg>,<pid_out>,<pwm>\\n

Perintah PC -> ESP32:
    S,<kp>,<ki>,<kd>,<setpoint_deg>\\n     (tombol Kirim)
    X\\n                                    (tombol STOP: matikan motor)
"""

import os

os.environ.setdefault("QT_API", "pyside6")

import argparse
import math
import queue
import sys
import threading
import time
from bisect import bisect_left

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

import pyqtgraph as pg
import pyvista as pv
from pyvistaqt import QtInteractor

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


# ---------------------------------------------------------------------------
# Serial link (thread terpisah)
# ---------------------------------------------------------------------------
class SerialLink:
    def __init__(self, baud):
        self.baud = baud
        self.ser = None
        self.thread = None
        self.q = queue.SimpleQueue()
        self.stop_evt = threading.Event()
        self.wlock = threading.Lock()
        self.error = None
        self.good = 0
        self.bad = 0

    @property
    def is_open(self):
        return (self.ser is not None and self.thread is not None
                and self.thread.is_alive())

    def open(self, port):
        self.close()
        self.ser = serial.Serial(port, self.baud, timeout=0.2)
        self.error = None
        self.good = self.bad = 0
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop_evt.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.thread = None

    def write(self, text):
        with self.wlock:
            self.ser.write(text.encode("ascii"))

    def _run(self):
        ser = self.ser
        while not self.stop_evt.is_set():
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as e:
                self.error = str(e)
                break
            line = raw.decode("ascii", errors="ignore").strip()
            if not line.startswith("T,"):
                continue  # abaikan teks debug
            parts = line.split(",")
            try:
                if len(parts) != 7:
                    raise ValueError
                self.q.put(tuple(float(p) for p in parts[1:]))
                self.good += 1
            except ValueError:
                self.bad += 1


class DemoSource:
    """Plant orde-2 sederhana (dengan ESS kecil) untuk tes tampilan tanpa ESP32."""

    def __init__(self):
        self.t0 = time.time()
        self.last = self.t0
        self.y = 0.0
        self.v = 0.0
        self.r = 0.0
        self.rng = np.random.default_rng()

    def set_setpoint(self, r):
        self.r = r

    def poll(self):
        now = time.time()
        out = []
        wn, z, dt = 6.0, 0.35, 0.01
        while self.last + dt <= now:
            self.last += dt
            a = wn * wn * (0.97 * self.r - self.y) - 2 * z * wn * self.v
            self.v += a * dt
            self.y += self.v * dt
            out.append(((self.last - self.t0) * 1000.0,
                        self.y + self.rng.normal(0, 0.05),
                        math.degrees(0) + self.v, self.r, 0.0, 0.0))
        return out


# ---------------------------------------------------------------------------
# Transformasi 3D
# ---------------------------------------------------------------------------
def rotation_about_pivot(angle_deg, axis, pivot):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == "y":
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    else:
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    pivot = np.asarray(pivot, dtype=float)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = pivot - R @ pivot
    return M


AXIS_VEC = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}


# ---------------------------------------------------------------------------
# Jendela utama
# ---------------------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args, mesh):
        super().__init__()
        self.setWindowTitle("Flying Stick PID Monitor - ESP32 + IMU")
        self.args = args
        self.window_s = args.window
        self.alpha = min(max(args.smooth, 0.0), 0.95)
        self.sign = -1.0 if args.invert else 1.0
        self.offset = args.offset

        self.demo = DemoSource() if args.demo else None
        self.link = None if args.demo else SerialLink(args.baud)
        self.connected = bool(args.demo)

        self.reset_data()
        self.angle_f = 0.0
        self.last_rx = 0.0

        self._build_ui()
        self._build_scene(mesh)
        self.refresh_ports()
        self._set_connected_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    # ------------------------------------------------------------------ data
    def reset_data(self):
        self.T, self.Y, self.R = [], [], []
        self.G, self.P, self.W = [], [], []
        self.ms0 = None
        self.step_i = 0
        self.step_t = 0.0
        self.step_y0 = 0.0
        self.last = None

    def ingest(self, s):
        ms, ang, gyro, sp, pid, pwm = s
        if self.ms0 is None:
            self.ms0 = ms
        t = (ms - self.ms0) / 1000.0
        if self.T and t < self.T[-1] - 0.5:  # ESP32 restart
            self.reset_data()
            self.ms0 = ms
            t = 0.0
        if (not self.R) or abs(sp - self.R[-1]) > 1e-6:
            self.step_i = len(self.T)
            self.step_t = t
            self.step_y0 = self.Y[-1] if self.Y else ang
        self.T.append(t)
        self.Y.append(ang)
        self.R.append(sp)
        self.G.append(gyro)
        self.P.append(pid)
        self.W.append(pwm)
        self.last = s
        self.last_rx = time.time()
        if len(self.T) > 40000:
            cut = 10000
            for lst in (self.T, self.Y, self.R, self.G, self.P, self.W):
                del lst[:cut]
            self.step_i = max(0, self.step_i - cut)

    def compute_metrics(self):
        res = dict(ess=None, os=None, ts=None, tp=None)
        n = len(self.T)
        i0 = self.step_i
        if n - i0 < 10:
            return res
        t = np.asarray(self.T[i0:])
        y = np.asarray(self.Y[i0:])
        r = self.R[-1]
        dur = t[-1] - self.step_t
        A = r - self.step_y0

        if dur >= 1.5:  # ESS: rata-rata error 1 detik terakhir
            m = t >= t[-1] - 1.0
            res["ess"] = r - float(np.mean(y[m]))

        if abs(A) >= 0.5:  # OS dan TP hanya bermakna untuk step yang cukup besar
            s = math.copysign(1.0, A)
            k = int(np.argmax(s * y))
            if k < len(y) - 3:
                res["tp"] = float(t[k] - self.step_t)
                res["os"] = max(0.0, float((s * y[k] - s * r) / abs(A) * 100.0))

        band = max(0.02 * abs(A), 0.3)  # pita settling: 2% step atau 0.3 deg
        out = np.nonzero(np.abs(y - r) > band)[0]
        if len(out) == 0:
            res["ts"] = 0.0
        elif out[-1] < len(y) - 1 and dur >= 1.5:
            res["ts"] = float(t[out[-1] + 1] - self.step_t)
        return res

    # -------------------------------------------------------------------- UI
    def _build_ui(self):
        pg.setConfigOptions(antialias=True, background="w", foreground="k")

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(split)

        # kiri: 3D viewer
        self.plotter = QtInteractor(self)
        w3d = getattr(self.plotter, "interactor", self.plotter)
        if not isinstance(w3d, QtWidgets.QWidget):
            w3d = self.plotter
        w3d.setMinimumWidth(380)
        split.addWidget(w3d)

        # kanan: kontrol + grafik
        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(6, 6, 6, 6)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Port:"))
        self.cmb = QtWidgets.QComboBox()
        self.cmb.setMinimumWidth(110)
        top.addWidget(self.cmb)
        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top.addWidget(self.btn_refresh)
        self.btn_conn = QtWidgets.QPushButton("Connect")
        self.btn_conn.clicked.connect(self.on_connect)
        top.addWidget(self.btn_conn)
        self.lbl_status = QtWidgets.QLabel("Terputus")
        top.addWidget(self.lbl_status)
        top.addStretch(1)
        rl.addLayout(top)

        chips = QtWidgets.QHBoxLayout()
        self.chip = {}
        for key, name, color in (("p", "P", "#b5651d"), ("i", "I", "#1f5fbf"),
                                 ("d", "D", "#1f5fbf"), ("sp", "Setpoint", "#000000")):
            lab = QtWidgets.QLabel(f"{name} = -")
            lab.setStyleSheet(
                f"QLabel{{color:{color}; border:1px solid #bbb; border-radius:4px;"
                "padding:4px 10px; font-size:14px;}}")
            chips.addWidget(lab)
            self.chip[key] = (lab, name)
        chips.addStretch(1)
        rl.addLayout(chips)

        self.glw = pg.GraphicsLayoutWidget()
        rl.addWidget(self.glw, 1)

        self.p1 = self.glw.addPlot(row=0, col=0, title="Grafik Nilai Sudut Sensor IMU")
        self.p1.setLabel("left", "Sudut (deg)")
        self.p1.setLabel("bottom", "Waktu (s)")
        self.p1.showGrid(x=True, y=True, alpha=0.3)
        self.p1.addLegend(offset=(-10, 10))
        self.c_angle = self.p1.plot(pen=pg.mkPen("#1f77b4", width=2), name="Sudut")
        self.c_set = self.p1.plot(pen=pg.mkPen("k", width=2, style=QtCore.Qt.DashLine),
                                  name="Setpoint")
        self.stats = pg.TextItem(anchor=(0, 0), color="k",
                                 fill=pg.mkBrush(255, 255, 255, 220),
                                 border=pg.mkPen("#bbbbbb"))
        self.stats.setFont(QtGui.QFont("Consolas", 9))
        self.p1.addItem(self.stats, ignoreBounds=True)

        self.p2 = self.glw.addPlot(row=1, col=0, title="Grafik Nilai Error Steady State")
        self.p2.setLabel("left", "Error (deg)")
        self.p2.setLabel("bottom", "Waktu (s)")
        self.p2.showGrid(x=True, y=True, alpha=0.3)
        self.p2.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#999999")))
        self.c_err = self.p2.plot(pen=pg.mkPen("#d62728", width=2))

        self.p1.setXRange(0, self.window_s, padding=0)
        self.p2.setXRange(0, self.window_s, padding=0)
        self.p1.setYRange(-1, 1)
        self.p2.setYRange(-1, 1)
        self._update_stats_text(dict(ess=None, os=None, ts=None, tp=None), 0, 1)

        bottom = QtWidgets.QHBoxLayout()
        self.ed = {}
        for key, label in (("p", "Masukkan nilai P:"), ("i", "Masukkan nilai I:"),
                           ("d", "Masukkan nilai D:"), ("sp", "Setpoint (deg):")):
            bottom.addWidget(QtWidgets.QLabel(label))
            e = QtWidgets.QLineEdit("0")
            e.setFixedWidth(70)
            e.returnPressed.connect(self.on_send)
            bottom.addWidget(e)
            self.ed[key] = e
        bottom.addStretch(1)
        self.btn_send = QtWidgets.QPushButton("Kirim")
        self.btn_send.clicked.connect(self.on_send)
        bottom.addWidget(self.btn_send)
        self.btn_stop = QtWidgets.QPushButton("STOP")
        self.btn_stop.setStyleSheet("QPushButton{background:#c62828;color:white;"
                                    "font-weight:bold;padding:4px 14px;}")
        self.btn_stop.clicked.connect(self.on_stop)
        bottom.addWidget(self.btn_stop)
        rl.addLayout(bottom)

        split.addWidget(right)
        split.setSizes([620, 980])
        self.resize(1600, 820)

    def _build_scene(self, mesh):
        a = self.args
        b = mesh.bounds
        ext = np.array([b[1] - b[0], b[3] - b[2], b[5] - b[4]])
        lo = np.array([b[0], b[2], b[4]])
        ctr = np.array(mesh.center)

        self.up_axis = a.up or "xyz"[int(np.argmax(ext))]
        # self.rot_axis = a.axis or ("x" if self.up_axis != "x" else "z") //ini nanti dia pivot di
        self.rot_axis = a.axis or "z"
        if self.rot_axis == self.up_axis:
            self.rot_axis = "z" if self.up_axis != "z" else "x"

        if a.pivot is not None:
            self.pivot = np.array(a.pivot, dtype=float) * a.scale
        else:  # default: ujung bawah stick (minimum sepanjang sumbu panjang)
            self.pivot = ctr.copy()
            iu = "xyz".index(self.up_axis)
            self.pivot[iu] = lo[iu]

        print(f"[model] ukuran (x,y,z) = {ext.round(4)}")
        print(f"[model] sumbu tegak = {self.up_axis}, sumbu rotasi = {self.rot_axis}")
        print(f"[model] pivot = {self.pivot.round(4)}")

        pl = self.plotter
        pl.set_background("white")
        self.actor = pl.add_mesh(mesh, color="lightsteelblue", smooth_shading=True)
        #pl.add_mesh(pv.Sphere(radius=0.02 * mesh.length, center=self.pivot), color="red")
        up = np.array(AXIS_VEC[self.up_axis], dtype=float)
        pl.add_mesh(pv.Line(self.pivot, self.pivot + up * mesh.length * 0.6),
                    color="gray", line_width=2)
        pl.add_axes()

        view_dir = np.array(AXIS_VEC[self.rot_axis], dtype=float)
        pl.camera_position = [tuple(ctr + view_dir * mesh.length * 2.0),
                              tuple(ctr), tuple(up)]
        pl.reset_camera()

    # ------------------------------------------------------------- handlers
    def msg(self, text):
        self.statusBar().showMessage(text, 6000)

    def refresh_ports(self):
        self.cmb.clear()
        if list_ports is None:
            return
        for p in list_ports.comports():
            self.cmb.addItem(p.device, p.device)

    def _set_connected_ui(self):
        if self.demo:
            self.lbl_status.setText("Demo (simulasi)")
            self.lbl_status.setStyleSheet("color:#b5651d;")
            self.btn_conn.setEnabled(False)
            self.cmb.setEnabled(False)
            self.btn_refresh.setEnabled(False)
            return
        if self.connected:
            self.lbl_status.setText(f"Terhubung ({self.cmb.currentText()})")
            self.lbl_status.setStyleSheet("color:#2e7d32;")
            self.btn_conn.setText("Disconnect")
        else:
            self.lbl_status.setText("Terputus")
            self.lbl_status.setStyleSheet("color:#c62828;")
            self.btn_conn.setText("Connect")

    def on_connect(self):
        if self.connected:
            self.link.close()
            self.connected = False
            self._set_connected_ui()
            return
        if serial is None:
            self.msg("pyserial belum terpasang")
            return
        port = self.cmb.currentData() or self.cmb.currentText()
        if not port:
            self.msg("Tidak ada port. Colokkan ESP32 lalu klik Refresh.")
            return
        try:
            self.link.open(port)
        except Exception as e:
            self.msg(f"Gagal membuka {port}: {e}")
            return
        self.reset_data()
        self.connected = True
        self._set_connected_ui()

    def _handle_lost(self):
        err = self.link.error or "koneksi terputus"
        self.link.close()
        self.connected = False
        self._set_connected_ui()
        self.msg(f"Serial terputus: {err}")

    def on_send(self):
        try:
            kp, ki, kd, sp = [float(self.ed[k].text().strip().replace(",", "."))
                              for k in ("p", "i", "d", "sp")]
        except ValueError:
            self.msg("Input tidak valid: isi P, I, D, dan Setpoint dengan angka.")
            return
        if abs(sp) > 30:
            self.msg("Setpoint ditolak: di luar batas aman +/-30 deg.")
            return
        if self.demo:
            self.demo.set_setpoint(sp)
        else:
            if not self.connected:
                self.msg("Belum terhubung ke ESP32.")
                return
            try:
                self.link.write(f"S,{kp:.4f},{ki:.4f},{kd:.4f},{sp:.3f}\n")
            except Exception as e:
                self.msg(f"Gagal mengirim: {e}")
                return
        for key, val in (("p", kp), ("i", ki), ("d", kd), ("sp", sp)):
            lab, name = self.chip[key]
            lab.setText(f"{name} = {val:g}")
        self.msg("Terkirim.")

    def on_stop(self):
        if self.demo:
            self.demo.set_setpoint(0.0)
            self.msg("STOP (demo).")
            return
        if not self.connected:
            self.msg("Belum terhubung ke ESP32.")
            return
        try:
            self.link.write("X\n")
            self.msg("Perintah STOP dikirim.")
        except Exception as e:
            self.msg(f"Gagal mengirim STOP: {e}")

    # ----------------------------------------------------------------- loop
    def tick(self):
        samples = []
        if self.demo:
            samples = self.demo.poll()
        elif self.link is not None:
            try:
                while True:
                    samples.append(self.link.q.get_nowait())
            except queue.Empty:
                pass
            if self.connected and not self.link.is_open:
                self._handle_lost()
        for s in samples:
            self.ingest(s)

        fresh = self.connected and (time.time() - self.last_rx) < 0.5 and self.last
        if fresh:
            target = self.sign * self.last[1] + self.offset
        else:
            target = 0.0  # tidak ada data -> stick berdiri lurus
        self.angle_f = self.alpha * self.angle_f + (1 - self.alpha) * target
        self.actor.user_matrix = rotation_about_pivot(self.angle_f, self.rot_axis, self.pivot)

        if fresh:
            _, ang, gyro, sp, pid, pwm = self.last
            pk = (f"{self.link.good} ok / {self.link.bad} error"
                  if self.link else "demo")
            hud = (f"Status   : OK\nAngle    : {ang:7.2f} deg\n"
                   f"Setpoint : {sp:7.2f} deg\nGyro     : {gyro:7.1f} deg/s\n"
                   f"PID out  : {pid:7.3f}\nPWM      : {pwm:7.0f}\nPaket    : {pk}")
        else:
            hud = ("Status   : NO DATA\nAngle    :       -\nSetpoint :       -\n"
                   "Gyro     :       -\nPID out  :       -\nPWM      :       -\n"
                   "Paket    : -")
        self.plotter.add_text(hud, position="upper_left", font_size=9,
                              color="black", name="hud", font="courier")
        self.plotter.render()

        if samples and self.T:
            self.update_plots()

    def update_plots(self):
        t_end = self.T[-1]
        i = bisect_left(self.T, t_end - self.window_s)
        t = np.asarray(self.T[i:])
        y = np.asarray(self.Y[i:])
        r = np.asarray(self.R[i:])
        e = r - y
        self.c_angle.setData(t, y)
        self.c_set.setData(t, r)
        self.c_err.setData(t, e)

        x0 = max(0.0, t_end - self.window_s)
        x1 = max(self.window_s, t_end)
        self.p1.setXRange(x0, x1, padding=0)
        self.p2.setXRange(x0, x1, padding=0)

        lo1, hi1 = float(min(y.min(), r.min())), float(max(y.max(), r.max()))
        pad = max(1.0, 0.1 * (hi1 - lo1))
        lo1, hi1 = lo1 - pad, hi1 + pad
        self.p1.setYRange(lo1, hi1, padding=0)
        lo2, hi2 = float(min(e.min(), 0.0)), float(max(e.max(), 0.0))
        pad = max(0.5, 0.1 * (hi2 - lo2))
        self.p2.setYRange(lo2 - pad, hi2 + pad, padding=0)

        self._update_stats_text(self.compute_metrics(), x0 + 0.01 * self.window_s,
                                hi1 - 0.03 * (hi1 - lo1))

    def _update_stats_text(self, m, x, y):
        def f(v, fmt):
            return "-" if v is None else fmt.format(v)
        txt = (f"ESS : {f(m['ess'], '{:+.2f} deg')}\n"
               f"OS  : {f(m['os'], '{:.1f} %')}\n"
               f"TS  : {f(m['ts'], '{:.2f} s')}\n"
               f"TP  : {f(m['tp'], '{:.2f} s')}")
        self.stats.setText(txt)
        self.stats.setPos(x, y)

    def closeEvent(self, ev):
        self.timer.stop()
        if self.link is not None:
            self.link.close()
        try:
            self.plotter.close()
        except Exception:
            pass
        super().closeEvent(ev)


# ---------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(here, "3Dmodel", "iterasi2.obj"))
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--up", choices=["x", "y", "z"],
                    help="sumbu panjang/tegak model (default: otomatis)")
    ap.add_argument("--axis", choices=["x", "y", "z"],
                    help="sumbu rotasi (pivot) model (default: x)")
    ap.add_argument("--pivot", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="posisi pivot di koordinat OBJ (default: ujung bawah stick)")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--invert", action="store_true", help="balik arah rotasi")
    ap.add_argument("--offset", type=float, default=0.0, help="offset sudut visual (deg)")
    ap.add_argument("--smooth", type=float, default=0.0, help="0..0.95")
    ap.add_argument("--window", type=float, default=20.0, help="lebar grafik (detik)")
    ap.add_argument("--demo", action="store_true", help="data simulasi tanpa ESP32")
    args = ap.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit(f"File model tidak ditemukan: {args.model}")
    if serial is None and not args.demo:
        raise SystemExit("pyserial belum terpasang: py -3.12 -m pip install pyserial")

    mesh = pv.read(args.model)
    if args.scale != 1.0:
        mesh = mesh.scale([args.scale] * 3, inplace=False)

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(args, mesh)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()