"""GUI kontrol PID (Tkinter) untuk ESP32 + IMU.

Layout:
  - kartu P / I / D saat ini (dikonfirmasi balik oleh ESP32)
  - grafik sudut IMU + setpoint, dengan kotak ESS / OS / TS / TP
  - grafik error, dengan kotak ESS / OS / TS / TP
  - input P, I, D, setpoint + tombol Kirim
  - log serial di kiri

Protokol (115200 baud):
  ESP32 -> GUI : DATA,t_ms,sudut,setpoint,error | PID,Kp,Ki,Kd,sp | MSG,teks
  GUI -> ESP32 : SET,Kp,Ki,Kd,sp | GET

Install:  python -m pip install pyserial matplotlib
"""
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

import numpy as np
import serial
from serial.tools import list_ports
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

BAUD = 115200
WINDOW_S = 20.0      # lebar jendela grafik (detik)
MAX_STORE = 60000    # batas jumlah sampel tersimpan
MAX_PLOT = 1500      # jumlah sampel terakhir yang digambar
LOG_MAX_LINES = 500


# ---------------------------------------------------------------
# Metrik respon step: ESS, OS, TS, TP
# ---------------------------------------------------------------
def step_metrics(t, y, r, band_frac=0.02, min_band=0.5):
    """Hitung metrik respon dari awal 'step' (setpoint terakhir berubah).

    ESS : error steady state = r - rata-rata y pada 10% sampel terakhir
    OS  : overshoot (% dari besar step)
    TS  : settling time, masuk dan bertahan dalam pita +-2% (min 0,5 derajat)
    TP  : peak time (waktu puncak overshoot)
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(y)
    if n < 10:
        return None
    k = max(5, n // 10)
    res = {"ess": r - float(np.mean(y[-k:])), "os": None, "ts": None, "tp": None}

    step = r - y[0]
    if abs(step) < 2.0:          # tidak ada step berarti, OS/TS/TP tidak bermakna
        return res

    d = 1.0 if step > 0 else -1.0
    swing = d * (y - r)          # > 0 artinya melewati setpoint
    peak = float(np.max(swing))
    if peak > min_band:
        res["os"] = peak / abs(step) * 100.0
        res["tp"] = float(t[int(np.argmax(swing))] - t[0])
    else:
        res["os"] = 0.0

    band = max(band_frac * abs(step), min_band)
    outside = np.where(np.abs(y - r) > band)[0]
    if len(outside) == 0:
        res["ts"] = 0.0
    elif outside[-1] >= n - 1:
        res["ts"] = "belum"      # belum settle
    else:
        res["ts"] = float(t[outside[-1] + 1] - t[0])
    return res


def fmt(v, unit, digits=2):
    if v is None:
        return "-"
    if isinstance(v, str):
        return v
    if abs(v) < 0.5 * 10 ** (-digits):
        v = 0.0                  # hindari tampilan "-0.00"
    return f"{v:.{digits}f} {unit}"


def stats_text(res):
    if res is None:
        return "ESS : -\nOS  : -\nTS  : -\nTP  : -"
    return (f"ESS : {fmt(res['ess'], 'deg')}\n"
            f"OS  : {fmt(res['os'], '%', 1)}\n"
            f"TS  : {fmt(res['ts'], 's')}\n"
            f"TP  : {fmt(res['tp'], 's')}")


# ---------------------------------------------------------------
# Aplikasi
# ---------------------------------------------------------------
class App:
    def __init__(self, root):
        self.root = root
        root.title("GUI Kendali PID - ESP32 + IMU")
        root.geometry("1250x760")

        self.ser = None
        self.reader = None
        self.stop_evt = threading.Event()
        self.q = queue.Queue()

        self.reset_data()
        self.last_draw = 0.0
        self.last_stats = 0.0

        self.build_ui()
        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.poll)

    # ---------- data ----------
    def reset_data(self):
        self.t, self.y, self.r, self.e = [], [], [], []
        self.t0 = None
        self.step_idx = 0

    def add_sample(self, t_s, y, r, e):
        if self.t0 is None or (self.t and t_s < self.t0 + self.t[-1] - 0.5):
            self.reset_data()            # ESP32 restart: mulai sesi baru
            self.t0 = t_s
        if self.r and abs(r - self.r[-1]) > 1e-6:
            self.step_idx = len(self.t)  # setpoint berubah: mulai jendela step baru
        self.t.append(t_s - self.t0)
        self.y.append(y)
        self.r.append(r)
        self.e.append(e)
        if len(self.t) > MAX_STORE:
            cut = MAX_STORE // 6
            for arr in (self.t, self.y, self.r, self.e):
                del arr[:cut]
            self.step_idx = max(0, self.step_idx - cut)

    # ---------- UI ----------
    def build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=14, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.btn_conn.pack(side="left", padx=8)
        self.status = ttk.Label(top, text="Terputus")
        self.status.pack(side="left")

        body = ttk.Frame(self.root, padding=6)
        body.pack(fill="both", expand=True)

        # log di kiri
        self.log = scrolledtext.ScrolledText(body, width=34, state="disabled", font=("Consolas", 9))
        self.log.pack(side="left", fill="y")

        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=(8, 0))

        # kartu P / I / D saat ini
        cards = ttk.Frame(right)
        cards.pack(fill="x", pady=(0, 6))
        self.lbl_p = self.card(cards, "P = -")
        self.lbl_i = self.card(cards, "I = -")
        self.lbl_d = self.card(cards, "D = -")
        self.lbl_sp = self.card(cards, "Setpoint = -")

        # grafik
        fig = Figure(figsize=(8, 6), dpi=100)
        self.ax1, self.ax2 = fig.subplots(2, 1, sharex=True)

        self.ax1.set_title("Grafik Nilai Sudut Sensor IMU", fontsize=11)
        self.ax1.set_ylabel("Sudut (deg)")
        self.ax1.grid(True)
        (self.l_y,) = self.ax1.plot([], [], lw=1.5, label="Sudut")
        (self.l_r,) = self.ax1.plot([], [], lw=1.2, ls="--", color="tab:red", label="Setpoint")
        self.ax1.legend(loc="upper right", fontsize=8)

        self.ax2.set_title("Grafik Nilai Error Steady State", fontsize=11)
        self.ax2.set_ylabel("Error (deg)")
        self.ax2.set_xlabel("Waktu (s)")
        self.ax2.grid(True)
        self.ax2.axhline(0, color="0.5", lw=0.8)
        (self.l_e,) = self.ax2.plot([], [], lw=1.5, color="tab:green")

        box = dict(boxstyle="round,pad=0.4", fc="white", ec="0.7")
        self.txt1 = self.ax1.text(0.01, 0.96, stats_text(None), transform=self.ax1.transAxes,
                                  va="top", ha="left", family="monospace", fontsize=9, bbox=box)
        # self.txt2 = self.ax2.text(0.01, 0.96, stats_text(None), transform=self.ax2.transAxes,
        #                           va="top", ha="left", family="monospace", fontsize=9, bbox=box)
        self.ax1.set_xlim(0, WINDOW_S)
        fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # input P / I / D / setpoint + Kirim
        inp = ttk.Frame(right)
        inp.pack(fill="x", pady=(6, 0))
        self.e_p = self.field(inp, "Masukkan nilai P:", "0")
        self.e_i = self.field(inp, "Masukkan nilai I:", "0")
        self.e_d = self.field(inp, "Masukkan nilai D:", "0")
        self.e_sp = self.field(inp, "Setpoint (deg):", "0")
        ttk.Button(inp, text="Kirim", command=self.kirim).pack(side="right", padx=(8, 0))
        for e in (self.e_p, self.e_i, self.e_d, self.e_sp):
            e.bind("<Return>", lambda ev: self.kirim())

    def card(self, parent, text):
        lbl = tk.Label(parent, text=text, font=("Segoe UI", 14), relief="groove",
                       bd=2, padx=14, pady=6, bg="white")
        lbl.pack(side="left", padx=(0, 10))
        return lbl

    def field(self, parent, label, default):
        ttk.Label(parent, text=label).pack(side="left", padx=(0, 4))
        e = ttk.Entry(parent, width=8)
        e.insert(0, default)
        e.pack(side="left", padx=(0, 12))
        return e

    def write_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > LOG_MAX_LINES:
            self.log.delete("1.0", "100.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ---------- koneksi ----------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            self.write_log("[!] Pilih port dulu")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
        except serial.SerialException as e:
            self.write_log(f"[!] Gagal buka {port}: {e}")
            self.ser = None
            return
        self.reset_data()
        self.stop_evt.clear()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.btn_conn.configure(text="Disconnect")
        self.status.configure(text=f"Terhubung: {port}")
        self.write_log(f"[i] Terhubung ke {port} @ {BAUD}")
        self.root.after(2500, lambda: self.send_raw("GET", quiet=True))

    def disconnect(self):
        self.stop_evt.set()
        if self.reader:
            self.reader.join(timeout=1)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_conn.configure(text="Connect")
        self.status.configure(text="Terputus")
        self.write_log("[i] Terputus")

    def read_loop(self):
        while not self.stop_evt.is_set():
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError, AttributeError) as e:
                self.q.put(f"ERR,{e}")
                break
            if raw:
                line = raw.decode(errors="ignore").strip()
                if line:
                    self.q.put(line)

    # ---------- kirim ----------
    def send_raw(self, text, quiet=False):
        if not (self.ser and self.ser.is_open):
            self.write_log("[!] Belum terhubung")
            return False
        try:
            self.ser.write((text + "\n").encode())
        except (serial.SerialException, OSError) as e:
            self.write_log(f"[!] Gagal kirim: {e}")
            return False
        if not quiet:
            self.write_log(f"> {text}")
        return True

    def kirim(self):
        try:
            kp, ki, kd, sp = (float(e.get().strip().replace(",", "."))
                              for e in (self.e_p, self.e_i, self.e_d, self.e_sp))
        except ValueError:
            self.write_log("[!] P, I, D, setpoint harus berupa angka")
            return
        self.send_raw(f"SET,{kp:g},{ki:g},{kd:g},{sp:g}")

    # ---------- update GUI ----------
    def poll(self):
        got_data = False
        while not self.q.empty():
            line = self.q.get()
            if line.startswith("DATA,"):
                p = line.split(",")
                if len(p) == 5:
                    try:
                        tms, y, r, e = (float(x) for x in p[1:])
                    except ValueError:
                        continue
                    self.add_sample(tms / 1000.0, y, r, e)
                    got_data = True
            elif line.startswith("PID,"):
                p = line.split(",")
                if len(p) == 5:
                    self.lbl_p.configure(text=f"P = {p[1]}")
                    self.lbl_i.configure(text=f"I = {p[2]}")
                    self.lbl_d.configure(text=f"D = {p[3]}")
                    self.lbl_sp.configure(text=f"Setpoint = {p[4]}")
                    self.write_log(f"ESP32: PID aktif P={p[1]} I={p[2]} D={p[3]} SP={p[4]}")
            elif line.startswith("MSG,"):
                self.write_log(f"ESP32: {line[4:]}")
            elif line.startswith("ERR,"):
                self.write_log(f"[!] Serial error: {line[4:]}")
                self.disconnect()
            else:
                self.write_log(line)

        now = time.time()
        if got_data and now - self.last_draw >= 0.1:
            self.last_draw = now
            self.draw()
        if self.t and now - self.last_stats >= 0.5:
            self.last_stats = now
            self.update_stats()
        self.root.after(50, self.poll)

    def draw(self):
        if not self.t:
            return
        t = np.array(self.t[-MAX_PLOT:])
        self.l_y.set_data(t, self.y[-MAX_PLOT:])
        self.l_r.set_data(t, self.r[-MAX_PLOT:])
        self.l_e.set_data(t, self.e[-MAX_PLOT:])
        right = t[-1]
        if right < WINDOW_S:
            self.ax1.set_xlim(0, WINDOW_S)
        else:
            self.ax1.set_xlim(right - WINDOW_S, right)
        for ax in (self.ax1, self.ax2):
            ax.relim()
            ax.autoscale_view(scalex=False)
        self.canvas.draw_idle()

    def update_stats(self):
        i0 = self.step_idx
        res = step_metrics(self.t[i0:], self.y[i0:], self.r[-1])
        # error = setpoint - sudut, jadi OS/TS/TP kedua kotak berasal dari respon yang sama
        txt = stats_text(res)
        self.txt1.set_text(txt)
        self.txt2.set_text(txt)
        self.canvas.draw_idle()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()