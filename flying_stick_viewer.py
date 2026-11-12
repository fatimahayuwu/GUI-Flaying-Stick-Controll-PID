"""
Flying Stick 3D viewer
Membaca telemetri ESP32 (serial) dan memutar model OBJ sesuai sudut IMU.

Install:
    pip install pyvista pyserial numpy

Contoh pakai:
    python flying_stick_viewer.py --model stick.obj --port COM5
    python flying_stick_viewer.py --model stick.obj --port /dev/ttyUSB0 --axis y
    python flying_stick_viewer.py --model stick.obj --demo      # tanpa ESP32, sudut disimulasikan

Format paket dari ESP32 (satu baris per sampel, 115200 baud):
    T,<ms>,<angle_deg>,<gyro_dps>,<setpoint_deg>,<pid_out>,<pwm>\n
Contoh:
    T,12345,3.42,-10.5,5.00,0.12,142
"""

import argparse
import math
import threading
import time

import numpy as np
import pyvista as pv

try:
    import serial
except ImportError:
    serial = None


# --------------------------------------------------------------------------
# Serial reader (thread terpisah)
# --------------------------------------------------------------------------
class TelemetryReader(threading.Thread):
    FIELDS = ("ms", "angle", "gyro", "setpoint", "pid", "pwm")

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.lock = threading.Lock()
        self.data = {k: 0.0 for k in self.FIELDS}
        self.connected = False
        self.good = 0
        self.bad = 0
        self.last_rx = 0.0
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def snapshot(self):
        with self.lock:
            d = dict(self.data)
            d.update(connected=self.connected, good=self.good, bad=self.bad,
                     age=time.time() - self.last_rx if self.last_rx else 1e9)
        return d

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.5) as ser:
                    self.connected = True
                    while not self._stop.is_set():
                        line = ser.readline().decode("ascii", errors="ignore").strip()
                        if line:
                            self._parse(line)
            except Exception:
                self.connected = False
                time.sleep(1.0)  # coba sambung ulang

    def _parse(self, line):
        parts = line.split(",")
        try:
            if parts[0] != "T" or len(parts) != 7:
                raise ValueError
            vals = [float(p) for p in parts[1:]]
        except ValueError:
            with self.lock:
                self.bad += 1
            return
        with self.lock:
            self.data = dict(zip(self.FIELDS, vals))
            self.good += 1
            self.last_rx = time.time()


class DemoSource:
    """Pengganti ESP32 untuk tes tampilan."""
    def __init__(self):
        self.t0 = time.time()

    def snapshot(self):
        t = time.time() - self.t0
        return dict(ms=t * 1000, angle=12 * math.sin(2 * math.pi * 0.4 * t),
                    gyro=12 * 2 * math.pi * 0.4 * math.cos(2 * math.pi * 0.4 * t),
                    setpoint=5.0, pid=0.0, pwm=0.0,
                    connected=True, good=0, bad=0, age=0.0)

    def stop(self):
        pass


# --------------------------------------------------------------------------
# Transformasi
# --------------------------------------------------------------------------
def rotation_about_pivot(angle_deg, axis, pivot):
    """Matriks 4x4: rotasi angle_deg di sumbu `axis` melewati titik `pivot`."""
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


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path file .obj")
    ap.add_argument("--port", help="serial port, mis. COM5 atau /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--axis", choices=["x", "y", "z"], default="x",
                    help="sumbu rotasi model (sumbu pivot) di koordinat OBJ")
    ap.add_argument("--pivot", type=float, nargs=3, default=[0, 0, 0],
                    metavar=("X", "Y", "Z"), help="posisi pivot di koordinat OBJ")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="skala model (mis. 0.001 kalau OBJ dalam mm)")
    ap.add_argument("--invert", action="store_true", help="balik arah rotasi")
    ap.add_argument("--offset", type=float, default=0.0,
                    help="offset sudut (derajat) untuk kalibrasi nol visual")
    ap.add_argument("--smooth", type=float, default=0.0,
                    help="0 = tanpa smoothing, 0.5 = smoothing sedang (0..0.95)")
    ap.add_argument("--demo", action="store_true", help="pakai sudut simulasi")
    args = ap.parse_args()

    # Sumber data
    if args.demo:
        src = DemoSource()
    else:
        if serial is None:
            raise SystemExit("pyserial belum terpasang: pip install pyserial")
        if not args.port:
            raise SystemExit("Isi --port atau pakai --demo")
        src = TelemetryReader(args.port, args.baud)
        src.start()

    # Model
    mesh = pv.read(args.model)
    if args.scale != 1.0:
        mesh = mesh.scale(args.scale, inplace=False)
    pivot = np.array(args.pivot, dtype=float) * args.scale

    pl = pv.Plotter(title="Flying Stick 3D")
    pl.set_background("white")
    actor = pl.add_mesh(mesh, color="lightsteelblue", smooth_shading=True,
                        show_edges=False)

    # Marker pivot + garis vertikal referensi + sumbu
    pl.add_mesh(pv.Sphere(radius=0.01 * max(mesh.length, 1e-6) * 3,
                          center=pivot), color="red")
    size = mesh.length
    ref_dir = {"x": (0, 0, 1), "y": (0, 0, 1), "z": (0, 1, 0)}[args.axis]
    ref_end = pivot + np.array(ref_dir) * size * 1.1
    pl.add_mesh(pv.Line(pivot, ref_end), color="gray", line_width=2)
    pl.add_axes()
    pl.camera_position = "iso"

    angle_f = 0.0
    sign = -1.0 if args.invert else 1.0
    alpha = min(max(args.smooth, 0.0), 0.95)
    pl.show(interactive_update=True, auto_close=False)

    try:
        while not getattr(pl, "_closed", False):
            d = src.snapshot()
            ang = sign * d["angle"] + args.offset
            angle_f = alpha * angle_f + (1 - alpha) * ang

            actor.user_matrix = rotation_about_pivot(angle_f, args.axis, pivot)

            status = "OK" if d["connected"] and d["age"] < 0.5 else "NO DATA"
            hud = (f"Status   : {status}\n"
                   f"Angle    : {d['angle']:7.2f} deg\n"
                   f"Setpoint : {d['setpoint']:7.2f} deg\n"
                   f"Gyro     : {d['gyro']:7.1f} deg/s\n"
                   f"PID out  : {d['pid']:7.3f}\n"
                   f"PWM      : {d['pwm']:7.0f}\n"
                   f"Paket    : {d['good']} ok / {d['bad']} error")
            pl.add_text(hud, position="upper_left", font_size=10,
                        color="black", name="hud", font="courier")

            pl.update()
            time.sleep(1 / 60)
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()
        pl.close()


if __name__ == "__main__":
    main()