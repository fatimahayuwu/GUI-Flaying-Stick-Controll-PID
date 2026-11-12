"""GUI Tkinter sederhana untuk ESP32 lewat serial (USB).

- Memilih port COM dan connect/disconnect
- Menampilkan semua baris dari ESP32 di log
- Baris berformat "VAL,<angka>" digambar di grafik
- Kotak input di bawah menggantikan Serial Monitor (kirim teks ke ESP32)

Install dulu:  pip install pyserial matplotlib
"""
import queue
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext

import serial
from serial.tools import list_ports
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

BAUD = 115200
MAX_POINTS = 100


class App:
    def __init__(self, root):
        self.root = root
        root.title("ESP32 Serial GUI")
        self.ser = None
        self.reader = None
        self.stop_evt = threading.Event()
        self.q = queue.Queue()
        self.values = []

        # --- baris atas: port dan koneksi ---
        top = ttk.Frame(root, padding=6)
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

        # --- tengah: log dan grafik ---
        mid = ttk.Frame(root, padding=6)
        mid.pack(fill="both", expand=True)
        self.log = scrolledtext.ScrolledText(mid, width=38, height=18, state="disabled")
        self.log.pack(side="left", fill="y")

        fig = Figure(figsize=(5, 3), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Nilai dari ESP32")
        self.ax.set_xlabel("Sampel")
        self.ax.set_ylabel("Nilai")
        self.ax.grid(True)
        (self.line,) = self.ax.plot([], [], marker="o", markersize=3)
        self.canvas = FigureCanvasTkAgg(fig, master=mid)
        self.canvas.get_tk_widget().pack(side="left", fill="both", expand=True, padx=(6, 0))

        # --- bawah: input kirim ---
        bottom = ttk.Frame(root, padding=6)
        bottom.pack(fill="x")
        self.entry = ttk.Entry(bottom)
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", lambda e: self.send())
        ttk.Button(bottom, text="Kirim", command=self.send).pack(side="left", padx=(6, 0))

        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.poll)

    # ---------- util ----------
    def write_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ---------- koneksi ----------
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
        self.stop_evt.clear()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.btn_conn.configure(text="Disconnect")
        self.status.configure(text=f"Terhubung: {port}")
        self.write_log(f"[i] Terhubung ke {port} @ {BAUD}")

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

    # ---------- thread pembaca ----------
    def read_loop(self):
        while not self.stop_evt.is_set():
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.q.put(f"ERR,{e}")
                break
            if raw:
                line = raw.decode(errors="ignore").strip()
                if line:
                    self.q.put(line)

    # ---------- update GUI (thread utama) ----------
    def poll(self):
        updated = False
        while not self.q.empty():
            line = self.q.get()
            if line.startswith("VAL,"):
                try:
                    v = float(line[4:])
                    self.values.append(v)
                    self.values = self.values[-MAX_POINTS:]
                    self.write_log(f"Angka: {v}")
                    updated = True
                except ValueError:
                    self.write_log(line)
            elif line.startswith("MSG,"):
                self.write_log(f"ESP32: {line[4:]}")
            elif line.startswith("ERR,"):
                self.write_log(f"[!] Serial error: {line[4:]}")
                self.disconnect()
            else:
                self.write_log(line)
        if updated:
            self.line.set_data(range(len(self.values)), self.values)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()
        self.root.after(50, self.poll)

    # ---------- kirim ke ESP32 ----------
    def send(self):
        text = self.entry.get().strip()
        if not text:
            return
        if not (self.ser and self.ser.is_open):
            self.write_log("[!] Belum terhubung")
            return
        try:
            self.ser.write((text + "\n").encode())
            self.write_log(f"> {text}")
            self.entry.delete(0, "end")
        except (serial.SerialException, OSError) as e:
            self.write_log(f"[!] Gagal kirim: {e}")

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()