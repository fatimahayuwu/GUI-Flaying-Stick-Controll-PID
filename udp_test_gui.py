import sys, time
from PySide6.QtCore import QTimer
from PySide6.QtNetwork import QUdpSocket, QHostAddress
from PySide6.QtCore import QByteArray
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                               QLineEdit, QPushButton, QLabel, QDoubleSpinBox,
                               QPlainTextEdit, QFormLayout, QSpinBox)


class UdpTest(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UDP Comm Test - Balancing Stick")
        self.resize(560, 560)

        self.sock = QUdpSocket(self)
        self.sock.bind(QHostAddress.SpecialAddress.AnyIPv4, 0)
        self.sock.readyRead.connect(self.on_ready)

        self.connected = False
        self.rx_count = 0
        self.lost = 0
        self.last_seq = None
        self.rate_count = 0
        self.rtts = []

        # --- UI ---
        self.ip = QLineEdit("192.168.4.1")
        self.port = QSpinBox(); self.port.setRange(1, 65535); self.port.setValue(4210)
        btn_conn = QPushButton("Connect (HELLO)")
        btn_ping = QPushButton("PING")
        btn_led_on = QPushButton("LED ON")
        btn_led_off = QPushButton("LED OFF")
        btn_tel_on = QPushButton("Telemetri ON")
        btn_tel_off = QPushButton("Telemetri OFF")

        self.kp = self._spin(0.02); self.ki = self._spin(0.0); self.kd = self._spin(0.002)
        self.sp = self._spin(0.0, dec=2)
        btn_send = QPushButton("Kirim S,kp,ki,kd,sp")

        self.lbl_status = QLabel("Status: belum terhubung")
        self.lbl_stat = QLabel("RX: 0 | Loss: 0 (0.0%) | Rate: 0 Hz | RTT: - ms")
        self.lbl_tel = QLabel("Telemetri terakhir: -")
        self.log = QPlainTextEdit(); self.log.setReadOnly(True)

        top = QHBoxLayout()
        top.addWidget(QLabel("IP ESP32:")); top.addWidget(self.ip)
        top.addWidget(QLabel("Port:")); top.addWidget(self.port)
        top.addWidget(btn_conn)

        row1 = QHBoxLayout()
        for b in (btn_ping, btn_led_on, btn_led_off, btn_tel_on, btn_tel_off):
            row1.addWidget(b)

        form = QFormLayout()
        form.addRow("Kp", self.kp); form.addRow("Ki", self.ki)
        form.addRow("Kd", self.kd); form.addRow("Setpoint", self.sp)

        lay = QVBoxLayout(self)
        lay.addLayout(top); lay.addLayout(row1)
        lay.addLayout(form); lay.addWidget(btn_send)
        lay.addWidget(self.lbl_status); lay.addWidget(self.lbl_stat)
        lay.addWidget(self.lbl_tel); lay.addWidget(self.log)

        # --- Signal ---
        btn_conn.clicked.connect(self.connect_esp)
        btn_ping.clicked.connect(self.send_ping)
        btn_led_on.clicked.connect(lambda: self.send("LED,1"))
        btn_led_off.clicked.connect(lambda: self.send("LED,0"))
        btn_tel_on.clicked.connect(lambda: self.send("TEL,1"))
        btn_tel_off.clicked.connect(lambda: self.send("TEL,0"))
        btn_send.clicked.connect(self.send_params)

        # Heartbeat 500 ms
        self.hb = QTimer(self); self.hb.timeout.connect(lambda: self.send("HB", log=False))
        # Hitung rate telemetri tiap 1 s
        self.rate_t = QTimer(self); self.rate_t.timeout.connect(self.update_stats)
        self.rate_t.start(1000)

    def _spin(self, val, dec=5):
        s = QDoubleSpinBox(); s.setDecimals(dec); s.setRange(-1000, 1000)
        s.setSingleStep(0.001); s.setValue(val)
        return s

    def send(self, msg, log=True):
        self.sock.writeDatagram(QByteArray(msg.encode()),
                                QHostAddress(self.ip.text().strip()), self.port.value())
        if log:
            self.log.appendPlainText(f"TX> {msg}")

    def connect_esp(self):
        self.last_seq = None
        self.rx_count = 0
        self.lost = 0
        self.send("HELLO")
        self.hb.start(500)

    def send_ping(self):
        self.send(f"PING,{time.perf_counter() * 1000:.3f}")

    def send_params(self):
        self.send(f"S,{self.kp.value():.5f},{self.ki.value():.5f},"
                  f"{self.kd.value():.5f},{self.sp.value():.3f}")

    def on_ready(self):
        while self.sock.hasPendingDatagrams():
            dg = self.sock.receiveDatagram()
            msg = bytes(dg.data()).decode(errors="ignore").strip()
            self.handle(msg)

    def handle(self, msg):
        p = msg.split(",")
        if p[0] == "T" and len(p) >= 4:
            seq = int(p[1])
            if self.last_seq is not None and seq > self.last_seq + 1:
                self.lost += seq - self.last_seq - 1
            self.last_seq = seq
            self.rx_count += 1
            self.rate_count += 1
            self.lbl_tel.setText(f"Telemetri terakhir: seq={seq}  t_esp={p[2]} ms  dummy={p[3]}")
            return  # telemetri tidak dimasukkan ke log (terlalu ramai)
        if p[0] == "PONG" and len(p) >= 3:
            rtt = time.perf_counter() * 1000 - float(p[1])
            self.rtts.append(rtt)
            self.rtts = self.rtts[-20:]
            self.log.appendPlainText(f"RX< {msg}   -> RTT {rtt:.1f} ms")
            return
        if p[0] == "HELLO_ACK":
            self.connected = True
            self.lbl_status.setText("Status: TERHUBUNG")
        self.log.appendPlainText(f"RX< {msg}")

    def update_stats(self):
        total = self.rx_count + self.lost
        loss_pct = (self.lost / total * 100) if total else 0.0
        rtt = f"{sum(self.rtts)/len(self.rtts):.1f}" if self.rtts else "-"
        self.lbl_stat.setText(f"RX: {self.rx_count} | Loss: {self.lost} ({loss_pct:.1f}%) | "
                              f"Rate: {self.rate_count} Hz | RTT rata-rata: {rtt} ms")
        self.rate_count = 0


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = UdpTest(); w.show()
    sys.exit(app.exec())