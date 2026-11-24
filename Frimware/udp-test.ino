// ===== UDP Comm Test - Balancing Stick (ESP32-C3 Super Mini) =====
// Hanya tes komunikasi 2 arah. Motor TIDAK digerakkan.

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- Konfigurasi ----------
#define USE_AP_MODE 1            // 1 = ESP buat WiFi sendiri, 0 = konek ke WiFi/hotspot

const char* AP_SSID  = "BalancingStick";
const char* AP_PASS  = "12345678";      // minimal 8 karakter

const char* STA_SSID = "NAMA_WIFI";     // dipakai jika USE_AP_MODE = 0
const char* STA_PASS = "PASSWORD_WIFI";

const uint16_t UDP_PORT      = 4210;
const uint32_t TELEM_PERIOD  = 20;      // ms -> 50 Hz
const uint32_t LINK_TIMEOUT  = 2000;    // ms tanpa paket dari PC -> link dianggap putus
const int LED_PIN = 8;                  // LED onboard Super Mini (active LOW)

// ---------- State ----------
WiFiUDP udp;
IPAddress clientIP;
uint16_t clientPort = 0;
bool hasClient = false;
bool linkOk    = false;
bool telemOn   = true;

uint32_t lastTelem = 0, lastRx = 0, seqNo = 0;
float kp = 0, ki = 0, kd = 0, sp = 0;
char rxBuf[128];
char txBuf[160];

// ---------- Helper ----------
void motorStop() {
  // TODO: nanti isi dengan ledcWrite(IN1,0); ledcWrite(IN2,0);
}

void sendToClient(const char* msg) {
  if (!hasClient) return;
  udp.beginPacket(clientIP, clientPort);
  udp.print(msg);
  udp.endPacket();
}

void handlePacket(char* msg) {
  // trim newline
  size_t n = strlen(msg);
  while (n > 0 && (msg[n - 1] == '\n' || msg[n - 1] == '\r')) msg[--n] = 0;

  if (strcmp(msg, "HELLO") == 0) {
    snprintf(txBuf, sizeof(txBuf), "HELLO_ACK,%lu", (unsigned long)millis());
    sendToClient(txBuf);
  }
  else if (strcmp(msg, "HB") == 0) {
    // heartbeat: cukup update lastRx (sudah dilakukan di loop)
  }
  else if (strncmp(msg, "PING,", 5) == 0) {
    snprintf(txBuf, sizeof(txBuf), "PONG,%s,%lu", msg + 5, (unsigned long)millis());
    sendToClient(txBuf);
  }
  else if (strncmp(msg, "S,", 2) == 0) {
    float a, b, c, d;
    if (sscanf(msg + 2, "%f,%f,%f,%f", &a, &b, &c, &d) == 4) {
      kp = a; ki = b; kd = c; sp = d;
      snprintf(txBuf, sizeof(txBuf), "ACK,S,%.5f,%.5f,%.5f,%.3f", kp, ki, kd, sp);
    } else {
      snprintf(txBuf, sizeof(txBuf), "ERR,S,format");
    }
    sendToClient(txBuf);
  }
  else if (strncmp(msg, "LED,", 4) == 0) {
    int v = atoi(msg + 4);
    digitalWrite(LED_PIN, v ? LOW : HIGH);   // active LOW
    snprintf(txBuf, sizeof(txBuf), "ACK,LED,%d", v ? 1 : 0);
    sendToClient(txBuf);
  }
  else if (strncmp(msg, "TEL,", 4) == 0) {
    telemOn = atoi(msg + 4) != 0;
    snprintf(txBuf, sizeof(txBuf), "ACK,TEL,%d", telemOn ? 1 : 0);
    sendToClient(txBuf);
  }
  else {
    snprintf(txBuf, sizeof(txBuf), "ERR,unknown");
    sendToClient(txBuf);
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // LED mati

#if USE_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // stabilkan WiFi di board Super Mini
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP aktif, SSID: "); Serial.println(AP_SSID);
  Serial.print("IP ESP32: "); Serial.println(WiFi.softAPIP());
#else
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("Menghubungkan");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("IP ESP32: "); Serial.println(WiFi.localIP());
#endif

  WiFi.setSleep(false);          // matikan power-save WiFi -> latensi lebih rendah & stabil
  udp.begin(UDP_PORT);
  Serial.printf("UDP listen di port %u\n", UDP_PORT);
}

// ---------- Loop ----------
void loop() {
  // 1) Terima paket
  int sz = udp.parsePacket();
  if (sz > 0) {
    int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
    if (len > 0) {
      rxBuf[len] = 0;
      clientIP   = udp.remoteIP();
      clientPort = udp.remotePort();
      hasClient  = true;
      lastRx     = millis();
      if (!linkOk) { linkOk = true; Serial.println("Link OK"); }
      handlePacket(rxBuf);
    }
  }

  // 2) Failsafe link
  if (hasClient && linkOk && (millis() - lastRx > LINK_TIMEOUT)) {
    linkOk = false;
    motorStop();
    Serial.println("LINK LOST -> motorStop()");
  }

  // 3) Telemetri 50 Hz (hanya jika link OK)
  uint32_t now = millis();
  if (linkOk && telemOn && (now - lastTelem >= TELEM_PERIOD)) {
    lastTelem = now;
    float dummy = 10.0f * sinf(now * 0.002f);   // DATA DUMMY, bukan sensor
    snprintf(txBuf, sizeof(txBuf), "T,%lu,%lu,%.3f",
             (unsigned long)seqNo++, (unsigned long)now, dummy);
    sendToClient(txBuf);
  }
}