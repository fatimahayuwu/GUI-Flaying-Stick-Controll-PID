/*
  Flying Stick - firmware ESP32 (Arduino) + tampilan Serial Monitor

  Dasar: firmware Flying Stick (MPU6050 -> complementary filter -> PID dasar -> H-bridge).
  Tambahan di versi ini:
    - PRINT_MODE 1: cetak status terbaca manusia di Serial Monitor (sudut, PWM, arah putar motor).
    - PWM yang dicetak adalah PERINTAH hasil PID, jadi tetap terlihat walau TELEMETRY_ONLY 1
      (motor tidak benar-benar digerakkan).
    - Gain default (DEF_KP/KI/KD) supaya PWM terlihat tanpa harus mengirim perintah S dulu.

  Protokol GUI (PRINT_MODE 0, 115200 baud):
    ESP32 -> PC : T,<ms>,<angle_deg>,<gyro_dps>,<setpoint_deg>,<pid_out>,<pwm>\n
    PC -> ESP32 : S,<kp>,<ki>,<kd>,<setpoint_deg>\n   (set gain + setpoint, sekaligus ARM motor)
                  X\n                                  (STOP: motor mati, harus ARM ulang dengan S)

  Di Serial Monitor Arduino, atur line ending ke "Newline" kalau ingin mengetik perintah S atau X.

  PENTING: semua pin, arah sumbu, dan tanda di bagian KONFIGURASI adalah ASUMSI.
  Sesuaikan dengan rangkaianmu sebelum menyalakan motor.
*/

#include <Arduino.h>
#include <Wire.h>

// =====================================================================
// KONFIGURASI
// =====================================================================

// 1 = motor TIDAK PERNAH digerakkan (hanya baca sensor + hitung/cetak perintah PWM).
#define TELEMETRY_ONLY 1

// ---- Tampilan di Serial Monitor ----
// 0 = format GUI (baris T,...) untuk flying_stick_gui.py
// 1 = format terbaca manusia (PRINT_HZ kali per detik), tanpa baris T
#define PRINT_MODE 1
#define PRINT_HZ   10
// 1 = tambah baris acc/gyro mentah di tiap cetakan (untuk memeriksa AX_ALONG/AX_SIDE/AX_GYRO)
#define SHOW_RAW   0

// Arah putar motor saat duty POSITIF (PWM keluar di PIN_IN1): 1 = CW, 0 = CCW.
// ASUMSI. Tergantung polaritas kabel motor ke driver. Cek dengan melihat motor berputar.
#define POS_DUTY_IS_CW 1
// Aturan yang kamu tentukan: motor CW = DORONG (stick mau jatuh ke DEPAN),
//                            motor CCW = TARIK (stick mau jatuh ke BELAKANG).
// 1 = sudut positif berarti stick miring ke DEPAN. ASUMSI. Cek dengan memiringkan stick.
#define FRONT_IS_POSITIVE 1

// Pin (ASUMSI - sesuaikan dengan board dan wiring-mu).
// Catatan ESP32-C3: GPIO hanya 0..21, jadi pin 22/25/26 di bawah TIDAK valid di C3.
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_IN1 25   // input driver: PWM di sini = duty positif
#define PIN_IN2 26   // input driver: PWM di sini = duty negatif

// PWM
#define PWM_FREQ 20000                  // 20 kHz
#define PWM_BITS 10                     // resolusi 0..1023
#define PWM_MAX  ((1 << PWM_BITS) - 1)

// Orientasi MPU6050 (0=X, 1=Y, 2=Z pada CHIP).
// AX_ALONG = sumbu chip searah panjang stick (kena gravitasi saat stick tegak)
// AX_SIDE  = sumbu chip searah arah miring (maju-mundur)
// AX_GYRO  = sumbu chip sejajar pivot (sumbu putar stick)
#define AX_ALONG 2
#define AX_SIDE  0
#define AX_GYRO  1
#define ALONG_POSITIVE 1   // 1 = sumbu AX_ALONG membaca +1 g saat stick tegak, 0 = -1 g

// Tanda (+1 / -1)
#define ANGLE_SIGN 1.0f   // balik arah sudut positif (gyro ikut terbalik otomatis)
#define GYRO_SIGN  1.0f   // normalnya jangan diubah (sudut accel sudah dihitung searah gyro)
#define MOTOR_SIGN 1.0f   // balik arah dorong motor jika koreksi malah memperparah

// Gain default (dipakai sebelum ada perintah S)
const float DEF_KP = 0.02f;
const float DEF_KI = 0.0f;
const float DEF_KD = 0.002f;

// Parameter
const float LOOP_HZ        = 200.0f;
const float TELEM_HZ       = 50.0f;
const float ALPHA          = 0.995f;  // complementary filter (porsi gyro)
const float TILT_LIMIT_DEG = 45.0f;   // lewat batas ini motor dimatikan (latched)
const float SP_LIMIT_DEG   = 30.0f;   // batas setpoint dari GUI
const float K_MAX          = 10.0f;   // batas gain yang diterima
const float I_LIMIT        = 0.30f;   // batas kontribusi integral (satuan output)
const float U_BIAS         = 0.0f;    // offset output tetap, default 0
const float DEADBAND       = 0.0f;    // duty minimum motor mulai berputar (0..1)
const int   CAL_SAMPLES    = 500;

// =====================================================================
// MPU6050
// =====================================================================
#define MPU_ADDR 0x68
const float ACC_LSB  = 16384.0f;  // +-2 g
const float GYRO_LSB = 65.5f;     // +-500 dps

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  bool ok = true;
  ok &= mpuWrite(0x6B, 0x00);  // bangunkan sensor
  delay(100);
  ok &= mpuWrite(0x19, 0x00);  // sample rate divider
  ok &= mpuWrite(0x1A, 0x03);  // DLPF ~42 Hz
  ok &= mpuWrite(0x1B, 0x08);  // gyro +-500 dps
  ok &= mpuWrite(0x1C, 0x00);  // accel +-2 g
  return ok;
}

// acc dalam g, gyr dalam deg/s (belum dikurangi bias)
bool mpuRead(float acc[3], float gyr[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  for (int i = 0; i < 3; i++) {
    acc[i] = r[i] / ACC_LSB;
    gyr[i] = r[4 + i] / GYRO_LSB;
  }
  return true;
}

// =====================================================================
// Motor (H-bridge 2 input PWM, kelas DRV8833)
// =====================================================================
#ifdef ESP_ARDUINO_VERSION_MAJOR
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    #define PWM_NEW_API 1
  #endif
#endif

void pwmInit() {
#ifdef PWM_NEW_API
  ledcAttach(PIN_IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_IN2, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcSetup(1, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_IN1, 0);
  ledcAttachPin(PIN_IN2, 1);
#endif
}

void pwmWrite(int in1, int in2) {
#ifdef PWM_NEW_API
  ledcWrite(PIN_IN1, in1);
  ledcWrite(PIN_IN2, in2);
#else
  ledcWrite(0, in1);
  ledcWrite(1, in2);
#endif
}

int g_pwmSigned = 0;  // duty yang benar-benar diterapkan ke motor (untuk telemetri GUI)

// u: -1..+1
void setMotor(float u) {
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  int duty = (int)(fabsf(u) * PWM_MAX + 0.5f);
  if (duty == 0) {
    pwmWrite(0, 0);
    g_pwmSigned = 0;
  } else if (u > 0) {
    pwmWrite(duty, 0);
    g_pwmSigned = duty;
  } else {
    pwmWrite(0, duty);
    g_pwmSigned = -duty;
  }
}

// =====================================================================
// State
// =====================================================================
float g_kp = DEF_KP, g_ki = DEF_KI, g_kd = DEF_KD;
float g_setpoint = 0;
float g_integ = 0;           // integral error (deg*s)
bool  g_armed = false;       // motor hanya aktif setelah perintah S diterima
bool  g_tripped = false;     // latched: sudut berlebih / IMU gagal
float g_gyroBias = 0;        // deg/s
float g_angle0 = 0;          // sudut akselerometer saat kalibrasi (= posisi tegak/nol)
float g_angle = 0;           // sudut terfilter (deg)

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Sudut akselerometer = rotasi kaidah tangan kanan di sekitar AX_GYRO (searah gyro positif).
const float ALONG_SGN = ALONG_POSITIVE ? 1.0f : -1.0f;
const float SIDE_SGN  = ((((AX_GYRO + 1) % 3) == AX_SIDE) ? 1.0f : -1.0f) * ALONG_SGN;

float accAngleRaw(const float acc[3]) {
  return degrees(atan2f(SIDE_SGN * acc[AX_SIDE], ALONG_SGN * acc[AX_ALONG]));
}

// Teks arah putar motor dari perintah PWM bertanda
const char* dirText(int pwm) {
  if (pwm == 0) return "STOP        ";
  bool cw = ((pwm > 0) == (POS_DUTY_IS_CW != 0));
  return cw ? "CW  -> DORONG" : "CCW -> TARIK ";
}

// =====================================================================
// Kalibrasi: stick harus DIAM dan TEGAK (posisi nol)
// =====================================================================
bool calibrate() {
  Serial.println("# KALIBRASI: tahan stick tegak dan diam...");
  float sumAcc[3] = {0, 0, 0};
  float sumGyro = 0;
  int good = 0;
  float acc[3], gyr[3];
  for (int i = 0; i < CAL_SAMPLES; i++) {
    if (mpuRead(acc, gyr)) {
      for (int k = 0; k < 3; k++) sumAcc[k] += acc[k];
      sumGyro += gyr[AX_GYRO];
      good++;
    }
    delay(4);
  }
  if (good < CAL_SAMPLES / 2) return false;
  float avgAcc[3] = {sumAcc[0] / good, sumAcc[1] / good, sumAcc[2] / good};
  g_angle0 = accAngleRaw(avgAcc);
  g_gyroBias = sumGyro / good;
  Serial.printf("# KALIBRASI selesai: gyro bias=%.3f dps, sudut nol=%.2f deg\n",
                g_gyroBias, g_angle0);
  if (fabsf(g_gyroBias) > 5.0f) Serial.println("# PERINGATAN: bias gyro besar, stick mungkin bergerak saat kalibrasi");
  return true;
}

// =====================================================================
// Perintah serial
// =====================================================================
void processLine(const char* s) {
  if (s[0] == 'X') {
    g_armed = false;
    g_integ = 0;
    setMotor(0);
    Serial.println("# STOP: motor dimatikan");
    return;
  }
  if (s[0] == 'S' && s[1] == ',') {
    float kp, ki, kd, sp;
    if (sscanf(s, "S,%f,%f,%f,%f", &kp, &ki, &kd, &sp) == 4 &&
        isfinite(kp) && isfinite(ki) && isfinite(kd) && isfinite(sp) &&
        kp >= 0 && ki >= 0 && kd >= 0 &&
        kp <= K_MAX && ki <= K_MAX && kd <= K_MAX &&
        fabsf(sp) <= SP_LIMIT_DEG) {
      g_kp = kp; g_ki = ki; g_kd = kd; g_setpoint = sp;
      g_integ = 0;
      g_tripped = false;
      g_armed = true;
      Serial.printf("# OK S: Kp=%.4f Ki=%.4f Kd=%.4f SP=%.2f (ARMED)\n", kp, ki, kd, sp);
    } else {
      Serial.println("# ERR S: nilai tidak valid / di luar batas");
    }
    return;
  }
  Serial.println("# ERR: perintah tidak dikenal");
}

void handleSerial() {
  static char buf[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        buf[n] = 0;
        processLine(buf);
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;  // baris terlalu panjang, buang
    }
  }
}

// =====================================================================
// setup / loop
// =====================================================================
void setup() {
  // Pin motor dipastikan LOW sebelum apa pun
  pinMode(PIN_IN1, OUTPUT); digitalWrite(PIN_IN1, LOW);
  pinMode(PIN_IN2, OUTPUT); digitalWrite(PIN_IN2, LOW);

  Serial.begin(115200);
  Serial.setTimeout(5);
  delay(500);
  Serial.println("# Flying Stick firmware start");
#if TELEMETRY_ONLY
  Serial.println("# MODE: TELEMETRY_ONLY (motor tidak akan digerakkan, PWM yang tampil = perintah)");
#endif

  pwmInit();
  setMotor(0);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!mpuInit()) {
    Serial.println("# FAULT: MPU6050 tidak merespons (cek wiring SDA/SCL/VCC/GND)");
    g_tripped = true;
    return;
  }
  if (!calibrate()) {
    Serial.println("# FAULT: kalibrasi gagal");
    g_tripped = true;
    return;
  }
  g_angle = 0;
}

void loop() {
  static uint32_t tPrev = micros();
  static uint32_t tTelem = 0;
  static uint32_t tPrint = 0;
  static int failCount = 0;
  const uint32_t loopUs = (uint32_t)(1e6f / LOOP_HZ);
  const uint32_t telemMs = (uint32_t)(1000.0f / TELEM_HZ);
  const uint32_t printMs = (uint32_t)(1000.0f / PRINT_HZ);

  handleSerial();

  uint32_t now = micros();
  if ((uint32_t)(now - tPrev) < loopUs) return;
  float dt = (now - tPrev) * 1e-6f;
  tPrev = now;
  if (dt > 0.05f) dt = 0.05f;

  // --- IMU ---
  float acc[3], gyr[3];
  if (!mpuRead(acc, gyr)) {
    if (++failCount >= 5 && !g_tripped) {
      g_tripped = true;
      g_armed = false;
      setMotor(0);
      Serial.println("# FAULT: pembacaan IMU gagal, motor dimatikan");
    }
    return;
  }
  failCount = 0;

  float gyro = ANGLE_SIGN * GYRO_SIGN * (gyr[AX_GYRO] - g_gyroBias);       // deg/s
  float accAng = ANGLE_SIGN * wrap180(accAngleRaw(acc) - g_angle0);        // deg
  g_angle = ALPHA * (g_angle + gyro * dt) + (1.0f - ALPHA) * accAng;

  // --- Proteksi sudut ---
  if (fabsf(g_angle) > TILT_LIMIT_DEG && !g_tripped) {
    g_tripped = true;
    g_armed = false;
    setMotor(0);
    Serial.println("# TRIP: sudut melewati batas, motor dimatikan (kirim S untuk ARM ulang)");
  }

  // --- PID dasar ---
  float e = g_setpoint - g_angle;
  float pTerm = g_kp * e;
  float dTerm = -g_kd * gyro;                  // derivatif pada pengukuran (tanpa derivative kick)
  float uTry = pTerm + dTerm + U_BIAS + g_ki * g_integ;
  bool sat = fabsf(uTry) >= 1.0f;
  if (!sat || (uTry * e < 0)) g_integ += e * dt;   // anti-windup: tahan integral saat jenuh
  if (g_ki > 1e-6f) {
    float lim = I_LIMIT / g_ki;
    if (g_integ > lim) g_integ = lim;
    if (g_integ < -lim) g_integ = -lim;
  }
  float u = pTerm + dTerm + U_BIAS + g_ki * g_integ;
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;

  // --- Perintah motor (dihitung selalu, supaya PWM bisa dicetak walau motor tidak digerakkan) ---
  float um = u;
  if (fabsf(um) > 0.001f && DEADBAND > 0.0f) {
    um = (um > 0 ? 1.0f : -1.0f) * (DEADBAND + (1.0f - DEADBAND) * fabsf(um));
  }
  float uMotor = MOTOR_SIGN * um;                                       // -1..+1
  int pwmCmd = (int)(uMotor * PWM_MAX + (uMotor >= 0 ? 0.5f : -0.5f));  // PWM bertanda

  // --- Aktuator ---
#if TELEMETRY_ONLY
  setMotor(0);
#else
  if (g_armed && !g_tripped) {
    setMotor(uMotor);
  } else {
    setMotor(0);
    g_integ = 0;
  }
#endif

  // --- Keluaran serial ---
  uint32_t ms = millis();
#if PRINT_MODE == 0
  if (ms - tTelem >= telemMs) {
    tTelem = ms;
    Serial.printf("T,%lu,%.2f,%.1f,%.2f,%.3f,%d\n",
                  (unsigned long)ms, g_angle, gyro, g_setpoint, u, g_pwmSigned);
  }
#else
  if (ms - tPrint >= printMs) {
    tPrint = ms;

    bool front = ((g_angle > 0) == (FRONT_IS_POSITIVE != 0));
    const char* tilt = (fabsf(g_angle) < 1.0f) ? "TEGAK   " : (front ? "DEPAN   " : "BELAKANG");

    // cek aturan: stick ke DEPAN -> motor CW (dorong), stick ke BELAKANG -> motor CCW (tarik)
    const char* cek = "-";
    if (fabsf(g_angle) >= 2.0f && pwmCmd != 0) {
      bool cw = ((pwmCmd > 0) == (POS_DUTY_IS_CW != 0));
      cek = (front == cw) ? "sesuai" : "KEBALIK";
    }

    const char* st = TELEMETRY_ONLY ? "SIM"
                   : (g_tripped ? "TRIP" : (g_armed ? "AKTIF" : "BELUM ARM"));

    Serial.printf("sudut=%+7.2f %s gyro=%+7.1f set=%+6.2f PID=%+6.3f PWM=%+5d (%+6.1f%%) %s [%s] cek:%s\n",
                  g_angle, tilt, gyro, g_setpoint, u, pwmCmd,
                  100.0f * pwmCmd / PWM_MAX, dirText(pwmCmd), st, cek);
#if SHOW_RAW
    Serial.printf("   raw acc[g] X=%+.2f Y=%+.2f Z=%+.2f | gyro[dps] X=%+.1f Y=%+.1f Z=%+.1f\n",
                  acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
#endif
  }
#endif
}
