/*
  FLYING STICK OTOMATIS
  ESP32-C3 Super Mini
  MPU6050 + DRV8833 + Motor DC Coreless

  SERIAL MONITOR:
  Baud rate : 115200
  Line ending: Newline

  PERINTAH:
  M,400,1          = Tes motor arah 1
  M,400,-1         = Tes motor arah 2
  X                = Stop motor
  S,0.02,0,0.002,0 = Aktifkan PID dengan parameter manual

  PID otomatis aktif setelah kalibrasi MPU6050.
*/

// =====================================================
// LIBRARY
// =====================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>

// =====================================================
// KONFIGURASI PIN
// =====================================================

#define TELEMETRY_ONLY 0

#define PIN_SDA 6
#define PIN_SCL 7

#define PIN_IN1 4
#define PIN_IN2 5

// PWM
#define PWM_FREQ 20000
#define PWM_BITS 10
#define PWM_MAX 1023

// MPU6050
#define MPU_ADDR 0x68

// =====================================================
// KONFIGURASI SENSOR
// =====================================================

// Berdasarkan hasil pengujian MPU6050:
// Pitch = gerakan kanan-kiri

#define AX_ALONG 2
#define AX_SIDE 0
#define AX_GYRO 1

#define ALONG_POSITIVE 1

#define ANGLE_SIGN 1.0f
#define GYRO_SIGN 1.0f

// Jika motor memperbesar kemiringan,
// ubah 1 menjadi -1.
#define MOTOR_SIGN 1.0f

// =====================================================
// PARAMETER PID
// =====================================================

const float LOOP_HZ = 200.0f;
const float TELEM_HZ = 50.0f;

const float ALPHA = 0.995f;

const float TILT_LIMIT_DEG = 45.0f;
const float SP_LIMIT_DEG = 30.0f;

// PID awal
float g_kp = 0.02f;
float g_ki = 0.0f;
float g_kd = 0.002f;

float g_setpoint = 0.0f;

// Integral
const float I_LIMIT = 0.30f;

// Batas keluaran
const float U_BIAS = 0.0f;

// Motor berhenti ketika sudut mendekati tengah
const float ANGLE_DEADBAND_DEG = 1.5f;

// Kalibrasi
const int CAL_SAMPLES = 500;

const float ACC_LSB = 16384.0f;
const float GYRO_LSB = 65.5f;

// =====================================================
// VARIABEL GLOBAL
// =====================================================

float g_integ = 0.0f;

float g_gyroBias = 0.0f;
float g_angle0 = 0.0f;
float g_angle = 0.0f;

bool g_armed = false;
bool g_tripped = false;

bool g_manualTest = false;

int g_manualDuty = 0;
int g_manualDir = 1;

int g_pwmSigned = 0;

bool g_pwmReady = false;
bool g_mpuOk = false;

// =====================================================
// MPU6050
// =====================================================

bool mpuWrite(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(val);

  return Wire.endTransmission() == 0;
}

bool mpuInit() {

  bool ok = true;

  ok &= mpuWrite(0x6B, 0x00);

  delay(100);

  ok &= mpuWrite(0x19, 0x00);
  ok &= mpuWrite(0x1A, 0x03);
  ok &= mpuWrite(0x1B, 0x08);
  ok &= mpuWrite(0x1C, 0x00);

  return ok;
}

bool mpuRead(float acc[3], float gyr[3]) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(
        (uint8_t)MPU_ADDR,
        (uint8_t)14
      ) != 14) {

    return false;
  }

  int16_t r[7];

  for (int i = 0; i < 7; i++) {

    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();

    r[i] = (int16_t)((hi << 8) | lo);
  }

  for (int i = 0; i < 3; i++) {

    acc[i] = r[i] / ACC_LSB;
    gyr[i] = r[i + 4] / GYRO_LSB;
  }

  return true;
}

// =====================================================
// MOTOR DRIVER DRV8833
// =====================================================

#ifdef ESP_ARDUINO_VERSION_MAJOR

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    #define PWM_NEW_API 1
  #endif

#endif

bool pwmInit() {

#ifdef PWM_NEW_API

  bool ok1 = ledcAttach(
    PIN_IN1,
    PWM_FREQ,
    PWM_BITS
  );

  bool ok2 = ledcAttach(
    PIN_IN2,
    PWM_FREQ,
    PWM_BITS
  );

  if (!ok1 || !ok2) {

    Serial.println("# ERROR PWM");

    return false;
  }

#else

  double f1 = ledcSetup(
    0,
    PWM_FREQ,
    PWM_BITS
  );

  double f2 = ledcSetup(
    1,
    PWM_FREQ,
    PWM_BITS
  );

  if (f1 <= 0 || f2 <= 0) {
    return false;
  }

  ledcAttachPin(PIN_IN1, 0);
  ledcAttachPin(PIN_IN2, 1);

#endif

  Serial.println("# PWM SIAP");

  return true;
}

void pwmWrite(int in1, int in2) {

  in1 = constrain(in1, 0, PWM_MAX);
  in2 = constrain(in2, 0, PWM_MAX);

  if (!g_pwmReady) {
    return;
  }

#ifdef PWM_NEW_API

  ledcWrite(PIN_IN1, in1);
  ledcWrite(PIN_IN2, in2);

#else

  ledcWrite(0, in1);
  ledcWrite(1, in2);

#endif
}

// =====================================================
// KONTROL MOTOR
// =====================================================

void setMotor(float u) {

  u = constrain(u, -1.0f, 1.0f);

  int duty =
    (int)(fabsf(u) * PWM_MAX + 0.5f);

  if (duty == 0) {

    pwmWrite(0, 0);

    g_pwmSigned = 0;
  }

  else if (u > 0) {

    pwmWrite(duty, 0);

    g_pwmSigned = duty;
  }

  else {

    pwmWrite(0, duty);

    g_pwmSigned = -duty;
  }
}

// =====================================================
// PERHITUNGAN SUDUT
// =====================================================

float wrap180(float a) {

  while (a > 180.0f) {
    a -= 360.0f;
  }

  while (a < -180.0f) {
    a += 360.0f;
  }

  return a;
}

const float ALONG_SGN =
  ALONG_POSITIVE ? 1.0f : -1.0f;

const float SIDE_SGN =
  ((((AX_GYRO + 1) % 3) == AX_SIDE)
  ? 1.0f : -1.0f) * ALONG_SGN;

float accAngleRaw(const float acc[3]) {

  return atan2f(
    SIDE_SGN * acc[AX_SIDE],
    ALONG_SGN * acc[AX_ALONG]
  ) * 180.0f / PI;
}

// =====================================================
// KALIBRASI MPU6050
// =====================================================

bool calibrate() {

  Serial.println("# KALIBRASI MPU6050");
  Serial.println("# TAHAN STIK TEGAK DAN DIAM");

  float sumAcc[3] = {0, 0, 0};

  float sumGyro = 0;

  int good = 0;

  float acc[3];
  float gyr[3];

  for (int i = 0; i < CAL_SAMPLES; i++) {

    if (mpuRead(acc, gyr)) {

      for (int k = 0; k < 3; k++) {
        sumAcc[k] += acc[k];
      }

      sumGyro += gyr[AX_GYRO];

      good++;
    }

    delay(4);
  }

  if (good < CAL_SAMPLES / 2) {
    return false;
  }

  float avgAcc[3] = {
    sumAcc[0] / good,
    sumAcc[1] / good,
    sumAcc[2] / good
  };

  g_angle0 = accAngleRaw(avgAcc);

  g_gyroBias = sumGyro / good;

  g_angle = 0;

  Serial.printf(
    "# GYRO BIAS: %.3f\n",
    g_gyroBias
  );

  Serial.printf(
    "# SUDUT NOL: %.2f\n",
    g_angle0
  );

  Serial.println("# KALIBRASI SELESAI");

  return true;
}

// =====================================================
// PEMROSESAN PERINTAH SERIAL
// =====================================================

void processLine(const char* s) {

  // STOP

  if (s[0] == 'X' || s[0] == 'x') {

    g_armed = false;
    g_manualTest = false;

    g_manualDuty = 0;
    g_integ = 0;

    setMotor(0);

    Serial.println("# MOTOR STOP");

    return;
  }

  // TES MOTOR MANUAL

  if (
    (s[0] == 'M' || s[0] == 'm') &&
    s[1] == ','
  ) {

    int duty = 0;
    int direction = 0;

    if (
      sscanf(s + 2, "%d,%d", &duty, &direction) == 2 &&
      duty >= 100 &&
      duty <= 700 &&
      (direction == 1 || direction == -1)
    ) {

      if (!g_pwmReady) {

        Serial.println("# PWM BELUM SIAP");

        return;
      }

      g_armed = false;
      g_integ = 0;

      g_manualDuty = duty;
      g_manualDir = direction;

      g_manualTest = true;

      Serial.println("# MOTOR MANUAL AKTIF");
    }

    else {

      Serial.println("# FORMAT: M,400,1");
    }

    return;
  }

  // AKTIFKAN PID MANUAL

  if (
    (s[0] == 'S' || s[0] == 's') &&
    s[1] == ','
  ) {

    float kp, ki, kd, sp;

    if (
      sscanf(s + 2, "%f,%f,%f,%f",
             &kp, &ki, &kd, &sp) == 4 &&

      isfinite(kp) &&
      isfinite(ki) &&
      isfinite(kd) &&
      isfinite(sp) &&

      kp >= 0 &&
      ki >= 0 &&
      kd >= 0 &&

      kp <= 10 &&
      ki <= 10 &&
      kd <= 10 &&

      fabsf(sp) <= SP_LIMIT_DEG
    ) {

      if (!g_mpuOk || !g_pwmReady) {

        Serial.println("# SENSOR ATAU PWM BELUM SIAP");

        return;
      }

      g_kp = kp;
      g_ki = ki;
      g_kd = kd;

      g_setpoint = sp;

      g_manualTest = false;

      g_integ = 0;
      g_tripped = false;
      g_armed = true;

      Serial.println("# PID AKTIF");
    }

    else {

      Serial.println("# PARAMETER PID TIDAK VALID");
    }

    return;
  }

  Serial.println("# PERINTAH TIDAK DIKENAL");
}

// =====================================================
// MEMBACA SERIAL
// =====================================================

void handleSerial() {

  static char buf[64];

  static uint8_t n = 0;

  while (Serial.available()) {

    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {

      if (n > 0) {

        buf[n] = '\0';

        processLine(buf);
      }

      n = 0;
    }

    else if (n < sizeof(buf) - 1) {

      buf[n++] = c;
    }

    else {

      n = 0;
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("# FLYING STICK START");

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  // PWM

  g_pwmReady = pwmInit();

  setMotor(0);

  // MPU6050

  Wire.begin(PIN_SDA, PIN_SCL);

  Wire.setClock(100000);

  if (!mpuInit()) {

    Serial.println("# MPU6050 TIDAK TERDETEKSI");

    g_mpuOk = false;

    return;
  }

  if (!calibrate()) {

    Serial.println("# KALIBRASI GAGAL");

    g_mpuOk = false;

    return;
  }

  g_mpuOk = true;

  // PID OTOMATIS AKTIF

  if (g_pwmReady) {

    g_armed = true;

    Serial.println("# MPU6050 SIAP");
    Serial.println("# PID OTOMATIS AKTIF");

  }

  else {

    g_armed = false;

    Serial.println("# PWM GAGAL");
  }

  Serial.println("# TARGET SUDUT: 0 DERAJAT");
}

// =====================================================
// LOOP UTAMA
// =====================================================

void loop() {

  static uint32_t tPrev = micros();

  static uint32_t tTelem = 0;

  static int failCount = 0;

  const uint32_t loopUs =
    (uint32_t)(1000000.0f / LOOP_HZ);

  const uint32_t telemMs =
    (uint32_t)(1000.0f / TELEM_HZ);

  handleSerial();

  uint32_t now = micros();

  if ((uint32_t)(now - tPrev) < loopUs) {
    return;
  }

  float dt = (now - tPrev) * 1.0e-6f;

  tPrev = now;

  if (dt > 0.05f) {
    dt = 0.05f;
  }

  // =================================================
  // MODE MOTOR MANUAL
  // =================================================

#if !TELEMETRY_ONLY

  if (g_manualTest && !g_tripped) {

    float motorOutput =
      (float)g_manualDuty / (float)PWM_MAX;

    setMotor(g_manualDir * motorOutput);

    return;
  }

#endif

  // =================================================
  // CEK SENSOR
  // =================================================

  if (!g_mpuOk) {

    setMotor(0);

    return;
  }

  float acc[3];
  float gyr[3];

  if (!mpuRead(acc, gyr)) {

    failCount++;

    if (failCount >= 5) {

      g_tripped = true;
      g_armed = false;
      g_manualTest = false;

      setMotor(0);

      Serial.println("# FAULT MPU6050");
    }

    return;
  }

  failCount = 0;

  // =================================================
  // COMPLEMENTARY FILTER
  // =================================================

  float gyro =
    ANGLE_SIGN *
    GYRO_SIGN *
    (gyr[AX_GYRO] - g_gyroBias);

  float accAng =
    ANGLE_SIGN *
    wrap180(accAngleRaw(acc) - g_angle0);

  g_angle =
    ALPHA * (g_angle + gyro * dt) +
    (1.0f - ALPHA) * accAng;

  // =================================================
  // PROTEKSI SUDUT
  // =================================================

  if (
    fabsf(g_angle) > TILT_LIMIT_DEG &&
    !g_tripped
  ) {

    g_tripped = true;
    g_armed = false;

    setMotor(0);

    Serial.println("# TRIP: SUDUT MELEBIHI 45 DERAJAT");
  }

  // =================================================
  // HITUNG PID
  // =================================================

  float error =
    g_setpoint - g_angle;

  float pTerm =
    g_kp * error;

  float dTerm =
    -g_kd * gyro;

  float uTry =
    pTerm +
    dTerm +
    U_BIAS +
    g_ki * g_integ;

  bool sat =
    fabsf(uTry) >= 1.0f;

  if (!sat || (uTry * error < 0)) {

    g_integ += error * dt;
  }

  if (g_ki > 1.0e-6f) {

    float lim = I_LIMIT / g_ki;

    g_integ = constrain(
      g_integ,
      -lim,
      lim
    );
  }

  float u =
    pTerm +
    dTerm +
    U_BIAS +
    g_ki * g_integ;

  u = constrain(u, -1.0f, 1.0f);

  // =================================================
  // AKTUATOR MOTOR
  // =================================================

#if TELEMETRY_ONLY

  setMotor(0);

#else

  if (g_tripped || !g_armed) {

    setMotor(0);
  }

  else {

    // MOTOR MATI DI AREA TENGAH

    if (
      fabsf(error) <= ANGLE_DEADBAND_DEG
    ) {

      setMotor(0);

      g_integ = 0;
    }

    else {

      setMotor(MOTOR_SIGN * u);
    }
  }

#endif

  // =================================================
  // TELEMETRI
  // =================================================

  uint32_t ms = millis();

  if (ms - tTelem >= telemMs) {

    tTelem = ms;

    Serial.printf(
      "T,%lu,%.2f,%.2f,%.2f,%.3f,%d\n",
      (unsigned long)ms,
      g_angle,
      gyro,
      g_setpoint,
      u,
      g_pwmSigned
    );
  }
}