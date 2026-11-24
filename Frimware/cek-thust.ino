
#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

// =============================================
// ESP32-C3 SUPER MINI
// =============================================

#define PIN_IN1 4
#define PIN_IN2 5

// =============================================
// PWM CONFIGURATION
// =============================================

#define PWM_FREQ 20000
#define PWM_BITS 10
#define PWM_MAX 1023

// Batas maksimum PWM
#define MAX_TEST_PERCENT 100

// =============================================
// GLOBAL VARIABLES
// =============================================

bool pwmReady = false;

int currentPWM = 0;
float currentPercent = 0;

// =============================================
// ESP32 PWM API VERSION
// =============================================

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define PWM_NEW_API 1
#else
#define PWM_NEW_API 0
#endif

// =============================================
// PWM INITIALIZATION
// =============================================

bool setupPWM() {

#if PWM_NEW_API

  bool ok1 = ledcAttach(PIN_IN1, PWM_FREQ, PWM_BITS);
  bool ok2 = ledcAttach(PIN_IN2, PWM_FREQ, PWM_BITS);

  if (!ok1 || !ok2) {
    return false;
  }

#else

  double f1 = ledcSetup(0, PWM_FREQ, PWM_BITS);
  double f2 = ledcSetup(1, PWM_FREQ, PWM_BITS);

  if (f1 <= 0 || f2 <= 0) {
    return false;
  }

  ledcAttachPin(PIN_IN1, 0);
  ledcAttachPin(PIN_IN2, 1);

#endif

  return true;
}

// =============================================
// WRITE PWM
// =============================================

void writePWM(int in1, int in2) {

  in1 = constrain(in1, 0, PWM_MAX);
  in2 = constrain(in2, 0, PWM_MAX);

#if PWM_NEW_API

  ledcWrite(PIN_IN1, in1);
  ledcWrite(PIN_IN2, in2);

#else

  ledcWrite(0, in1);
  ledcWrite(1, in2);

#endif

}

// =============================================
// MOTOR CONTROL
// =============================================

void setMotorPercent(float percent) {

  percent = constrain(
    percent,
    -(float)MAX_TEST_PERCENT,
    (float)MAX_TEST_PERCENT
  );

  // Convert percentage to PWM
  int pwm = roundf(
    fabsf(percent) * PWM_MAX / 100.0f
  );

  // Stop briefly before changing direction
  if ((percent > 0 && currentPercent < 0) ||
      (percent < 0 && currentPercent > 0)) {

    writePWM(0, 0);
    delay(200);
  }

  // Forward
  if (percent > 0) {

    writePWM(pwm, 0);

  }

  // Reverse
  else if (percent < 0) {

    writePWM(0, pwm);

  }

  // Stop
  else {

    writePWM(0, 0);

  }

  currentPWM = pwm;
  currentPercent = percent;

  // =========================================
  // SERIAL MONITOR
  // =========================================

  Serial.println();
  Serial.println("================================");

  Serial.print("PWM Command : ");
  Serial.print(percent, 1);
  Serial.println(" %");

  Serial.print("PWM Value   : ");
  Serial.print(currentPWM);
  Serial.print(" / ");
  Serial.println(PWM_MAX);

  Serial.print("Duty Cycle  : ");
  Serial.print(
    currentPWM * 100.0f / PWM_MAX,
    2
  );
  Serial.println(" %");

  Serial.print("Direction   : ");

  if (percent > 0) {
    Serial.println("FORWARD");
  }
  else if (percent < 0) {
    Serial.println("REVERSE");
  }
  else {
    Serial.println("STOP");
  }

  Serial.println("================================");
}

// =============================================
// STOP MOTOR
// =============================================

void stopMotor() {

  writePWM(0, 0);

  currentPWM = 0;
  currentPercent = 0;

  Serial.println("MOTOR STOP");
}

// =============================================
// SETUP
// =============================================

void setup() {

  Serial.begin(115200);
  delay(500);

  Serial.setTimeout(100);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  pwmReady = setupPWM();

  if (!pwmReady) {

    Serial.println("ERROR: PWM FAILED");

    return;
  }

  stopMotor();

  Serial.println();
  Serial.println("================================");
  Serial.println("       MOTOR THRUST TEST");
  Serial.println("================================");

  Serial.println("BOARD       : ESP32-C3");
  Serial.println("DRIVER      : DRV8833");
  Serial.println("MOTOR       : CORELESS 8520");
  Serial.println("PWM FREQ    : 20 kHz");
  Serial.println("PWM BITS    : 10 bit");
  Serial.println("PWM RANGE   : 0-1023");

  Serial.println();
  Serial.println("SERIAL COMMAND:");
  Serial.println("  0   = STOP");
  Serial.println("  10  = FORWARD 10%");
  Serial.println("  25  = FORWARD 25%");
  Serial.println("  50  = FORWARD 50%");
  Serial.println("  75  = FORWARD 75%");
  Serial.println("  100 = FORWARD 100%");
  Serial.println(" -50  = REVERSE 50%");

  Serial.println();
  Serial.println("READY");
  Serial.println("================================");
}

// =============================================
// LOOP
// =============================================

void loop() {

  if (!pwmReady) {
    return;
  }

  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      return;
    }

    char* endPtr;

    long value = strtol(
      input.c_str(),
      &endPtr,
      10
    );

    if (endPtr == input.c_str() || *endPtr != '\0') {

      Serial.println("ERROR: INPUT HARUS ANGKA");

      return;
    }

    if (value < -100 || value > 100) {

      Serial.println("ERROR: RANGE -100 SAMPAI 100");

      return;
    }

    setMotorPercent((float)value);
  }

}
