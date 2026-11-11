#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// HYPER Q-ROVER 1차시 배선: AIN1/D5, AIN2/D3, BIN1/D6, BIN2/D9.
constexpr int LEFT_IN1 = 5;
constexpr int LEFT_IN2 = 3;
constexpr int RIGHT_IN1 = 6;
constexpr int RIGHT_IN2 = 9;
constexpr int LEFT_FORWARD_SIGN = 1;
constexpr int RIGHT_FORWARD_SIGN = 1;
static_assert((LEFT_FORWARD_SIGN == 1 || LEFT_FORWARD_SIGN == -1) &&
              (RIGHT_FORWARD_SIGN == 1 || RIGHT_FORWARD_SIGN == -1), "Invalid wheel polarity.");

constexpr float TRACK_WIDTH_CM = 12.0f;
constexpr int LEFT_PWM_AT_8_CM_S = 0;
constexpr int RIGHT_PWM_AT_8_CM_S = 0;
constexpr int MAX_PWM = 75;
constexpr int TOF_STOP_MM = 120;
constexpr unsigned long TOF_MAX_AGE_MS = 150;
constexpr unsigned long COMMAND_TIMEOUT_MS = 300;

static_assert(TRACK_WIDTH_CM > 0 && LEFT_PWM_AT_8_CM_S > 0 && RIGHT_PWM_AT_8_CM_S > 0 &&
              LEFT_PWM_AT_8_CM_S <= 255 && RIGHT_PWM_AT_8_CM_S <= 255,
              "Set the measured wheel track and wheel PWM values.");
static_assert(MAX_PWM > 0 && MAX_PWM <= 255, "Invalid PWM limit.");

Adafruit_VL53L0X tof;
bool tofStarted = false;
bool rangeValid = false;
uint16_t distanceMm = 0;
unsigned long lastRangeMs = 0;
unsigned long lastPollMs = 0;
unsigned long lastDriveMs = 0;
bool driving = false;

enum ReplyCode {
  STOP_APPLIED = 0,
  DRIVE_APPLIED = 1,
  COMMAND_TIMEOUT = 3,
  TOF_OBSTACLE = 4,
  TOF_UNAVAILABLE = 5,
  STOP_LATCHED = 6,
  INVALID_COMMAND = 7,
};

void writeWheel(int in1, int in2, int pwm) {
  // 느린 감쇄: 구동 구간 사이에 (HIGH, HIGH)로 브레이크한다.
  if (pwm < 0) {
    analogWrite(in2, 255);
    analogWrite(in1, 255 + pwm);
  } else {
    analogWrite(in1, 255);
    analogWrite(in2, 255 - pwm);
  }
}

void brake() {
  writeWheel(LEFT_IN1, LEFT_IN2, 0);
  writeWheel(RIGHT_IN1, RIGHT_IN2, 0);
}

ReplyCode latchedReason = STOP_APPLIED;

void halt(ReplyCode reason) {
  brake();
  driving = false;
  latchedReason = reason;
}

ReplyCode checkSafety(unsigned long now) {
  if (latchedReason != STOP_APPLIED) {
    return latchedReason;
  }
  ReplyCode reason = STOP_APPLIED;
  if (driving && now - lastDriveMs >= COMMAND_TIMEOUT_MS) {
    reason = COMMAND_TIMEOUT;
  } else if (!rangeValid || now - lastRangeMs >= TOF_MAX_AGE_MS) {
    reason = TOF_UNAVAILABLE;
  } else if (distanceMm <= TOF_STOP_MM) {
    reason = TOF_OBSTACLE;
  }
  if (driving && reason != STOP_APPLIED) {
    halt(reason);
  }
  return reason;
}

void pollRange(unsigned long now) {
  if (!tofStarted || now - lastPollMs < 10) {
    return;
  }
  lastPollMs = now;
  if (!tof.isRangeComplete()) {
    return;
  }
  if (tof.Status != VL53L0X_ERROR_NONE) {
    rangeValid = false;
    return;
  }
  distanceMm = tof.readRangeResult();
  rangeValid = tof.Status == VL53L0X_ERROR_NONE && tof.readRangeStatus() == 0 &&
               distanceMm != 0 && distanceMm != 0xffff;
  lastRangeMs = millis();
}

int roverStop() {
  if (driving) {
    halt(STOP_LATCHED);
  } else {
    brake();
  }
  return STOP_APPLIED;
}

int roverDrive(int linearMilli, int angularMilli) {
  if (linearMilli < 0 || linearMilli > 8000 || angularMilli < -45000 || angularMilli > 45000) {
    roverStop();
    return INVALID_COMMAND;
  }
  const unsigned long now = millis();
  const ReplyCode safety = checkSafety(now);
  if (safety != STOP_APPLIED) {
    return safety;
  }

  const float linear = linearMilli / 1000.0f;
  const float turn = angularMilli * (PI / 180000.0f) * TRACK_WIDTH_CM / 2.0f;
  // 시계 방향이 양수이므로 우회전 시 왼쪽 바퀴가 더 빠르다.
  float left = (linear + turn) * LEFT_PWM_AT_8_CM_S / 8.0f;
  float right = (linear - turn) * RIGHT_PWM_AT_8_CM_S / 8.0f;
  const float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > MAX_PWM) {
    left *= MAX_PWM / peak;
    right *= MAX_PWM / peak;
  }
  writeWheel(LEFT_IN1, LEFT_IN2, LEFT_FORWARD_SIGN * lroundf(left));
  writeWheel(RIGHT_IN1, RIGHT_IN2, RIGHT_FORWARD_SIGN * lroundf(right));
  driving = driving || linearMilli != 0 || angularMilli != 0;
  lastDriveMs = now;
  return DRIVE_APPLIED;
}

void setup() {
  const int motorPins[] = {LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2};
  for (int pin : motorPins) {
    pinMode(pin, OUTPUT);
  }
  analogWriteResolution(8);
  brake();
  tofStarted = tof.begin() && tof.startRangeContinuous(50);
  Bridge.begin();
  Bridge.provide_safe("rover_drive", roverDrive);
  Bridge.provide_safe("rover_stop", roverStop);
}

void loop() {
  checkSafety(millis());
  pollRange(millis());
  checkSafety(millis());
  delay(2);
}
