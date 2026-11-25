#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// ToF 사용 여부. 현재 시험에서는 끈다.
#ifndef ROVER_TOF_ENABLED
#define ROVER_TOF_ENABLED 0
#endif

static_assert(
  ROVER_TOF_ENABLED == 0 || ROVER_TOF_ENABLED == 1,
  "ROVER_TOF_ENABLED must be 0 or 1."
);

#ifndef WHEEL_TUNING_VERIFIED
#define WHEEL_TUNING_VERIFIED 1
#endif

static_assert(
  WHEEL_TUNING_VERIFIED == 1,
  "Measure wheel PWM/polarity and set WHEEL_TUNING_VERIFIED=1."
);

// 모터와 엔코더 핀
constexpr int LI1 = 5;
constexpr int LI2 = 3;
constexpr int RI1 = 6;
constexpr int RI2 = 9;
constexpr int LE = 2;
constexpr int RE = 7;

constexpr int LEFT_FORWARD_SIGN = 1;
constexpr int RIGHT_FORWARD_SIGN = -1;

static_assert(
  (LEFT_FORWARD_SIGN == 1 || LEFT_FORWARD_SIGN == -1) &&
  (RIGHT_FORWARD_SIGN == 1 || RIGHT_FORWARD_SIGN == -1),
  "Check wheel polarity."
);

// 사용자 시험 PWM
constexpr int FORWARD_PWM = 150;
constexpr int TURN_PWM = 150;
constexpr int FINE_TURN_PWM = 150;
constexpr int MIN_PWM = 32;
constexpr int MAX_PWM = 150;
constexpr float KP = 5.0f;

constexpr unsigned long HEARTBEAT_MS = 2000;
constexpr unsigned long STALL_MS = 1500;
constexpr unsigned long JOB_MS = 6000;
constexpr unsigned long TOF_MAX_AGE_MS = 150;
constexpr unsigned long CONTROL_MS = 10;
constexpr int TOF_STOP_MM = 120;

static_assert(
  MIN_PWM <= FINE_TURN_PWM &&
  FINE_TURN_PWM <= TURN_PWM &&
  TURN_PWM <= MAX_PWM,
  "Check measured pivot PWM and motor limits."
);

// 엔코더 누적 펄스
volatile unsigned long lc = 0;
volatile unsigned long rc = 0;

// 2ms 미만 간격의 반복 에지를 제외한다.
// 실제 센서 분해능과 최대 회전 속도로 확인할 초기값이다.
constexpr unsigned long ENCODER_MIN_EDGE_US = 2000;
volatile unsigned long lastLeftEdgeUs = 0;
volatile unsigned long lastRightEdgeUs = 0;

void onL() {
  unsigned long now = micros();
  if (now - lastLeftEdgeUs < ENCODER_MIN_EDGE_US) return;
  ++lc;
  lastLeftEdgeUs = now;
}

void onR() {
  unsigned long now = micros();
  if (now - lastRightEdgeUs < ENCODER_MIN_EDGE_US) return;
  ++rc;
  lastRightEdgeUs = now;
}

void snapshot(unsigned long &l, unsigned long &r) {
  noInterrupts();
  l = lc;
  r = rc;
  interrupts();
}

Adafruit_VL53L0X tof;
bool tofStarted = false;
bool rangeValid = false;
unsigned long rangeAt = 0;
unsigned long pollAt = 0;
int rangeMM = 0;

// 0은 정지·완료, 1은 구동 중, 음수는 저장된 고장 상태
int state = 0;
int mode = 0;
int goal = 0;

unsigned long l0 = 0;
unsigned long r0 = 0;
unsigned long prevL = 0;
unsigned long prevR = 0;
unsigned long lastL = 0;
unsigned long lastR = 0;
unsigned long heartbeat = 0;
unsigned long started = 0;
unsigned long controlAt = 0;
unsigned long stoppedAt = 0;

void wheel(int a, int b, int p) {
  p = constrain(p, -255, 255);

  if (p < 0) {
    analogWrite(b, 255);
    analogWrite(a, 255 + p);
  } else {
    analogWrite(a, 255);
    analogWrite(b, 255 - p);
  }
}

void motors(int l, int r) {
  wheel(LI1, LI2, l * LEFT_FORWARD_SIGN);
  wheel(RI1, RI2, r * RIGHT_FORWARD_SIGN);
}

void halt(int reason) {
  motors(0, 0);
  state = reason;
  stoppedAt = millis();
}

void rangePoll() {
  if (!ROVER_TOF_ENABLED) return;
  if (!tofStarted || millis() - pollAt < 10) return;

  pollAt = millis();
  if (!tof.isRangeComplete()) return;

  if (tof.Status != VL53L0X_ERROR_NONE) {
    rangeValid = false;
    return;
  }

  rangeMM = tof.readRangeResult();
  rangeValid =
    tof.Status == VL53L0X_ERROR_NONE &&
    tof.readRangeStatus() == 0 &&
    rangeMM > 0 &&
    rangeMM != 65535;

  rangeAt = millis();
}

int safety() {
  if (!ROVER_TOF_ENABLED) return 0;
  if (!rangeValid || millis() - rangeAt >= TOF_MAX_AGE_MS) return -5;
  if (rangeMM <= TOF_STOP_MM) return -4;
  return 0;
}

// m=1 전진, m=2 시계 회전, m=3 반시계 회전
// n은 각 바퀴가 채울 상승 에지 펄스 수
int roverStep(int m, int n) {
  if (state < 0) return state;

  if (state == 1) {
    halt(-7);
    return state;
  }

  if (m < 1 || m > 3 || n < 1 || n > (m == 1 ? 8 : 2)) {
    halt(-7);
    return state;
  }

  if (millis() - stoppedAt < 200) {
    halt(-8);
    return state;
  }

  int fault = safety();
  if (fault) {
    halt(fault);
    return state;
  }

  snapshot(l0, r0);
  prevL = prevR = 0;
  lastL = lastR = heartbeat = started = millis();

  mode = m;
  goal = n;
  state = 1;
  controlAt = millis() - CONTROL_MS;
  return state;
}

int roverPoll() {
  heartbeat = millis();
  return state;
}

int roverStop() {
  if (state >= 0) {
    halt(0);
  } else {
    motors(0, 0);
  }
  return state;
}

void control() {
  if (state != 1) return;

  unsigned long now = millis();

  if (now - heartbeat > HEARTBEAT_MS) {
    halt(-3);
    return;
  }

  int fault = safety();
  if (fault) {
    halt(fault);
    return;
  }

  if (now - started > JOB_MS) {
    halt(-6);
    return;
  }

  if (now - controlAt < CONTROL_MS) return;
  controlAt = now;

  unsigned long l, r;
  snapshot(l, r);
  l -= l0;
  r -= r0;

  if (l != prevL) {
    lastL = now;
    prevL = l;
  }

  if (r != prevR) {
    lastR = now;
    prevR = r;
  }

  if ((l < (unsigned long)goal && now - lastL > STALL_MS) ||
      (r < (unsigned long)goal && now - lastR > STALL_MS)) {
    halt(-9);
    return;
  }

  // 양쪽 목표 도달을 먼저 확인한다.
  if (l >= (unsigned long)goal && r >= (unsigned long)goal) {
    halt(0);
    return;
  }

  // 목표를 넘긴 펄스는 좌우 진행 오차에서 제외한다.
  unsigned long rawL = l;
  unsigned long rawR = r;
  l = min(l, (unsigned long)goal);
  r = min(r, (unsigned long)goal);

  if (abs((long)l - (long)r) > 3) {
    halt(-10);
    Monitor.print("ENC mismatch L=");
    Monitor.print(rawL);
    Monitor.print(" R=");
    Monitor.println(rawR);
    return;
  }

  int base = mode == 1 ? FORWARD_PWM : TURN_PWM;

  if (mode == 1 && goal - (int)min(l, r) <= 2) {
    base = TURN_PWM;
  }

  int limit = MAX_PWM;
  if (mode != 1 && goal == 1) {
    base = FINE_TURN_PWM;
    limit = FINE_TURN_PWM;
  }

  int correction = lroundf(KP * ((long)l - (long)r));

  // 목표를 채운 바퀴는 먼저 제동한다.
  int lp = l >= (unsigned long)goal
    ? 0
    : constrain(base - correction, MIN_PWM, limit);

  int rp = r >= (unsigned long)goal
    ? 0
    : constrain(base + correction, MIN_PWM, limit);

  if (mode == 2) rp = -rp;
  if (mode == 3) lp = -lp;

  motors(lp, rp);
}

void setup() {


  const int pins[] = {LI1, LI2, RI1, RI2};
  for (int pin : pins) {
    pinMode(pin, OUTPUT);
  }

  analogWriteResolution(8);
  motors(0, 0);

  pinMode(LE, INPUT);
  pinMode(RE, INPUT);
  attachInterrupt(digitalPinToInterrupt(LE), onL, RISING);
  attachInterrupt(digitalPinToInterrupt(RE), onR, RISING);

  if (ROVER_TOF_ENABLED) {
    tofStarted = tof.begin() && tof.startRangeContinuous(50);
  }

  stoppedAt = millis();
  Bridge.begin();
  Monitor.begin();
  Bridge.provide_safe("encoder_step", roverStep);
  Bridge.provide_safe("encoder_poll", roverPoll);
  Bridge.provide_safe("encoder_stop", roverStop);
}

void loop(){
  control();
  rangePoll();
  control();
  delay(1);
}