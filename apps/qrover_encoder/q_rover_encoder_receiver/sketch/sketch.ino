#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// 바퀴를 띄운 시험과 PWM 실측을 마친 뒤에만 1로 변경한다.
#ifndef WHEEL_TUNING_VERIFIED
#define WHEEL_TUNING_VERIFIED 1
#endif
static_assert(WHEEL_TUNING_VERIFIED == 1, "Measure wheel PWM/polarity and set WHEEL_TUNING_VERIFIED=1.");

// UNO Q + DRV8833. Test motor polarity with wheels lifted first.
constexpr int LI1=5, LI2=3, RI1=6, RI2=9;
constexpr int LE=2, RE=7;
constexpr int LEFT_FORWARD_SIGN=1, RIGHT_FORWARD_SIGN=1;
static_assert((LEFT_FORWARD_SIGN == 1 || LEFT_FORWARD_SIGN == -1) &&
              (RIGHT_FORWARD_SIGN == 1 || RIGHT_FORWARD_SIGN == -1), "Check wheel polarity.");
constexpr int FORWARD_PWM=55, TURN_PWM=42, FINE_TURN_PWM=32, MIN_PWM=32, MAX_PWM=70;
constexpr float KP=5.0f;
constexpr unsigned long HEARTBEAT_MS=400, STALL_MS=1500, JOB_MS=6000;
constexpr unsigned long TOF_MAX_AGE_MS=150, CONTROL_MS=10;
constexpr int TOF_STOP_MM=120;
static_assert(MIN_PWM <= FINE_TURN_PWM && FINE_TURN_PWM <= TURN_PWM && TURN_PWM <= MAX_PWM,
              "Check measured pivot PWM and motor limits.");
volatile unsigned long lc=0,rc=0;
void onL(){++lc;}
void onR(){++rc;}
void snapshot(unsigned long &l,unsigned long &r){noInterrupts();l=lc;r=rc;interrupts();}
Adafruit_VL53L0X tof;
bool tofStarted=false,rangeValid=false;
unsigned long rangeAt=0,pollAt=0;
int rangeMM=0;
// state: 0 idle/done, 1 running; negative values are latched faults.
int state=0,mode=0,goal=0;
unsigned long l0=0,r0=0,prevL=0,prevR=0,lastL=0,lastR=0;
unsigned long heartbeat=0,started=0,controlAt=0,stoppedAt=0;
void wheel(int a,int b,int p){
  p=constrain(p,-255,255);
  if(p<0){analogWrite(b,255);analogWrite(a,255+p);}
  else{analogWrite(a,255);analogWrite(b,255-p);}
}
void motors(int l,int r){wheel(LI1,LI2,l*LEFT_FORWARD_SIGN);wheel(RI1,RI2,r*RIGHT_FORWARD_SIGN);}
void halt(int reason){motors(0,0);state=reason;stoppedAt=millis();}
void rangePoll(){
  if(!tofStarted||millis()-pollAt<10)return;
  pollAt=millis();
  if(!tof.isRangeComplete())return;
  if(tof.Status!=VL53L0X_ERROR_NONE){rangeValid=false;return;}
  rangeMM=tof.readRangeResult();
  rangeValid=tof.Status==VL53L0X_ERROR_NONE&&tof.readRangeStatus()==0&&rangeMM>0&&rangeMM!=65535;
  rangeAt=millis();
}
int safety(){
  if(!rangeValid||millis()-rangeAt>=TOF_MAX_AGE_MS)return -5;
  if(rangeMM<=TOF_STOP_MM)return -4;
  return 0;
}
// m=1 forward, m=2 clockwise pivot, m=3 counterclockwise pivot.
// n is rising-edge pulse target PER WHEEL, never a signed encoder count.
int roverStep(int m,int n){
  if(state<0)return state;
  if(state==1){halt(-7);return state;} // 중복 작업도 즉시 제동하고 오류를 유지한다.
  if(m<1||m>3||n<1||n>(m==1?8:2)){halt(-7);return state;}
  if(millis()-stoppedAt<200){halt(-8);return state;}
  int fault=safety();if(fault){halt(fault);return state;}
  snapshot(l0,r0);prevL=prevR=0;
  lastL=lastR=heartbeat=started=millis();
  mode=m;goal=n;state=1;controlAt=millis()-CONTROL_MS;
  return state;
}
int roverPoll(){heartbeat=millis();return state;}
int roverStop(){if(state>=0)halt(0);else motors(0,0);return state;}
void control(){
  if(state!=1)return;
  unsigned long now=millis();
  if(now-heartbeat>HEARTBEAT_MS){halt(-3);return;}
  int fault=safety();if(fault){halt(fault);return;}
  if(now-started>JOB_MS){halt(-6);return;}
  if(now-controlAt<CONTROL_MS)return;
  controlAt=now;
  unsigned long l,r;snapshot(l,r);l-=l0;r-=r0;
  if(l!=prevL){lastL=now;prevL=l;}
  if(r!=prevR){lastR=now;prevR=r;}
  if((l<(unsigned long)goal&&now-lastL>STALL_MS)||(r<(unsigned long)goal&&now-lastR>STALL_MS)){halt(-9);return;}
  if(abs((long)l-(long)r)>3){halt(-10);return;}
  if(l>=(unsigned long)goal&&r>=(unsigned long)goal){halt(0);return;}
  int base=mode==1?FORWARD_PWM:TURN_PWM;
  // 직진 끝에서는 감속하고, 1펄스 미세 회전은 낮은 출력으로 제한한다.
  if(mode==1 && goal-(int)min(l,r)<=2)base=TURN_PWM;
  int limit=MAX_PWM;
  if(mode!=1 && goal==1){
    base=FINE_TURN_PWM;
    limit=FINE_TURN_PWM;
  }
  int correction=lroundf(KP*((long)l-(long)r));
  int lp=l>=(unsigned long)goal?0:constrain(base-correction,MIN_PWM,limit);
  int rp=r>=(unsigned long)goal?0:constrain(base+correction,MIN_PWM,limit);
  // Match ABSOLUTE pulse progress during pivot, then apply opposite directions.
  if(mode==2)rp=-rp;
  if(mode==3)lp=-lp;
  motors(lp,rp);
}
void setup(){
  const int pins[]={LI1,LI2,RI1,RI2};
  for(int pin:pins)pinMode(pin,OUTPUT);
  analogWriteResolution(8);motors(0,0);
  pinMode(LE,INPUT);pinMode(RE,INPUT);
  attachInterrupt(digitalPinToInterrupt(LE),onL,RISING);
  attachInterrupt(digitalPinToInterrupt(RE),onR,RISING);
  tofStarted=tof.begin()&&tof.startRangeContinuous(50);
  stoppedAt=millis();
  Bridge.begin();
  Bridge.provide_safe("encoder_step",roverStep);
  Bridge.provide_safe("encoder_poll",roverPoll);
  Bridge.provide_safe("encoder_stop",roverStop);
}
void loop(){control();rangePoll();control();delay(1);}
