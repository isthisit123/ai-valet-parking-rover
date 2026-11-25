"""실제 스케치를 호스트 C++로 컴파일해 안전정지 상태 전이를 시험한다.

Arduino/ToF/Bridge API만 모의 구현한다. UNO Q 빌드와 배선 검증은 별도다.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from receiver_source import read_receiver

ARDUINO = r'''
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
using std::min;
using std::abs;
constexpr double PI=3.14159265358979323846;
constexpr int OUTPUT=1,INPUT=0,RISING=1;
inline unsigned long mockNow=1000;
inline unsigned long mockMicros=1000000;
inline int outputs[32]={};
inline unsigned long millis(){return mockNow;}
inline unsigned long micros(){return mockMicros;}
inline void analogWrite(int p,int v){outputs[p]=v;}
inline void pinMode(int,int){}
inline void analogWriteResolution(int){}
inline void noInterrupts(){}
inline void interrupts(){}
inline int digitalPinToInterrupt(int p){return p;}
inline int interruptCalls=0;
inline void attachInterrupt(int,void(*)(),int){++interruptCalls;}
inline void delay(int){}
#define constrain(v,a,b) ((v)<(a)?(a):((v)>(b)?(b):(v)))
'''
BRIDGE = r'''
#pragma once
struct MockBridge{void begin(){} template<class T>void provide_safe(const char*,T){}};
inline MockBridge Bridge;
struct MockMonitor{void begin(){} template<class T>void print(T){} template<class T>void println(T){}};
inline MockMonitor Monitor;
'''
TOF = r'''
#pragma once
#define VL53L0X_ERROR_NONE 0
inline int tofBeginCalls=0,tofReadCalls=0;
struct Adafruit_VL53L0X{
 int Status=0;
 bool begin(){++tofBeginCalls;return true;} bool startRangeContinuous(int){return true;}
 bool isRangeComplete(){++tofReadCalls;return true;} int readRangeResult(){return 500;}
 int readRangeStatus(){return 0;}
};
'''
HARNESS = r'''
#include <cstdio>
#include "sketch.ino"
int passed=0;
#define CHECK(x) do{if(!(x)){std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1;}++passed;}while(0)
void reset(){
 mockNow=1000;lc=rc=0;state=0;stoppedAt=0;
 mockMicros=1000000;lastLeftEdgeUs=lastRightEdgeUs=0;
 rangeValid=true;rangeAt=mockNow;rangeMM=500;
 mode=goal=0;controlAt=0;motors(0,0);
}
bool braked(){return outputs[LI1]==255&&outputs[LI2]==255&&outputs[RI1]==255&&outputs[RI2]==255;}
int leftPower(){return (outputs[LI1]-outputs[LI2])*LEFT_FORWARD_SIGN;}
int rightPower(){return (outputs[RI1]-outputs[RI2])*RIGHT_FORWARD_SIGN;}
int main(){
 setup();CHECK(tofBeginCalls==ROVER_TOF_ENABLED);CHECK(interruptCalls==2);
 mockNow+=10;rangePoll();CHECK(tofReadCalls==ROVER_TOF_ENABLED);
 reset();CHECK(roverStep(1,5)==1);control();CHECK(leftPower()>0&&rightPower()>0);
 lc=rc=5;controlAt=0;control();CHECK(state==0&&braked());
 reset();roverStep(1,5);mockNow+=HEARTBEAT_MS+1;rangeAt=mockNow;control();CHECK(state==-3&&braked());
 CHECK(roverStop()==-3);CHECK(roverStep(1,1)==-3);
#if ROVER_TOF_ENABLED
 reset();roverStep(1,5);rangeMM=120;control();CHECK(state==-4&&braked());
 reset();roverStep(1,5);mockNow+=150;control();CHECK(state==-5&&braked());
 reset();roverStep(1,5);rangeValid=false;control();CHECK(state==-5&&braked());
#else
 reset();rangeValid=false;rangeAt=0;rangeMM=0;CHECK(roverStep(1,5)==1);
 control();CHECK(state==1&&!braked());CHECK(roverStop()==0&&braked());
 reset();rangeMM=1;CHECK(roverStep(1,5)==1);control();CHECK(state==1&&!braked());
 mockNow+=150;control();CHECK(state==1&&!braked());roverStop();
#endif
 reset();roverStep(1,5);mockNow+=STALL_MS+1;rangeAt=heartbeat=mockNow;control();CHECK(state==-9&&braked());
 reset();roverStep(1,5);lc=4;controlAt=0;control();CHECK(state==-10&&braked());
 reset();roverStep(2,1);rc=8;control();CHECK(state==1);
 CHECK(rightPower()==0&&leftPower()>0);
 lc=1;controlAt=0;control();CHECK(state==0&&braked());
 reset();roverStep(3,1);lc=8;control();CHECK(state==1);
 CHECK(leftPower()==0&&rightPower()>0);
 mockNow+=STALL_MS+1;rangeAt=heartbeat=mockNow;control();CHECK(state==-9&&braked());
 reset();roverStep(1,5);mockNow+=JOB_MS+1;rangeAt=heartbeat=mockNow;control();CHECK(state==-6&&braked());
 reset();roverStep(1,5);CHECK(roverStep(1,2)==-7&&braked());CHECK(roverPoll()==-7);
 reset();CHECK(roverStep(9,1)==-7&&braked());
 reset();CHECK(roverStep(1,9)==-7&&braked());
 reset();CHECK(roverStep(2,3)==-7&&braked());
 reset();CHECK(roverStep(3,0)==-7&&braked());
 reset();stoppedAt=mockNow;CHECK(roverStep(1,1)==-8&&braked());
 reset();CHECK(roverStep(2,1)==1);control();CHECK(leftPower()>0&&rightPower()<0);
 reset();CHECK(roverStep(3,1)==1);control();CHECK(leftPower()<0&&rightPower()>0);
 reset();onL();onR();unsigned long l,r;snapshot(l,r);CHECK(l==1&&r==1);
 onL();onR();snapshot(l,r);CHECK(l==1&&r==1);
 mockMicros+=ENCODER_MIN_EDGE_US-1;onL();onR();snapshot(l,r);CHECK(l==1&&r==1);
 mockMicros+=1;onL();onR();snapshot(l,r);CHECK(l==2&&r==2);
 lastLeftEdgeUs=0xFFFFFF00UL;mockMicros=1000;onL();CHECK(lc==2);
 mockMicros=2000;onL();CHECK(lc==3);
 reset();roverStep(1,5);lc=1;controlAt=0;control();
 CHECK(leftPower()==FORWARD_PWM-(int)KP&&rightPower()==MAX_PWM);
 reset();roverStep(2,1);control();CHECK(leftPower()==FINE_TURN_PWM&&rightPower()==-FINE_TURN_PWM);
 reset();roverStep(1,5);lc=5;rc=4;controlAt=0;control();CHECK(leftPower()==0&&rightPower()>0);
 CHECK(roverStop()==0&&braked());
 std::printf("firmware checks passed: %d (ToF %s, encoder enabled)\n",passed,ROVER_TOF_ENABLED?"enabled":"disabled");return 0;
}
'''



def compiler():
    env = os.environ.copy()
    for name in ('g++', 'clang++'):
        path = shutil.which(name)
        if path:
            return [path], env, False
    vsroot = Path(os.environ.get('ProgramFiles', r'C:\Program Files')) / 'Microsoft Visual Studio'
    matches = sorted(vsroot.glob('*/Community/VC/Tools/MSVC/*/bin/Hostx64/x64/cl.exe'))
    if matches:
        exe = matches[-1]
        version = exe.parents[3]
        kits = Path(os.environ.get('ProgramFiles(x86)', r'C:\Program Files (x86)')) / 'Windows Kits/10'
        versions = sorted((kits / 'Include').glob('*'))
        if not versions:
            raise RuntimeError('Windows SDK가 필요합니다.')
        sdk = versions[-1].name
        env['INCLUDE'] = os.pathsep.join(str(p) for p in
            (version / 'include', kits / 'Include' / sdk / 'ucrt', kits / 'Include' / sdk / 'shared', kits / 'Include' / sdk / 'um'))
        env['LIB'] = os.pathsep.join(str(p) for p in
            (version / 'lib/x64', kits / 'Lib' / sdk / 'ucrt/x64', kits / 'Lib' / sdk / 'um/x64'))
        return [str(exe)], env, True
    raise RuntimeError('C++ 컴파일러를 찾지 못했습니다. Visual Studio C++ 또는 g++/clang++를 설치하세요.')


def main():
    command, env, msvc = compiler()
    with tempfile.TemporaryDirectory(prefix='rover_firmware_') as temporary:
        folder = Path(temporary)
        for name, text in (('Arduino.h', ARDUINO), ('Arduino_RouterBridge.h', BRIDGE),
                           ('Adafruit_VL53L0X.h', TOF), ('sketch.ino', read_receiver('sketch/sketch.ino')),
                           ('test.cpp', HARNESS)):
            (folder / name).write_text(text, encoding='utf-8')
        if msvc:
            args = ['/nologo', '/EHsc', '/std:c++17', '/utf-8', '/I.', '/DWHEEL_TUNING_VERIFIED=1',
                    'test.cpp', '/Fe:test.exe']
            executable = folder / 'test.exe'
        else:
            args = ['-std=c++17', '-I.', '-DWHEEL_TUNING_VERIFIED=1', 'test.cpp', '-o', 'test.exe']
            executable = folder / 'test.exe'
        flag = '/D' if msvc else '-D'
        for options in ([], [flag + 'ROVER_TOF_ENABLED=1']):
            built = subprocess.run([*command, *args, *options], cwd=folder, env=env, capture_output=True, text=True)
            if built.returncode:
                raise RuntimeError(built.stdout + built.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            print(result.stdout, end='')
        # 작업자가 저장한 현재 값과 무관하게 미승인(0) 상태의 빌드 차단을 검증한다.
        without_flag = [arg.replace('WHEEL_TUNING_VERIFIED=1', 'WHEEL_TUNING_VERIFIED=0') for arg in args]
        blocked = subprocess.run([*command, *without_flag], cwd=folder, env=env, capture_output=True, text=True)
        if blocked.returncode == 0:
            raise AssertionError('미보정 펌웨어 빌드가 차단되지 않았습니다.')
        print('uncalibrated firmware build: blocked as expected')


if __name__ == '__main__':
    main()
