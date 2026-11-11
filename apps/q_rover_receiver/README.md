# Q-ROVER Receiver

노트북의 TCP 속도 명령을 UNO Q의 Linux Python이 받고, Bridge로 STM32 MCU에 전달합니다. 처음 가져온 앱은 모터 출력이 꺼져 있습니다. 카메라, YOLO, 경로 파일 없이 명령 전달과 응답부터 확인할 수 있습니다.

## 1. App Lab에 넣기

1. App Lab에서 UNO Q를 선택합니다. USB-C 데이터 케이블로 직접 연결하거나, 두 기기를 같은 Wi-Fi에 연결합니다.
2. 저장소 루트 PowerShell에서 아래 명령으로 App Lab용 ZIP을 만들거나, 이미 만든 `q_rover_receiver.zip`을 사용합니다.

   ```powershell
   Compress-Archive -Path .\apps\q_rover_receiver -DestinationPath .\q_rover_receiver.zip
   ```

3. App Lab에서 **Apps → Create new App + → Import or install App**을 열고 ZIP을 선택합니다. **Q-ROVER Receiver** 앱이 생기면 엽니다.
4. 파일 트리에 `python/main.py`와 `sketch/sketch.ino`가 보이는지 확인합니다. TCP 수신 코드는 **Python**, Bridge 응답과 모터 코드는 **Sketch**에 들어갑니다. `app.yaml`은 TCP 8765 포트를 공개하고 `sketch/sketch.yaml`은 스케치 라이브러리를 지정합니다. ZIP으로 가져오면 네 파일이 각각 제자리에 들어갑니다.
5. 이전 버전의 수신 앱이 실행 중이면 **Stop**으로 종료합니다. 새 앱의 `sketch/sketch.ino`에서 `#define OUTPUT_MODE 0`을 확인하고 **Run**을 누릅니다. 하단 **Console → Python**에 `LISTEN TCP 8765 mode=dry_run`이 나오면 수신 준비가 된 상태입니다. 시작 실패 메시지는 **App launch**에서 확인합니다.

[Arduino App Lab 가져오기](https://docs.arduino.cc/software/app-lab/apps/manage-apps) · [실행과 콘솔](https://docs.arduino.cc/software/app-lab/apps/run)

## 2. 노트북에서 전송 확인

### USB-C 유선 연결

1. UNO Q를 **USB-C 데이터 케이블로 노트북에 직접 연결**합니다. App Lab에서 USB로 연결된 보드를 선택한 뒤 수신 앱을 실행합니다. 통신만 확인할 때는 모터 전원을 끄고 `LISTEN TCP 8765 mode=dry_run`을 확인합니다.
2. 노트북의 PowerShell에서 `adb -d get-state`를 실행합니다. `device`가 나오면 USB로 인식된 상태입니다. ADB가 PATH에 없으면 아래처럼 설치된 실행 파일을 직접 지정합니다. 이 경로는 현재 개발 노트북의 예시입니다.

   ```powershell
   $adb = 'C:\Users\nam04\AppData\Local\Arduino15\packages\arduino\tools\adb\32.0.0\adb.exe'
   & $adb -d get-state
   ```

3. 저장소 루트에서 통신 시험을 실행합니다.

   ```powershell
   .\.venv\Scripts\python.exe -m src.rover_control.probe --usb --adb $adb
   ```

   ADB가 PATH에 있으면 `--adb $adb`를 생략합니다. 프로그램이 노트북의 빈 로컬 포트를 UNO Q의 TCP 8765로 전달하고 종료 시 정리합니다. `TCP 연결됨: 127.0.0.1:...` 뒤에 `통신 성공`이 나오면 MCU까지 왕복한 것입니다. 연결이 실패하면 데이터 케이블과 App Lab의 USB 연결 상태를 먼저 확인합니다. `device`인데 응답이 없으면 App Lab **Python** 탭의 `LISTEN`과 `SESSION CLOSED`를 확인합니다.
4. 바퀴를 띄운 시험에서는 앱을 `OUTPUT_MODE 2`로 실행하고 같은 명령에 `--bench-drive`를 붙입니다. 경로 주행 명령에서도 `--rover-host <IP>` 대신 `--usb --adb $adb`를 사용할 수 있습니다. 경로 주행에는 모드 1 설정, 경로 보정 파일 및 실시간 위치 입력이 필요합니다.

[Arduino UNO Q의 USB 포트 전달 안내](https://docs.arduino.cc/tutorials/uno-q/remote-access/)

### Wi-Fi 연결

1. UNO Q의 **Wi-Fi IPv4 주소**를 확인합니다. 보드 터미널에서 `ip -4 -brief addr`를 실행하고 Wi-Fi 인터페이스의 주소를 사용합니다. 예를 들어 `192.168.0.42/24`이면 뒤의 `/24`를 빼고 사용합니다.
2. 노트북에서 저장소 루트 PowerShell을 열고 아래 예시의 IP를 **UNO Q의 주소**로 바꿔 실행합니다. 기존 경로 주행 제어기와 같은 형식의 속도 명령 5개를 0.1초 간격으로 보내고 정지 명령을 보냅니다.

   ```powershell
   Set-Location -LiteralPath 'C:\Users\nam04\OneDrive\바탕 화면\ai-valet-parking-rover'
   .\.venv\Scripts\python.exe -m src.rover_control.probe --rover-host 192.168.0.42
   ```

3. App Lab의 **Python** 탭에서 `RX seq=2 drive v=2 w=0 MCU=2`부터 `RX seq=7 stop MCU=0`까지 확인합니다. 노트북에는 각 명령에 대응하는 `applied_seq`와 마지막 `통신 성공`이 표시됩니다. `motor_output_disabled`는 MCU까지 명령이 도착했고 모터 출력을 차단했다는 정상 응답입니다.
4. 연결이 안 되면 `Test-NetConnection 192.168.0.42 -Port 8765`를 실행합니다. `TcpTestSucceeded: False`이면 App Lab의 `LISTEN` 로그와 IP를 먼저 확인합니다. 둘 다 맞으면 네트워크에서 기기 간 연결을 허용하는지 확인합니다. TCP가 열리는데 probe가 실패하면 **Python** 탭의 `SESSION CLOSED`와 **App launch**를 확인합니다.

## 3. 배선 확인과 바퀴 시험

HYPER Q-ROVER 1차시 자료의 PDF 12~14쪽 배선입니다. 모터 전원을 끄고 실물 배선을 확인합니다.

| DRV8833 | 연결 |
| --- | --- |
| AIN1 / AIN2 | UNO Q D5 / D3 |
| BIN1 / BIN2 | UNO Q D6 / D9 |
| AOUT1 / AOUT2 | 왼쪽 TT 모터 |
| BOUT1 / BOUT2 | 오른쪽 TT 모터 |
| VM / GND | AA 4구 배터리의 + / − |
| GND | UNO Q GND와 공통 |

UNO Q는 USB-C 전원을 사용합니다. 스케치는 실습과 같은 느린 감쇄 PWM을 사용하고, 정지 명령에는 네 입력을 HIGH로 두어 제동합니다.

1. 로버를 받침대에 올려 **바퀴를 바닥에서 띄웁니다**. App Lab에서 앱을 정지하고 `#define OUTPUT_MODE 0`을 `#define OUTPUT_MODE 2`로 바꿉니다.
2. **Run**을 눌러 `LISTEN TCP 8765 mode=bench`를 확인한 뒤 모터 전원을 연결합니다. 노트북과 로버만 사용하는 시험용 네트워크에서 다음 명령을 실행합니다.

   ```powershell
   .\.venv\Scripts\python.exe -m src.rover_control.probe --rover-host 192.168.0.42 --bench-drive
   ```

3. 두 바퀴가 약 **0.2초** 전진 방향으로 돌고 제동하는지 확인합니다. MCU가 시간을 직접 제한하며, App을 한 번 실행할 때 한 번만 동작합니다. 재시험은 **Stop → Run** 후 실행합니다. 한쪽 방향이 반대면 모터 전원을 끄고 해당 `*_FORWARD_SIGN`을 바꿉니다.

## 4. 경로 주행 연결

`OUTPUT_MODE 1`은 센서를 연결한 뒤 사용하는 경로 주행 모드입니다. 1차시 자료에는 ToF 모델과 배선이 없으므로, 코드의 VL53L0X 지원은 장착 센서가 일치할 때 사용합니다. `TRACK_WIDTH_CM`에 좌우 바퀴 중심 간격을, `LEFT_PWM_AT_8_CM_S`와 `RIGHT_PWM_AT_8_CM_S`에 실측 PWM 보정값을 넣습니다. 이 모드에서는 센서가 유효한 거리를 읽어야 주행 명령을 받을 수 있습니다. 바닥 시험 전에는 바퀴를 띄운 상태에서 명령 끊김(300 ms), 장애물(초기 기준 120 mm), 센서 유실(측정 만료 150 ms) 시 제동을 확인합니다.

[Arduino Bridge API](https://docs.arduino.cc/software/app-lab/bridge/bridge-api) · [DRV8833 데이터시트](https://www.ti.com/lit/ds/symlink/drv8833.pdf) · [Adafruit VL53L0X 라이브러리](https://github.com/adafruit/Adafruit_VL53L0X)
