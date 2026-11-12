# Q-ROVER Receiver

노트북의 경로 추종 제어기가 보낸 TCP 속도 명령을 UNO Q의 Linux에서 받고, Bridge로 STM32 MCU에 전달합니다.

## App Lab 설정

1. UNO Q를 노트북과 USB-C 데이터 케이블로 연결하거나 같은 Wi-Fi에 연결합니다.
2. 저장소 루트의 PowerShell에서 앱 폴더를 ZIP으로 묶습니다.

   ```powershell
   Compress-Archive -Path .\apps\q_rover_receiver -DestinationPath .\q_rover_receiver.zip
   ```

3. App Lab의 **Apps → Create new App + → Import or install App**에서 ZIP을 선택합니다. 기존 앱이 실행 중이면 정지합니다.
4. 실제 장착한 센서가 VL53L0X인지 확인하고 Arduino `Wire` 버스에 연결합니다. 스케치의 `TRACK_WIDTH_CM`은 실측 12 cm를 반영합니다. `LEFT_PWM_AT_8_CM_S`와 `RIGHT_PWM_AT_8_CM_S`에는 각 바퀴의 실측 PWM을 입력합니다. 보정값을 비워 두면 스케치 빌드에서 오류를 표시합니다.
5. **Run**을 누르고 **Console → Python**의 `LISTEN TCP 8765 | MCU stopped`를 확인합니다. 주행 명령에 앞서 MCU에 정지를 요청하고 응답을 받은 뒤 포트를 엽니다.

[Arduino App Lab 가져오기](https://docs.arduino.cc/software/app-lab/apps/manage-apps) · [실행과 콘솔](https://docs.arduino.cc/software/app-lab/apps/run)

## 배선

| DRV8833 | 연결 |
| --- | --- |
| AIN1 / AIN2 | UNO Q D5 / D3 |
| BIN1 / BIN2 | UNO Q D6 / D9 |
| AOUT1 / AOUT2 | 왼쪽 TT 모터 |
| BOUT1 / BOUT2 | 오른쪽 TT 모터 |
| VM / GND | AA 4구 배터리의 + / − |
| GND | UNO Q GND와 공통 |

UNO Q는 USB-C 전원을 사용합니다. 스케치는 느린 감쇄 PWM을 사용하며 정지 시 네 입력을 HIGH로 두어 제동합니다. 좌우 바퀴 방향은 `*_FORWARD_SIGN`으로 설정합니다.

## 경로 주행

저장소 루트에서 `python -m src.rover_control`로 지도와 경로를 생성합니다. 보정을 통과한 cm 경로와 같은 좌표계의 실시간 위치를 준비한 뒤 `drive`를 실행합니다. 아래 명령의 파일 경로는 생성된 주행용 JSON 경로로 바꿉니다.

```powershell
.\.venv\Scripts\python.exe -m src.rover_control.drive --usb --route '<검증된 cm 경로 JSON>'
```

- USB 연결에서 ADB가 PATH에 없으면 `--adb <adb.exe 경로>`를 추가합니다. `adb -d get-state`가 `device`를 출력하는지 확인합니다. Wi-Fi를 쓸 때는 `--usb` 자리에 `--rover-host <UNO Q IP>`를 넣습니다.
- 위치 입력은 노트북의 UDP 8766 포트로 받습니다. 같은 지도 ID의 로버 중심과 방향이 0.20초 안에 갱신돼야 합니다. [위치 및 명령 형식](../../docs/TEAM_GUIDE.md#경로와-로버-위치)
- MCU는 유효한 명령이 300 ms 동안 없거나 ToF가 장애물을 감지하거나 측정이 만료되면 제동합니다. 초기 장애물 기준은 120 mm, ToF 측정 만료는 150 ms입니다. 주행 중 정지한 뒤 다시 출발하려면 App Lab에서 **Stop → Run**을 실행합니다.
- App Lab에 `tof_unavailable`이 보고되면 센서 배선과 유효 거리 측정을 확인합니다. `tof_obstacle`이면 정지 기준 이내의 물체를 확인합니다. 바닥 주행에 앞서 바퀴를 띄운 상태에서 ToF, 통신 끊김, 위치 유실 시 정지 동작을 확인합니다.

[Arduino Bridge API](https://docs.arduino.cc/software/app-lab/bridge/bridge-api) · [DRV8833 데이터시트](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
