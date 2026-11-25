# Q-ROVER 엔코더 웹 주행 — 최종 App Lab 버전

2026-11-25 사용자가 실물 주행 정상 동작을 확인한 App Lab Export를 반영했습니다. 실행 순서는 저장소 [README의 최종 버전 실행 순서](../../README.md#최종-버전-실행-순서--app-lab핫스팟웹캠-1번)를 따릅니다. 실물 확인은 사용자 보고이며 아래 자동 시험과 구분합니다.

## 앱 파일과 설정

- `q_rover_encoder_receiver.zip`은 사용자 최종 Export를 반영한 앱입니다. 스케치의 공백만 있는 빈 줄 한 곳만 정리했고 제어 코드는 그대로입니다. 수정 전 Export는 Git 제외 `backups/`에 보관했습니다. App Lab에서 Import한 뒤 Run합니다.
- `q_rover_encoder_receiver/`의 `app.yaml`, `python/main.py`, `sketch/sketch.ino`, `sketch/sketch.yaml`은 ZIP의 내용과 바이트 단위로 같습니다. ZIP 내부 앱 폴더 이름은 `q-rover-encoder-receiver`입니다.
- 보드는 Arduino UNO Q이며 HC-020K 상승 에지 입력은 왼쪽 D2·오른쪽 D7, 모터는 왼쪽 D5/D3·오른쪽 D6/D9입니다.
- 최종 스케치는 왼쪽 극성 `1`, 오른쪽 `-1`, 전진·회전·미세 회전 PWM `150`, `KP=5.0`, 중복 에지 제외 간격 `2000us`를 사용합니다. 다른 하드웨어는 극성과 구동 출력을 다시 확인합니다.
- ToF는 최종 Export에서 꺼져 있습니다. 해당 앱은 ToF 장애물 자동 정지를 제공하지 않습니다.
- MCU는 `encoder_step`, `encoder_poll`, `encoder_stop`을 제공합니다. Python 콘솔의 `step / timed_step` 표시는 Python 수신기가 지원하는 메시지 목록이며, 최종 MCU 스케치에는 `timed_step`이 없습니다. 최종 앱과 주행할 때는 반드시 `--encoder-drive`를 사용합니다.

## 웹캠·핫스팟 실행

PC와 보드를 같은 핫스팟에 연결합니다. 첫 번째 PowerShell은 저장소 루트에서 웹캠 1번 서버를 실행합니다.

```powershell
.\.venv\Scripts\python.exe -B -X utf8 -m scripts.http_map_server --port 8001 --camera-index 1 --marker-layout assets/maps/parking_map_real_markers.json
```

App Lab에서 최종 ZIP을 Import하고 Run합니다. [웹](http://localhost:8001)에서 카메라 시작 → 현재 렌즈 위치·마커 높이 입력 → 빈 지도 생성·cm 보정 검증 → 주차칸 경계 저장 → 감시 모드 → 빈칸 선택 → 테스트 시작을 진행합니다. 다른 세션이나 사진의 보정은 재사용하지 않습니다.

두 번째 PowerShell에서 보드의 현재 핫스팟 IP를 직접 입력하고 사전 검사합니다. 윤거는 좌우 바퀴 중심 간 거리 15cm 기준입니다.

```powershell
$roverHost = Read-Host '보드의 핫스팟 IP 주소'
.\.venv\Scripts\python.exe -B -X utf8 .\apps\qrover_encoder\web_drive.py --rover-host "$roverHost" --track-width-cm 15 --encoder-drive
```

사전 검사는 모터에 연결하지 않습니다. 출발 관측·목적지 ID·중심 좌표·첫 펄스 명령을 확인한 뒤 같은 창에서 실행합니다.

```powershell
.\.venv\Scripts\python.exe -B -X utf8 .\apps\qrover_encoder\web_drive.py --rover-host "$roverHost" --track-width-cm 15 --encoder-drive --drive
```

웹 테스트 중단 또는 주행 터미널 Ctrl+C로 정지합니다. 처음부터 목표 반경 안이면 바퀴를 움직이지 않습니다. 새 테스트를 시작했거나 통신·MCU 오류로 종료됐다면 상태를 확인하고 명령을 다시 실행합니다. `--reference-cm X Y`는 관측 좌표와 실측값의 차이를 표시하며 보정값을 변경하지 않습니다.

USB는 같은 실행 명령의 `--rover-host "$roverHost"`를 `--usb`로 바꿉니다. ADB에 보드가 나타나야 하며 PATH에 없으면 `--adb <adb.exe 경로>`를 지정합니다. 핫스팟으로만 연결한 보드는 `--rover-host`를 사용합니다.

## 추종과 도착

실행마다 현재 웹 테스트의 `parking_space_id`를 읽습니다. 높이 보정된 새 ArUco 마커 중심 관측 한 장에서 출발하며, 실제 현재 위치 → 도로 합류 → 연결된 도로 그래프 → 선택한 주차칸 중심 순서로 경로를 만듭니다. 차체 외곽 검사는 웹 주행에 적용하지 않습니다.

현재 구간과 앞 8cm 범위에서 진행점을 유지하고 6.5cm 앞을 추종합니다. 45도 이상 코너는 1.5cm 이내 접근까지 목표점을 코너로 제한하고, 통과한 코너를 기록합니다. 전진은 1~5펄스, 회전은 1~2펄스이며 목표 부근에서는 전진을 줄입니다. 현재·앞 구간에서 5cm 이상 이탈하면 관측 위치에서 같은 목적지로 다시 계획합니다.

매 동작 뒤 새 관측 한 장으로 다음 명령을 결정합니다. 같은 프레임을 재사용하지 않습니다. 관측 유효 시간은 촬영 후 200ms이며 마커가 사라지거나 오래되면 제동하고 같은 테스트에서 재관측을 기다립니다. 새 위치가 돌아오면 자동 재개합니다. 웹 Stop·새 테스트·목적지 점유/만료·MCU 고장·통신 오류는 종료 사유입니다.

마커 중심과 목적지 중심의 거리가 기본 2cm 이내인 새 관측 한 장이면 즉시 제동합니다. 최종 방향·차체 겹침·연속 관측 횟수는 요구하지 않습니다. 반경은 사전 검사와 실주행 양쪽에 `--arrival-radius-cm`으로 지정할 수 있습니다. 웹 화면의 성공 표시와 PC 주행기의 cm 도착 판정은 별개이며 터미널의 모터 주행 완료 메시지를 확인합니다.

## 통신과 오류

최종 Python 수신기의 클라이언트 대기는 `TIMEOUT=5.0`, Bridge RPC 대기는 `RPC_TIMEOUT=2.0`입니다. PC의 TCP 연결 대기는 2초, 응답 대기는 1.5초이며 최종 Export를 반영하면서 주행 제어 값은 변경하지 않았습니다. 따라서 PC가 Bridge의 2초 오류 응답보다 먼저 종료될 수 있으며 같은 시각 App Lab Python 콘솔도 확인합니다.

MCU는 2초 heartbeat, 1.5초 엔코더 정체, 6초 작업 제한과 200ms 제동 후 재시작 대기를 사용합니다. 음수 고장은 유지되며 PC에서 무시하거나 자동 재구동하지 않습니다. Bridge UTF-8 오류와 시간 초과의 과거 발생 원인은 확정하지 않았습니다. 사용자 최종 실물 정상 동작 확인과 별도로 이 이력은 남깁니다.

## 소스와 Import ZIP 갱신

소스를 수정했다면 저장소 루트에서 실행합니다. 기존 ZIP은 Git에서 제외한 `backups/`에 보관하고 새 ZIP을 만듭니다.

```powershell
.\.venv\Scripts\python.exe -B -X utf8 .\apps\qrover_encoder\package_receiver.py
```

새 ZIP은 앱 폴더 이름에 밑줄을 사용하지만 네 소스의 내용은 그대로입니다. App Lab에 이미 Import된 앱에는 자동 반영되지 않으므로 새 ZIP을 Import합니다.

## 자동 검증과 로컬 산출물

```powershell
.\.venv\Scripts\python.exe -B -X utf8 -m unittest discover -s apps/qrover_encoder/simulation -p 'test_*.py' -q
.\.venv\Scripts\python.exe -B -X utf8 .\apps\qrover_encoder\simulation\check_interfaces.py
.\.venv\Scripts\python.exe -B -X utf8 .\apps\qrover_encoder\simulation\check_firmware.py
```

통신 시험은 localhost의 모의 Bridge를 사용합니다. 펌웨어 시험은 C++ 컴파일러로 실제 최종 스케치를 호스트에서 실행하며 Arduino/Bridge/ToF API를 모의 구현합니다. 실제 보드 업로드나 모터 구동은 하지 않습니다. 저장 캡처를 이용하는 일부 주행 시험은 `data/http_map_captures/latest`와 대응 도착 기록이 있는 PC에서 실행하고, 새 clone에는 없으면 skip합니다.

사진·보정 JSON·진단 로그는 `.gitignore`로 제외하고 로컬에 보관합니다. 기본 지도 graph/semantic과 입력 이미지·라벨·기준점·모델은 유지합니다. 지도 경로 PNG/JSON과 미리보기는 필요할 때 기존 지도 생성·시각화 명령으로 재생성합니다.
