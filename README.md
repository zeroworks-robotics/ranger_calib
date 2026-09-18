# ranger_calib

Ranger 로봇의 RGBD 카메라 외부 파라미터(extrinsic)를 브라우저에서 눈으로 맞추고, 그 결과를 URDF `<origin>` 으로 저장하는 도구입니다.

라이다(`rslidar`) 포인트클라우드를 고정 기준으로 두고, 5개 RGBD 카메라의 포인트클라우드를 마우스로 끌어 정렬합니다. 정렬이 끝나면 로봇이 실제로 쓰는 URDF 위에 조정값을 반영한 **새 URDF 파일**을 만들어 줍니다. 원본 URDF는 이 도구가 절대 수정하지 않습니다.

- 기준 좌표계: `base_link` (Z-up, ROS 규약)
- 편집 대상: `base_link` → `camera_*RGBD_link` 조인트의 `<origin>`
- 고정 참조: `rslidar` (회색 점, 드래그 불가)

![웹 UI 화면](docs/ui-overview.png)

위 화면은 `Rear` 카메라를 선택해 이동 기즈모로 조정 중인 상태입니다. 왼쪽은 3D 뷰(센서별 색상 범례 포함), 오른쪽은 작업 패널입니다. 하단의 갱신 상태 줄에 `갱신 실패 — front, right, left 응답 없음 (3/6)` 처럼 캡처 결과가 그대로 표시됩니다.

## 구성

| 파일 | 역할 |
| --- | --- |
| `calib_server.py` | 웹 UI 서버. 정적 파일 제공 + URDF 읽기/저장 + 스냅샷 갱신·자동 정렬 트리거 |
| `auto_calib.py` | 자동 정렬 계산기 (point-to-plane ICP). 결과만 돌려주고 저장은 안 함 |
| `capture_once.py` | 6개 센서 토픽을 한 번씩 캡처해 `data/*.json` 으로 저장 |
| `capture_loop.py` | 같은 캡처를 4초 주기로 반복 (서버 없이 계속 갱신할 때 사용) |
| `index.html` | 3D 정렬 웹 UI (three.js) |
| `urdf/` | 기준 URDF 및 저장 결과 URDF |
| `data/` | 캡처된 포인트클라우드 스냅샷 (JSON) |
| `tests/` | 자동 정렬 검증 스크립트 (로봇 없이 실행) |

## 요구 사항

실행은 **로봇 본체(ROS 2 노드가 도는 장비)** 에서 합니다. 캡처 스크립트가 ROS 토픽을 직접 구독하기 때문입니다.

- ROS 2 Humble (`/opt/ros/humble/setup.bash`)
- `rosbridge_server` 패키지 (별도 터미널에서 먼저 띄웁니다 — 아래 [실행](#실행) 참고)
- Python 3, `rclpy`, `numpy`, `sensor_msgs_py`
- `scipy` (자동 정렬에만 필요. 없으면 `자동 정렬` 버튼만 동작하지 않고 나머지는 그대로 작동)
- 브라우저 (three.js를 CDN에서 받으므로 **UI를 여는 쪽은 인터넷 연결 필요**)

캡처 대상 토픽 6개:

```
/rslidar_points
/camera_frontRGBD/depth/points
/camera_fdownRGBD/depth/points
/camera_rightRGBD/depth/points
/camera_rearRGBD/depth/points
/camera_leftRGBD/depth/points
```

## 실행

터미널 2개를 씁니다. 1번은 rosbridge, 2번은 캘리브레이션 서버입니다. 둘 다 로봇에서 실행합니다.

### 터미널 1 — rosbridge

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

작업이 끝날 때까지 이 터미널은 켜 둡니다.

### 터미널 2 — 캘리브레이션 서버

```bash
git clone git@github.com:zeroworks-robotics/ranger_calib.git
cd ranger_calib
python3 calib_server.py
```

이 터미널은 ROS 환경이 필요 없습니다. `calib_server.py` 는 `rclpy` 를 쓰지 않는 순수 HTTP 서버이고, 캡처가 필요할 때마다 하위 셸을 띄워 그 안에서 `source` 와 `export` 를 직접 수행합니다.

기동 시 콘솔에 읽어온 실사용 URDF 경로가 찍힙니다. 여기가 `MISSING` 이면 `CONA_URDF_PATH` 설정이 잘못된 것이므로 UI를 열기 전에 먼저 고쳐야 합니다.

```
serving /home/cona/ranger_calib on :8080, POST /save_urdf writes into /home/cona/ranger_calib/urdf
live urdf: ../data/CoNA/urdf/ranger_new_a01.urdf  (src=ROBOT_SETUP) [ok]
```

브라우저에서 `http://<로봇 IP>:8080` 을 엽니다 (예: `http://192.168.10.106:8080`).

## 캘리브레이션 절차

웹 UI 오른쪽 패널의 ①②③ 순서를 그대로 따릅니다.

### ① 기준 원본 — 최신 URDF 불러오기

`최신 URDF 불러오기` 를 누릅니다. 로봇의 실사용 URDF를 그 자리에서 읽어와 화면과 기준값을 갱신합니다.

- **이 단계를 건너뛰면 ③의 저장이 잠긴 채로 풀리지 않습니다.** 페이지에 내장된 예비 URDF를 기준으로 저장하면 실파일에만 있는 링크(초음파 등)가 사라지기 때문에 의도적으로 막아 둔 것입니다.
- 작업 시작 전에만 누릅니다. 이미 조정한 값은 전부 버려지고 되돌릴 수 없습니다. 조정값이 있으면 확인을 위해 두 번 눌러야 실행됩니다.

### ② 카메라 선택

탭에서 조정할 카메라를 고릅니다: Front / Front-down / Right / Rear / Left. 라이다는 선택 대상이 아닙니다.

### 정렬

3D 뷰에서 선택한 카메라의 포인트클라우드를 라이다 점군에 겹치도록 맞춥니다.

| 조작 | 동작 |
| --- | --- |
| 좌 드래그 | 시점 회전 |
| 우 드래그 | 시점 이동 |
| 휠 | 줌 |
| `G` / `이동` 버튼 | 기즈모를 이동 모드로 |
| `R` / `회전` 버튼 | 기즈모를 회전 모드로 |
| `Iso` `Top` `Front` `Side` | 고정 시점 전환 |

패널의 `위치 (m)` / `회전 R/P/Y (deg)` 입력란에 숫자를 직접 넣어 미세 조정할 수도 있습니다. 되돌릴 때는 `이 카메라 초기화` 또는 `전체 초기화` 를 씁니다. `포인트 크기` 슬라이더로 점 크기를 바꿉니다.

점군이 오래되어 보이면 `스냅샷` 항목의 `데이터 갱신` 을 누릅니다. 로봇에서 6개 센서 프레임을 새로 한 번 캡처하며, 조정 중인 위치/회전 값은 유지됩니다. 6개 전부 성공해야 성공으로 표시되고, 일부만 들어오면 실패로 보고합니다 — 오래된 점군에 맞춰 보정하는 사고를 막기 위한 동작입니다.

### 자동 정렬 (ICP) — 손으로 맞추는 대신

`자동 정렬 — 5개 카메라 계산` 을 누르면 서버가 `auto_calib.py` 를 돌려 카메라별 이동·회전 보정량을 계산하고, 그 값을 **화면에만** 적용합니다. 손으로 기즈모를 끌어 만든 상태와 같은 상태가 되며, 저장은 ③에서 직접 눌러야 합니다.

계산 방식:

- 카메라 점군에서 평면 법선을 뽑아 타깃으로 쓰고, 라이다 점군을 소스로 맞춥니다. 라이다는 링 스캔이라 점이 희박해 법선 추정이 나쁘므로 방향을 이렇게 잡습니다. 카메라에 적용하는 보정은 그 역변환입니다.
- 초기값은 화면의 현재 origin(손으로 조정한 값 포함)입니다. 이미 정답 근처이므로 대응점 탐색 범위를 좁게(최대 40~100 mm) 잡습니다. 넓히면 벽 위의 다른 지점에 붙어 엉뚱한 해로 걸어갑니다.
- 반복 1회당 이동 20 mm, 회전 1° 로 걸음을 제한하고, 누적 보정이 100 mm를 넘으면 발산으로 보고 중단합니다.

카메라별로 아래 조건을 모두 통과해야 `적용` 되고, 하나라도 걸리면 그 카메라는 **원래값을 유지**한 채 이유를 표시합니다.

| 조건 | 기준 | 뜻 |
| --- | --- | --- |
| `fitness` | ≥ 0.30 | 시야 안 라이다 점 중 정합에 쓰인 비율. 낮으면 겹침 부족 |
| `inlier_rmse` | ≤ 20 mm | 정합 후 점-평면 잔차 |
| 평행이동 보정 | ≤ 50 mm | 초기값이 실측이라 이보다 크면 오수렴 |
| 회전 보정 | ≤ 5° | 같은 이유 |
| 대응점 수 | ≥ 80 | 이보다 적으면 판정 불가 |

축 단위로도 걸러냅니다. 정보행렬에서 DOF별 1-sigma 불확실도를 계산해, 절대 기준(이동 10 mm, 회전 0.5°)을 넘거나 같은 그룹에서 가장 잘 구속된 축보다 15배 이상 나쁜 축은 보정을 버리고 원래값을 유지합니다. 그런 축은 보고에 `관측 불가로 제외: yaw, x, y` 형태로 표시됩니다.

이 축 단위 판정이 자동화의 핵심 안전장치입니다. 바닥 평면 하나만 겹치는 장면에서는 z·roll·pitch만 구속되고 x·y·yaw는 평면 위를 자유롭게 미끄러집니다. ICP는 그래도 수렴하고 `fitness`·`rmse`까지 좋게 나오므로, 이 판정 없이는 엉뚱한 값을 그대로 저장하게 됩니다.

따라서 **장면을 제대로 만드는 것이 알고리즘보다 중요합니다.** 서로 직교하는 면이 3개 보이게 세팅하십시오 — 방 구석에 로봇을 주차하거나, 각 카메라 시야에 박스를 2~3개 놓는 방식입니다. 바닥만 보이는 상태로 돌리면 대부분의 축이 제외된 채 거의 아무것도 보정되지 않습니다.

터미널에서 직접 돌릴 수도 있습니다. 요청 JSON 형식은 `auto_calib.py` 의 docstring에 있습니다.

```bash
python3 auto_calib.py request.json
```

### ③ URDF 저장

`현재 origin 복사` 로 `<origin>` 텍스트만 가져갈 수 있고, `URDF 만들기 — 새 파일로 저장` 을 누르면 서버가 `urdf/` 밑에 두 개를 씁니다.

```
urdf/<원본이름>_calib_<YYYYMMDD_HHMMSS>.urdf   # 이력용 타임스탬프 파일
urdf/<원본이름>_latest.urdf                     # 항상 최신 결과
```

원본 URDF는 건드리지 않으므로 여러 번 눌러도 안전합니다. 로봇에 실제로 반영하려면 결과 파일을 `CONA_URDF_PATH` 가 가리키는 위치로 직접 복사하고 관련 노드를 재시작하십시오.

## 서버 없이 캡처만 하기

점군 스냅샷만 갱신하려면 캡처 스크립트를 직접 돌립니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 capture_once.py    # 1회 캡처 (최대 8초 대기)
python3 capture_loop.py    # 4초 주기로 계속 갱신 (Ctrl+C 로 종료)
```

`capture_once.py` 는 토픽별 결과와 합계를 출력합니다. 서버는 이 출력의 `DONE` 줄을 보고 성공 여부를 판정합니다.

```
RESULT rslidar_points: OK
...
DONE 6/6
```

캡처 동작 요약:

- 센서당 최대 20,000점까지 무작위 추출 (`MAX_POINTS`)
- 원점에서 0.05 m 이내의 점은 무효 깊이 픽셀로 보고 제거 (`MIN_RANGE`). RGBD 드라이버가 무효 픽셀을 NaN이 아니라 `(0,0,0)` 으로 내보내므로 `skip_nans` 만으로는 걸러지지 않습니다.
- BEST_EFFORT / RELIABLE QoS 양쪽을 동시에 구독해 드라이버 설정 차이를 흡수합니다.
- 좌표값은 소수 4자리로 반올림한 평탄 배열(`[x,y,z,x,y,z,...]`)로 저장합니다.
- `.tmp` 에 쓰고 `os.replace` 로 교체하므로, 읽는 쪽이 반쯤 쓰인 파일을 보는 일은 없습니다.

## 개발 PC에서 확인하기

로봇 없이 개발 PC(Windows/Linux)에서도 대부분 확인됩니다. ROS 토픽을 구독하는 캡처만 불가능합니다.

| 기능 | 개발 PC | 비고 |
| --- | --- | --- |
| 자동 정렬 (ICP) | 동작 | `numpy`, `scipy` 만 필요 |
| 3D 뷰·기즈모·수동 조정 | 동작 | 브라우저만 필요 |
| `최신 URDF 불러오기` / `URDF 만들기` | 동작 | `CONA_URDF_PATH` 를 로컬 URDF 로 지정 |
| `데이터 갱신` | 불가 | `capture_once.py` 가 `rclpy` 로 토픽을 직접 구독 |

### 1. 파이썬 환경

전역 환경을 건드리지 않으려면 가상환경을 씁니다.

```powershell
# Windows PowerShell
py -m venv $HOME\venvs\ranger_calib
& "$HOME\venvs\ranger_calib\Scripts\python.exe" -m pip install numpy scipy
$py = "$HOME\venvs\ranger_calib\Scripts\python.exe"
```

```bash
# Linux
python3 -m venv ~/venvs/ranger_calib
~/venvs/ranger_calib/bin/pip install numpy scipy
py=~/venvs/ranger_calib/bin/python
```

### 2. 알고리즘 검증 (브라우저 없이)

`tests/` 의 두 스크립트가 자동 정렬의 정확도와 안전장치를 검증합니다. 정상 종료 코드는 0 입니다.

```powershell
& $py tests\test_synth.py    # 알려진 양을 틀어 넣고 되찾는지
& $py tests\test_slip.py     # 미끄러짐 검출기가 작동하는지
```

`test_synth.py` 는 카메라 점군을 라이다 자리에 그대로 놓아 정답을 만들고, 카메라 pose 만 10.8 mm / 0.54° 틀어 넣습니다. 기대 출력:

```
front  PASS    | 남은 오차 0.00mm 0.000deg ... 제외축 -
fdown  PARTIAL | 남은 오차 4.00mm 0.400deg ... 제외축 rot_z,y
```

`PARTIAL` 은 실패가 아닙니다. `fdown`·`rear` 는 시야가 바닥에 치우쳐 일부 축이 구속되지 않으므로, 그 축은 보정을 버리고 원래값을 유지합니다. 그만큼 오차가 남는 것이 설계된 동작입니다.

`test_slip.py` 는 바닥 평면만 있는 장면과 직교하는 벽 2개를 더한 장면을 합성해 비교합니다. 기대 출력:

```
바닥만            ok=True  제외축=rot_z,x,y        적용된 이동(mm)=[0.00, 0.00, -0.01]
바닥+직교벽2        ok=True  제외축=-                적용된 이동(mm)=[-29.99, -0.03, 0.02]
```

30 mm 를 틀어 넣었을 때, 바닥만 있는 장면은 x 를 구속하지 못하므로 보정을 버리고(0.00 mm), 직교 벽이 있는 장면은 전부 되찾습니다(-29.99 mm).

### 3. 눈으로 확인 — 틀어 놓고 되찾게 하기

정답이 있는 데모 데이터를 만들어, 화면에서 어긋난 상태가 자동 정렬로 맞춰지는 것을 직접 봅니다.

```powershell
& $py tests\make_demo_data.py
```

이 스크립트는 카메라 5대 점군을 실측 origin으로 펼쳐 합쳐 라이다 점군으로 되돌려 저장하고(정답 origin에서 완벽히 겹치는 상태), 거기서 카메라별로 25~32 mm·1.0~1.2° 틀어 놓은 `urdf/demo_misaligned.urdf` 를 만듭니다.

```powershell
$env:CONA_URDF_PATH = "urdf\demo_misaligned.urdf"
$env:RANGER_CALIB_PORT = "8099"
& $py calib_server.py
```

`http://127.0.0.1:8099` 에서 `최신 URDF 불러오기` → 점군이 회색 라이다에서 어긋남 → `자동 정렬` → 어긋남이 사라집니다. 실제 실행 결과:

| 카메라 | 틀어 놓음 | 적용된 보정 | 판정 |
| --- | --- | --- | --- |
| Front | 30.0 mm / 1.20° | 30.0 mm / 1.20° | 적용 |
| Right | 28.0 mm / 1.20° | 28.0 mm / 1.20° | 적용 |
| Rear | 29.2 mm / 1.00° | 28.8 mm / 1.01° | 적용 (y축 제외) |
| Left | 31.6 mm / 1.13° | 31.5 mm / 1.14° | 적용 |
| Front-down | 25.0 mm / 1.00° | 없음 | 유지 — 정합 발산 |

`Front-down` 은 시야가 바닥에 치우쳐 평면 하나만 겹치므로 미끄러집니다. 누적 보정이 100 mm를 넘어 발산으로 판정하고 원래값을 유지합니다 — 엉뚱한 값을 적용하는 것보다 안전한 쪽입니다.

확인이 끝나면 원상복구:

```powershell
git checkout -- data/rslidar_points.json
Remove-Item data\rslidar_points.json.orig, urdf\demo_misaligned.urdf
```

### 4. 실데이터로 웹 UI 확인

`CONA_URDF_PATH` 를 저장소 안의 URDF 로 지정하면 불러오기와 저장까지 동작합니다.

```powershell
$env:CONA_URDF_PATH = "urdf\ranger_new.urdf"
$env:RANGER_CALIB_PORT = "8099"
& $py calib_server.py
```

브라우저에서 `http://127.0.0.1:8099` 를 열고 `자동 정렬` 을 누릅니다. 계산은 카메라 5대에 수십 초 걸립니다.

저장을 눌러 생긴 `urdf/ranger_new_calib_*.urdf` 와 `urdf/ranger_new_latest.urdf` 는 확인 후 지우십시오 — `.gitignore` 가 타임스탬프 파일만 제외하므로 `_latest` 는 커밋 대상에 남습니다.

주의: 이 상태로 자동 정렬을 돌리면 **5대 전부 `유지` 로 나오는 것이 정상입니다.** 커밋된 `data/` 가 일부 센서만 갱신된 스냅샷(`3/6` 실패)이라 보정 전 잔차가 이미 38~42 mm 입니다. 정확도 판정은 위 `tests/` 로 하고, 실데이터 정합은 로봇에서 `6/6` 을 확인한 뒤 하십시오.

## 설정

| 환경변수 | 기본값 | 용도 |
| --- | --- | --- |
| `RANGER_CALIB_PORT` | `8080` | 웹 서버 포트 |
| `CONA_URDF_PATH` | (기체 설정파일에서 읽음) | 실사용 URDF 경로. 상대·절대 경로 모두 허용하며 디렉터리를 줘도 됩니다 |
| `ROBOT_SETUP` | `/etc/coga-robotics/cona/setup.bash` | `CONA_URDF_PATH` 를 찾을 기체 설정파일 |

URDF 경로 결정 순서:

1. 환경변수 `CONA_URDF_PATH`
2. `ROBOT_SETUP` 파일을 `bash` 로 source 해서 얻은 `CONA_URDF_PATH`
3. 폴백 `../data/CoNA/urdf/ranger_new_a01.urdf`

2번 경로가 필요한 이유: `.bashrc` 는 비대화형 셸에서 조기 return 하므로 systemd나 ssh로 서버를 띄우면 `CONA_URDF_PATH` 가 프로세스 환경에 실려 오지 않습니다.

저장 파일명은 실사용 URDF 이름을 그대로 따라갑니다. 기체가 `a01` → `a02` 로 바뀌면 코드를 고치지 않아도 결과 파일명이 같이 따라옵니다.

## HTTP API

| 메서드 | 경로 | 동작 |
| --- | --- | --- |
| `GET` | `/latest_urdf` | 실사용 URDF 원문 반환. 응답 헤더 `X-Live-Urdf` 에 실제 파일명 |
| `POST` | `/refresh_snapshot` | `capture_once.py` 실행. 6개 전부 성공 시 `200`, 부분 성공/실패 `500`, 15초 초과 `504` |
| `POST` | `/auto_align` | 본문(화면 현재 상태 JSON)으로 `auto_calib.py` 실행. 카메라별 제안 origin과 판정 근거를 JSON 으로 반환. 저장은 하지 않음 |
| `POST` | `/save_urdf` | 본문(URDF 전문)을 `urdf/` 밑에 타임스탬프 파일 + `_latest` 로 저장. 응답 본문은 저장 경로 |
| `GET` | 그 외 | 저장소 디렉터리의 정적 파일 (`index.html`, `data/*.json`) |

## 문제 해결

**서버 로그에 `live urdf: ... [MISSING - CONA_URDF_PATH 를 확인하세요]`**
`CONA_URDF_PATH` 가 없는 파일을 가리킵니다. UI의 `최신 URDF 불러오기` 가 실패하고 저장도 잠긴 상태로 남습니다. 경로를 고친 뒤 서버를 재시작하십시오.

**`URDF 만들기` 버튼이 계속 비활성**
①의 `최신 URDF 불러오기` 를 누르지 않았습니다. 의도된 잠금입니다.

**`데이터 갱신` 이 실패 / `DONE 4/6`**
해당 센서 토픽이 안 나오고 있습니다. `ros2 topic hz /camera_rearRGBD/depth/points` 등으로 퍼블리시 여부와 `ROS_DOMAIN_ID`(18), `RMW_IMPLEMENTATION`(`rmw_cyclonedds_cpp`) 설정을 확인하십시오.

**3D 뷰가 빈 화면**
three.js를 CDN(`cdnjs.cloudflare.com`, `cdn.jsdelivr.net`)에서 받습니다. 페이지를 여는 쪽 네트워크가 외부로 못 나가면 렌더링되지 않습니다.

**자동 정렬이 5대 모두 `유지` 로 나옴**
보고의 `initial_rmse`(보정 전 잔차)를 먼저 보십시오. 이 값이 이미 30 mm 이상이면 정합 문제가 아니라 데이터 문제입니다 — 카메라와 라이다 스냅샷의 촬영 시각이 다르거나(일부 센서만 갱신 성공), 그 사이 로봇이 움직인 경우입니다. `데이터 갱신` 을 눌러 `6/6` 을 확인한 뒤 다시 돌리십시오. 이 저장소에 커밋된 `data/` 가 바로 그런 상태(`3/6` 실패 스냅샷)라서, 받은 그대로 돌리면 전부 거부됩니다.

**자동 정렬에서 `ModuleNotFoundError: No module named 'scipy'`**
`scipy` 를 설치하십시오 (`pip3 install scipy` 또는 `sudo apt install python3-scipy`). 다른 기능은 영향 없습니다.

**포인트클라우드는 보이는데 갱신이 안 됨**
로봇이 아닌 곳에서 열린 페이지(예: 파일로 복사한 `index.html`)는 로봇 네트워크에 닿지 못해 `데이터 갱신` 이 동작하지 않습니다. 반드시 로봇에서 서비스하는 주소로 접속하십시오.
