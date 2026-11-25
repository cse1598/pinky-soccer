# pinky_soccer_calibration

노트북 웹캠만으로 리허설하려면 **실행 옵션** `-webcam`을 사용합니다(UI에서 소스를 전환하지 않습니다).

```bash
source ~/pinky-soccer/install/setup.bash
ros2 run pinky_soccer_calibration calibration_ui -webcam
# 또는 launch로 실행
ros2 launch pinky_soccer_calibration calibration_ui.launch.py webcam:=true
```

내부 보정/독립 검증 → 바닥 높이·각도 추정 → 공 인식 검토 순으로 진행합니다.
웹캠 전용 rehearsal 프로젝트에 저장하며 핑키 프로젝트와 혼용하지 않습니다.
장치 번호·해상도 옵션과 측정 기준은 [ROS_UI.md](ROS_UI.md)의 웹캠 절을 참고하세요.

현재 ROS 보정 UI는 **JPEG 실시간 미리보기/체커 표시 + 촬영 버튼의 raw 한 프레임 서비스**를
사용합니다. 상시 원본 구독 없이 보정용 원본 PNG·CameraInfo·Bag·CSV를 보존합니다.
실행·서버 갱신과 저장 구조는 [ROS_UI.md](ROS_UI.md)를 참고하세요.

현재 ROS 기반 올인원 UI는 **[ROS 캘리브레이션 스튜디오](ROS_UI.md)**를 사용하세요.
**공 인식 검토** 탭은 경기용 HSV 검출·공통 거리 투영·BallFilter를 직접 사용해 공 표시와
현재/추적 좌표·거리를 보여 줍니다. 범퍼 기준 실측 X/Y·검토용 오프셋을 입력해 오차와 적용 후 값을 비교하고 기록할 수 있습니다([입력 기준](ROS_UI.md#범퍼-기준-공-위치-실측과-오프셋)). 보정이 없으면 명시한 HFOV 가정으로 추정하며 출처를 구분합니다.
프로젝트를 새로 만들거나 기존 폴더를 열어 계속 기록할 수 있습니다. **카메라 보정** 한 탭에서 보드 설정·원본 촬영·파일 추가·사진 분류/삭제·미리보기·계산을 진행합니다.
**측정 기록** 탭에서는 바퀴·센서의 측정 횟수/실측값 확인과 삭제·재계산을 진행합니다.
현재 기록으로 다시 계산한 뒤 최종 `profile.yaml`을 내보내며 미완료 자료도 저장합니다.
첫 **센서 모니터** 탭에서 배터리·IMU 자세·LiDAR 점군·초음파·IR ADC·odom·관절·카메라 상태와 원본 메시지를 기록 시작 없이 확인할 수 있습니다.
LiDAR·초음파 거리는 범퍼 전면 바깥쪽 중앙을 기준으로 입력하여 센서 좌표로 자동 변환합니다.
장착 보정은 바닥 보드로 높이·기울기만 추정합니다. 아래는 초기 프로토타입·CLI 안내입니다.

## 현재 상태 (2026-11-20)

이 패키지는 지금까지 구현한 보정 도구를 보존한 작업 결과입니다. Jupyter 방식은 초기
프로토타입이며 추가 개발을 중지했습니다. 자체 `pinky_soccer_bringup`에 실물 카메라의
ROS 원본·압축·CameraInfo 발행 노드를 추가했습니다([실행 안내](../pinky_soccer_bringup/README.md)).
아래 ROS 기록·검증 CLI는 해당 토픽을 사용할 수 있습니다. 프로토타입 GUI의 입력 교체와
YOLO·판단 연결은 후속 작업이며, 신규 카메라 ROS 노드의 실물 검증도 필요합니다.
실제 체커보드 촬영과 실물 보정 정확도 검증도 아직 진행하지 않았습니다.

## 초기 프로토타입: 로봇 Jupyter 카메라 보정 화면

실물은 OV5647 + `pinkylib.Camera`이고, `http://192.168.4.1:8888`에서 접근을 확인했습니다.
**[Jupyter 보정 프로그램 실행 안내](WORKBENCH.md)**를 따라 PC에서 아래 명령으로 시작합니다.
미리보기·촬영·내장 보정·독립 검증·결과 저장을 한 화면에서 진행합니다.

```bash
./src/pinky_soccer_calibration/start_workbench.sh
```

아래 ROS 토픽 기반 도구도 별도로 유지합니다. `pinkylib` Jupyter 경로에는 `usb_cam`이 필요하지 않습니다.

Pinky Pro 실물 보정 실험 도구입니다. Ubuntu 24.04 / ROS 2 Jazzy에서 사용합니다.
현재 구현 범위는 **1단계 카메라 내부 보정**입니다. 추정은 기존 ROS
`camera_calibration` GUI를 사용하고, 이 패키지는 Bag·CSV 기록, 결과 보존,
독립 검증, 반복 통계와 검증된 YAML 내보내기를 담당합니다.
`src/pinky_pro` 및 기존 로봇 설정은 변경하지 않습니다. 모터 명령도 발행하지 않습니다.

## 1. 설치 및 카메라 최초 확인

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/pinky_soccer_calibration --ignore-src -r -y
colcon build --symlink-install --packages-select pinky_soccer_calibration
source install/setup.bash
ros2 run pinky_soccer_calibration camera_intrinsics --help
```

모든 터미널에서 위 두 `setup.bash`를 source합니다. 시스템 Python(`/usr/bin/python3`)을
사용합니다. 카메라가 연결된 로봇에서 드라이버를 실행하고, GUI는 화면이 있는 PC에서
동일한 `ROS_DOMAIN_ID`와 통신 가능한 DDS 환경으로 실행할 수 있습니다.
기존 `pinky_soccer_bringup robot.launch.py`는 카메라 드라이버를 실행하지 않습니다.
실물 카메라는 Picamera2/libcamera의 OV5647로 확인됐습니다. ROS Image 토픽 발행은 별도 연결이 필요합니다.

USB/V4L2 카메라인 경우 아래처럼 먼저 장치와 지원 모드를 확인합니다.
CSI/libcamera 전용 장치는 해당 장치의 ROS 드라이버가 필요합니다. 이 경우에도
출력이 원본 `sensor_msgs/msg/Image`이면 아래 실험에 사용할 수 있습니다.

```bash
sudo apt install v4l-utils ros-jazzy-usb-cam
v4l2-ctl --list-devices
read -r -p '확인한 카메라 장치 경로: ' camera_device
v4l2-ctl --device="$camera_device" --list-formats-ext
v4l2-ctl --device="$camera_device" --list-ctrls
```

장치가 지원하는 너비·높이·FPS 및 usb_cam 변환 포맷을 입력합니다.
usb_cam의 `pixel_format`은 장치 FOURCC와 이름이 다를 수 있습니다. 예를 들어
YUYV 장치는 `yuyv` 변환을 사용할 수 있으나 실제 드라이버 지원 포맷을 확인하세요.
초점·줌을 고정하고, 가능하면 자동 노출도 끈 뒤 충분한 조명에서 촬영합니다.
정확한 제어 이름과 값은 위 `--list-ctrls` 결과를 사용합니다.

```bash
read -r -p '지원 영상 너비(px): ' camera_width
read -r -p '지원 영상 높이(px): ' camera_height
read -r -p '지원 FPS: ' camera_fps
read -r -p 'usb_cam pixel_format: ' camera_format
read -r -p '카메라 식별명(공백 없이): ' camera_id
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/camera -p video_device:="$camera_device" \
  -p image_width:="$camera_width" -p image_height:="$camera_height" \
  -p framerate:="$camera_fps" -p pixel_format:="$camera_format" \
  -p camera_name:="$camera_id"
```

다른 터미널에서 실제 토픽을 확인합니다. 아래 예시는 `/camera/image_raw`입니다.
다르면 `record --image-topic`과 `--info-topic`을 바꿉니다. 압축 영상이나
`image_rect`가 아닌 **원본 왜곡 영상**을 사용합니다.

```bash
ros2 topic list -t
ros2 topic info /camera/image_raw --verbose
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view  # 필요 시 ros-jazzy-rqt-image-view 설치
```

영상이 안 나오면 보정부터 진행하지 말고 장치 권한(video 그룹), 장치 점유,
모드/포맷, 토픽과 DDS 통신을 먼저 확인합니다. `record`는 sensor-data QoS로
best-effort/reliable 발행자 모두의 영상을 구독합니다.

## 2. 체커보드와 촬영 조건

- **가로 9 × 세로 6 내부 코너**: 칸 수로는 10 × 7입니다.
- 사용자가 확인한 한 칸은 **30 mm**입니다. `--square-mm 30`을 명시합니다.
  인쇄 배율과 종이 변형을 확인하고 여러 칸의 전체 길이를 재어 칸 수로 나누세요.
  실측값이 달라지면 새 값을 입력합니다. 코드에는 칸 크기 기본값이 없습니다.
- 판을 평평한 단단한 바탕에 붙이고, 모든 내부 코너와 바깥 테두리가 보이게 합니다.
- 중앙·상하좌우·영상 가장자리, 가까운/먼 거리, 좌우/상하 기울기를 고르게 촬영합니다.
  같은 위치를 유지하기만 하면 프레임 수가 많아도 보정에 충분하지 않습니다.
- 판을 옮긴 후 잠깐 정지해 흔들림·롤링셔터 영향을 줄입니다. 초점, 줌, 해상도,
  crop/binning, 장착 카메라는 학습·검증·실사용 내내 유지합니다.

## 3. 학습 실험: 원본 기록 + 공식 GUI

기록 터미널에서 다음을 실행합니다. `--camera-settings`에는 실제 설정을 입력하고
검증 때 그대로 사용합니다. 세션 디렉터리는 매번 새 이름을 사용합니다.
`calibration_data/`는 git에서 제외됩니다. 데이터는 별도로 백업하세요.

```bash
read -r -p '카메라 식별명: ' camera_id
read -r -p '실제 해상도/포맷/초점/노출 설정 기록: ' camera_settings
ros2 run pinky_soccer_calibration camera_intrinsics record \
  --session calibration_data/intrinsics_01 --purpose calibration \
  --square-mm 30 --camera-id "$camera_id" --camera-settings "$camera_settings" \
  --duration 600 --interval 1
```

동시에 GUI 터미널에서 실행합니다. DISPLAY가 있는 환경이 필요합니다.

```bash
ros2 run pinky_soccer_calibration camera_intrinsics gui \
  --session calibration_data/intrinsics_01
```

공식 GUI에서 X/Y/Size/Skew 지표를 채우면서 **최소 15개, 권장 25개 이상 서로 다른 자세**를
수집하고 **CALIBRATE → SAVE**를 누릅니다. **COMMIT은 누르지 않습니다**. 검증 전
드라이버 설정을 바꾸지 않기 위해 이 도구는 서비스 확인 없이 실행합니다.
GUI가 보정에 채택한 영상과 `ost.yaml`은 `/tmp/calibrationdata.tar.gz`에 저장됩니다.
이 경로는 GUI를 실행한 컴퓨터에 있으며 다음 SAVE에서 덮어써지므로 즉시 보존합니다.
기록 프로세스는 SAVE 후 `Ctrl+C`로 종료하거나 지정 시간이 끝날 때까지 기다립니다.
GUI가 다른 PC에 있으면 archive를 기록 세션이 있는 컴퓨터로 복사합니다.

```bash
ros2 run pinky_soccer_calibration camera_intrinsics import \
  --session calibration_data/intrinsics_01 --archive /tmp/calibrationdata.tar.gz
ros2 run pinky_soccer_calibration camera_intrinsics check \
  --session calibration_data/intrinsics_01 \
  --calibration calibration_data/intrinsics_01/gui/camera.yaml
```

학습 검사는 GUI가 채택한 원본 영상에 대해 실행됩니다. `record`의 시간 간격 PNG는
별도로 보존되며 GUI 채택 여부와 같지 않습니다. `check`는 항상 새 `check/`를 만들고
기존 결과를 덮어쓰지 않습니다. 실패하면 원본을 보존한 채 새 세션으로 재촬영하세요.

## 4. 독립 검증: 새 촬영으로만 검사

GUI를 종료하고 같은 카메라 설정으로 **새 자세**를 촬영합니다. 검증 영상으로
CALIBRATE를 실행하지 않습니다. 아래 명령은 기록 터미널의 변수를 사용합니다.
새 터미널이면 `camera_id`와 `camera_settings`를 동일한 실제 값으로 다시 입력합니다.

```bash
ros2 run pinky_soccer_calibration camera_intrinsics record \
  --session calibration_data/intrinsics_01_validation --purpose validation \
  --square-mm 30 --camera-id "$camera_id" --camera-settings "$camera_settings" \
  --duration 90 --interval 2
ros2 run pinky_soccer_calibration camera_intrinsics check \
  --session calibration_data/intrinsics_01_validation \
  --calibration calibration_data/intrinsics_01/gui/camera.yaml
```

검사 내용:

- 학습 최소 15장 / 검증 최소 8장, 동일 바이트 PNG 중복 제외. 검증 영상 재사용은
  내보내기 시에도 거부합니다. 별도로 촬영했는지는 실험자가 확인해야 합니다.
- 카메라 행렬·왜곡계수 유한성, 양의 초점거리, 영상 내부 주점, 촬영 해상도 일치.
- 위치 다양성: 보드 중심의 X/Y 범위가 각각 영상 크기의 25% 이상.
  내부 코너 외곽 면적 최대/최소 비 1.5 이상, 최대 보드 기울기 15° 이상.
- 고정된 K/D로 `solvePnP` 보드 자세만 추정. 코너 잔차 `(du, dv)`와 영상별
  `RMS = sqrt(mean(du²+dv²))`를 계산합니다. **각 영상 RMS 0.7 px 이하**가 초기 기준입니다.
  `--max-rms-px`로 명시적으로 바꿀 수 있고 사용한 기준이 보고서에 저장됩니다.
- 영상별 RMS의 평균·표본 표준편차·최댓값, 검출 실패/중복 영상 목록.
- 학습 영상 재추정의 파라미터 표준편차(진단용 OpenCV `calibrateCameraExtended`).
  이는 GUI 결과 자체의 불확실도나 독립 반복 실험 표준편차와 같지 않습니다.
- 영상 범위에서 방사 왜곡 모델의 접힘 여부 검사. 어안 모델은 이 단계에서 지원하지 않습니다.

이 기준은 초기 품질 점검용이며 절대 위치/거리 정확도를 보증하지 않습니다.
낮은 재투영 오차만으로 잘못된 칸 크기나 평평하지 않은 판을 검출할 수 없습니다.
보드 자세는 **보드→카메라** 변환이며 `base_link` 기준 외부 보정값이 아닙니다.

`check/previews/`의 코너 표시가 모든 교점에 맞는지, `undistorted-*.png`에서
직선이 펴지는지와 가장자리 왜곡을 사람이 확인합니다. 기본 K 그대로 왜곡을 제거하므로
가장자리 검은 영역이 생길 수 있습니다. 실패 영상은 `views.csv`에서 확인합니다.
`check` 실패 시 종료 코드는 2이며 `report.yaml`에 이유가 남습니다.

두 검사 PASS와 시각 확인 후에만 내보냅니다.

```bash
ros2 run pinky_soccer_calibration camera_intrinsics export \
  --training calibration_data/intrinsics_01/check/report.yaml \
  --validation calibration_data/intrinsics_01_validation/check/report.yaml \
  --visual-check
```

## 5. 반복 실험·저장 결과·적용

학습과 독립 검증을 **각각 새로 촬영하여 3회 이상 권장**합니다. 예를 들어
`intrinsics_02`, `intrinsics_03`와 각 `_validation` 세션을 만듭니다.

```bash
ros2 run pinky_soccer_calibration camera_intrinsics summarize \
  --reports calibration_data/intrinsics_01/check/report.yaml \
            calibration_data/intrinsics_02/check/report.yaml \
            calibration_data/intrinsics_03/check/report.yaml \
  --output calibration_data/intrinsics_repeat_summary.yaml
```

`fx/fy/cx/cy`(px), `k1/k2/p1/p2/k3`에 대해 평균·표본 표준편차(`ddof=1`)를 계산합니다.
같은 카메라·설정·해상도의 독립 학습 실험만 합칩니다. 통계 평균을 자동으로 적용하지
않으며, 각 실험의 독립 검증 결과도 함께 검토해 사용할 모델을 선택합니다.

| 파일 | 내용 |
|---|---|
| `session.yaml` | 사용자 입력, 실험 목적, UTC 시작 시각, 토픽, 설정 |
| `acquisition.yaml` | 종료 상태, 수신/표본 수, 실제 해상도·frame·encoding |
| `raw_bag/` | 수신한 Image 및 CameraInfo 원본 CDR, sqlite3 ROS Bag |
| `frames.csv` | 모든 수신 Image의 수신시각·헤더시각·frame·크기·encoding·표본 경로 |
| `frames/*.png` | 지정 간격의 무손실 mono8 표본; 색/원본 encoding은 Bag에 보존 |
| `gui/` | GUI archive 원본, 채택 영상, 표준 ROS `camera.yaml` |
| `check/corners.csv` | 코너별 관측 픽셀 좌표와 재투영 잔차 |
| `check/views.csv`, `report.yaml` | 영상별 오차·검출 결과, 통계·기준·합격 여부·입력 해시 |
| `check/previews/` | 코너 표시와 왜곡 제거 영상 |
| `approved/camera.yaml` | 검증된 ROS CameraInfo 호환 K/D/R/P |
| `approved/calibration.yaml` | 단계별 보정 통합 스키마, 결과·검증 보고서 경로 |

녹화는 수신된 메시지를 기록합니다. 카메라 자체의 프레임 유실까지 보장하지 않습니다.
디스크 공간을 먼저 확인하고 `ros2 bag info <session>/raw_bag`로 수신 수를 확인합니다.
재생은 실제 카메라 발행을 중지하고 `ros2 bag play <session>/raw_bag`로 할 수 있습니다.
종료되지 않은 세션이나 해상도/frame/encoding이 바뀐 녹화는 검사에 사용할 수 없습니다.

적용은 실제 카메라 드라이버가 `camera_info_url`을 지원하는 경우 드라이버 재실행 인자에
다음을 추가합니다. USB 예제의 다른 설정은 촬영할 때와 동일하게 유지합니다.

```bash
# usb_cam 실행 인자에 추가 (camera_name도 보정한 카메라와 같아야 함)
# -p camera_info_url:="file://$HOME/pinky-soccer/calibration_data/intrinsics_01/approved/camera.yaml"
ros2 topic echo /camera/camera_info --once
```

복사한 PC/로봇에서는 절대 경로가 달라지므로 해당 머신의 경로를 사용합니다.
`CameraInfo`의 width/height와 K/D가 파일과 같고 Image와 헤더·시각이 맞는지 확인합니다.
내부 보정은 렌즈/영상의 K/D이고 URDF의 위치·각도를 바꾸는 보정이 아닙니다.
제어의 영상 처리는 이 `CameraInfo`를 구독하거나 `camera.yaml`을 읽도록 연결합니다.

## 단계별 진행 계획

통합 YAML에는 미측정 단계를 `null`로 둡니다. 0이나 임의 값으로 채우지 않습니다.
현재 `camera_extrinsics`, `wheel_measurements`, `odometry`, `lidar`, `imu_ultrasonic`은
**미구현**이며, 이번 내부 보정의 실물 결과를 검증한 뒤 아래 순서로 구현합니다.

| 다음 단계 | 사용자가 측정할 값·방법 | 적용 대상 |
|---|---|---|
| 카메라 장착 | 바닥 체커보드로 높이·roll/pitch만 추정, 위치/축 등록 입력 없이 반복 촬영 | `floor_camera`·`camera_mount` 측정/통계 YAML. 보정 높이/기울기 공 검토 우선·프로필 launch 적용, X/Y/yaw는 모델 유지; `ROS_UI.md` 참조 |
| 바퀴 실측 | 좌우 지름 반복 측정, 바퀴 중심 간격; 공식 URDF·Gazebo 값과 비교 | 자체 보정 YAML; 공식 submodule 수정 없음 |
| 오도메트리 | `/odom` 1 m 전후진은 범퍼 중앙 시작~끝 거리(m) 하나, ±360° 회전은 누적 각도만 분리 입력/저장. 공통 유효 반경/간격 추정·측정 절차는 `ROS_UI.md` 참조 | 실물 bringup `wheel_radius`/`wheel_separation`; 좌우 개별 반경은 제어 지원 확인 필요 |
| LiDAR | 여러 방향의 평면 표적 실측 거리와 `/scan`, 장착 XYZ/방향 측정 | 자체 description/센서 설정 |
| IMU·초음파 | 정지 반복과 알려진 회전각, 거리·방향별 표적/미검출 기록 | 자체 센서/제어 설정 |

후속 단계에도 ROS Bag·CSV 원본, 단위와 실측값, 반복 평균·표준편차, 검증 기준을
유지합니다. 각 단계의 실제 결과를 확인하기 전 다음 단계 실행이나 기존 설정 자동 적용은 하지 않습니다.

## 개발 검증과 참고

```bash
colcon test --packages-select pinky_soccer_calibration
colcon test-result --verbose --test-result-base build/pinky_soccer_calibration
```

자동 테스트의 합성 영상·숫자는 코드 검증용이며 실물 보정 결과가 아닙니다.
실물 GUI, 카메라 장치/드라이버, 조명·체커보드 품질은 현장에서 확인해야 합니다.

- [ROS 공식 단안 보정 도구](https://github.com/ros-perception/image_pipeline/tree/jazzy/camera_calibration)
- [usb_cam 실행과 지원 포맷](https://github.com/ros-drivers/usb_cam/tree/ros2)
- [OpenCV 보정·재투영 API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
# 새 ROS 올인원 UI

실행: `ros2 run pinky_soccer_calibration calibration_ui`. 장치 연결은 launch에서 수행합니다. 안내·실험·적용 범위와 아직 남은 기능은 [ROS_UI.md](ROS_UI.md)를 참고하세요.
LiDAR 탭의 **현재 LiDAR 경기장 모양 저장**은 프로젝트에 원본 Bag/CSV와
Nav2 형식 `map.yaml`/`map.pgm`을 저장합니다. 정지·경기장 정리 확인 후 기록 시작 없이 사용할 수
있으며, 단일 스캔의 미관측 영역과 로봇/센서 기준 좌표를 보존합니다.
준비·좌표·파일 설명은 [ROS_UI.md](ROS_UI.md#lidar-경기장-모양-저장)를 참고하세요.

**장착** 탭에는 내부 보정과 독립된 체커보드 규격 입력란이 있습니다. 다른 보드로 바꾸어도
내부 보정값과 사진은 유지되며, 장착 사진은 각자의 촬영 규격으로 저장·재계산됩니다.

# 실물 제어 캘리브레이션 확장 (2026-11-25)

기존 `calibration_ui`에 탭 두 개를 추가했습니다. 기존 탭·프로젝트·`profile.yaml` 형식은 그대로이며,
새 측정은 같은 프로젝트(`project.yaml` measurements)에 이어서 쌓입니다. MPC는 변경하지 않습니다.

## 연결·시간 탭 (PC 중심 시간 동기화)

구조: 인터넷 NTP → **노트북 PC**(chrony 클라이언트 + LAN 서버) → 각 Pinky(PC의 chrony 클라이언트).

- PC IPv4·인터페이스·ROS_DOMAIN_ID·RMW, 로봇 식별명·Pinky IPv4·namespace·SSH 사용자를 직접 입력하고 저장합니다
  (`~/.config/pinky_soccer/calibration_connection.yaml`, 보정 결과와 별개). IP는 코드에 고정되어 있지 않습니다.
- PC 인터페이스 목록은 참고로만 보여 주며 입력값을 덮어쓰지 않습니다.
- `설정 명령 복사`: 입력값으로 만든 chrony.conf 줄(PC `allow <Pinky>`, Pinky `server <PC> iburst prefer minpoll 2 maxpoll 4`),
  재시작·확인 명령, CycloneDDS peer 항목, UI 재시작 명령. **UI는 관리자 명령을 실행하지 않고 파일을 고치지 않습니다.**
- 상태는 각각 따로 판정합니다: ROS 연결 · 필수 센서 · PC 절대 동기화 · Pinky 절대 동기화 · Pinky가 지정 PC를 기준으로
  사용 · 상대 동기화 품질 · 단방향 지연 측정 가능 여부. 동기화가 안 되어도 카메라 등 다른 보정은 막지 않습니다.
- 상대 품질 상한(초) = 로봇 system time 잔차 + 로봇 RMS offset + (로봇 root delay − PC root delay)/2 + PC 소스 std dev.
  기준은 기존과 같은 5 ms이며 완화하지 않았습니다. `chronyc ntpdata`는 root가 필요해 쓰지 않습니다. 정보가 부족하면
  "미검증"입니다. Pinky가 예비 인터넷 서버로 넘어가면 "PC 기준 아님"입니다.
- `측정 → 프로젝트에 기록 추가`: N초 동안 상태를 표본으로 모아 `clock_sync` 기록을 추가합니다(이력 보존).
  예전 `results.time_sync`(당시 chronyc 미설치 기록)는 바꾸지 않습니다.
- namespace·DDS는 프로세스 시작 시에만 적용됩니다. 설정과 다르면 화면에 재시작 필요가 표시됩니다.
  `ros2 launch pinky_soccer_calibration calibration_ui.launch.py robot_namespace:=auto pc_ip:=`(빈 값=저장 설정).
- **로봇 쪽**: sources/sourcestats를 함께 보내는 `time_sync_status`는 이 커밋의 `pinky_soccer_bringup`이 필요합니다.
  예전 bringup이면 "PC 기준 사용" 여부만 판정되고 상대 품질은 미검증으로 남습니다.

## 모터 동역학 탭 (계단응답 자동 실험)

1. 프로젝트를 열고 `기록 시작/재개`(원본 Bag/CSV 필수).
2. 종류(직진·후진·제자리 좌/우·복합), 크기, 정지 준비/명령 유지/정지 후 기록 시간(기본 1/2/2 s), 반복, 최대 이동 거리,
   용도(식별용/독립 검증용), 바닥 재질을 입력하고 로봇·namespace를 확인해 주행 허용 후 실행합니다.
3. 상단 `■ 즉시 정지`로 언제든 멈춥니다.

- 안전: 한 번에 하나의 주행만, `/cmd_vel`에 다른 발행자(BT/MPC/teleop)가 있으면 거부, 기본 |v|≤0.20 m/s·|w|≤2.0 rad/s
  (고속 승인 시 0.25/3.0), 바퀴 0.9×103 rpm, 최대 이동 거리·회전각·시간 제한, odom/joint_states 끊김 시 정지,
  UI 종료 시 정지. 통신이 끊기면 PC 정지 명령은 도달하지 않으므로 로봇 `command_watchdog`(0.5 s)가 정지를 담당합니다.
- 기록: 실험 시작 시 `motor_response` 기록을 `running`으로 먼저 저장하고, 끝나면 `complete`/`user_stopped`/
  `safety_stopped`/`system_error`로 확정합니다. 프로그램이 도중에 꺼지면 다음에 불러올 때 `interrupted`로 표시하며
  주행은 자동으로 재개하지 않습니다.
- 파일: `motor_response/<실험 ID>/observations.csv`(joint_states 위치·속도, odom, 수신한 cmd_vel, IMU, 배터리;
  수신 monotonic·헤더 시각), `commands.csv`(발행한 모든 명령의 monotonic·ROS·Unix 시각), `meta.yaml`. 연속 원본은
  해당 기록 세션의 `raw_bag*`/`observations*.csv`에 그대로 있습니다.
- 실험에 쓴 바퀴 반경·간격과 출처(프로젝트 wheels 결과 또는 공식 기본값)·project revision을 기록합니다.
- 모델 식별(램프+1차 지연, 지연)·그래프·독립 검증은 아직 구현하지 않았습니다. 재계산은 현재 기록 집계만 합니다.
- 새 기록은 `profile.yaml`을 보관 처리하거나 그 결과에 섞이지 않습니다(`CONTROL_CATEGORIES`).

# 우선순위 2: 통신·명령 경로·제동·외형 (2026-11-25)

## 실물에서 읽을 수 있는 모터·엔코더 데이터 (공식 코드 분석, 실물 미확인)

| 항목 | 공식 드라이버 (`pinky_bringup/bringup.py`, `dynamixel_driver.py`) |
|---|---|
| JointState | 30 Hz 타이머. `velocity` 있음 = Present Velocity(주소128, 0.229 rpm 단위)→rad/s. `position` = Present Position(132)/4096×2π. **오른쪽은 원시 부호(전진 시 −)**, 이름 `left/right_wheel_joint` → relay가 `l/r_wheel_joint`로 바꿈 |
| 헤더 시각 | 노드 시계로 읽기 직전 시각 (엔코더 샘플 시각에 가까우나 모터 내부 샘플 시각은 아님) |
| 명령 | `drive/cmd_vel` 수신 즉시 Goal Velocity(104) 동기 쓰기, rpm→정수 절사(0.229 rpm), 큰 쪽 바퀴가 100 rpm 넘으면 두 바퀴 비례 축소. 오른쪽 부호 반전 |
| Profile Acceleration | 시작 시 200 (≈74.9 rad/s²) |
| 명령 경로 | PC `cmd_vel` → 로봇 `command_watchdog`가 **최신 명령을 20 Hz(50 ms)로 재발행**, 0.5 s 무명령 시 0 → `drive/cmd_vel` → 드라이버 |
| 모터 모델 | 코드에 없음 (사양상 XL330-M288-T). `motor_diagnostics:=true`면 모델 번호(주소0)를 읽어 `motor/info`로 발행 |

`motor_diagnostics:=true` (로봇 bringup, 기본 false): 공식 `Pinky` 클래스를 같은 프로세스에서 실행하고 피드백
읽기만 124~145 연속 블록으로 넓힙니다 → Present PWM·Current·Velocity·Position·**Velocity Trajectory(모터 내부 램프
목표)**·Position Trajectory·Input Voltage를 같은 시각에 `motor/diagnostics`(JSON)로 발행. 포트는 한 프로세스만
사용합니다. 주소는 X 시리즈(XL330) 표 기준이며 실물에서 값이 맞는지 아직 확인하지 않았습니다.
`motor_feedback_rate_hz`(기본 30=공식)는 50~100 Hz 실험용이며 하드웨어 검증 전입니다.

접근하지 않는 것: Goal Velocity 읽기(쓰기 값은 명령으로 기록), 모터 펌웨어 내부 제어 주기, 실제 토크.
Wi-Fi 구간 지연은 Ping-Pong과 명령 태그로만 측정합니다.

## 실물 실험 순서 (권장 입력·반복)

| 순서 | 탭 | 실험 | 입력 | 반복 |
|---|---|---|---|---|
| 0 | 연결·시간 | IP 저장, 상태 확인, 동기화 기록 | 60 s | 1 (실험 전·후) |
| 1 | 통신 지연 | Ping-Pong | 10, 20, 50 Hz · 20 s · 0 B | 각 3 |
| 2 | 모터 동역학 | 직진 저속 | forward 0.05 m/s · 1/2/2 s · 최대 0.3 m | 3 (첫 실물은 1회 후 정지 확인) |
| 3 | 모터 동역학 | 직진 | forward 0.10, 0.15 (0.20) | 각 3 |
| 4 | 모터 동역학 | 후진 | backward 0.05, 0.10 | 각 3 |
| 5 | 모터 동역학 | 제자리 회전 | spin_left/right 0.5, 1.0 (2.0) rad/s | 각 3 |
| 6 | 모터 동역학 | 복합 | combined 0.10 m/s + 1.0 rad/s | 3 |
| 7 | 모터 동역학 | 제동 | brake_forward 0.05~0.20 (0.25는 승인 후) · 유지 상한 4 s | 각 3 + 줄자 거리 |
| 8 | 모터 동역학 | 독립 검증용 | 위 중 일부를 '독립 검증용'으로 | 각 2 |
| 9 | 통신 지연 | 명령 경로 분석 | (2~8의 기록 대상) | — |
| 10 | 충돌 외형 | 줄자 실측 | 전후좌우 + 바퀴/범퍼 모서리 외곽점 | 2~3 |
| 11 | 바퀴 | (선택) 추가 관측 | 전후 X·좌우 Y·최종 자세각 | 기존과 같이 |

모든 주행: 기록 시작/재개 → 로봇·namespace 확인 체크 → 실행. BT/MPC/teleop는 끕니다(시작과 실행 중 모두 감시해
발견 시 안전 정지). 바퀴 0.9×103 rpm, 기본 0.20 m/s·2.0 rad/s, 최대 이동 거리·각·시간 제한.

## 저장 위치 (프로젝트 폴더 안, 상대 경로)

- `motor_response/<ID>/observations.csv`, `commands.csv`(명령 태그 포함), `meta.yaml`
- `communication_latency/<ID>/pingpong.csv`
- 연속 원본: `raw_bag*`, `observations*.csv` (기록 세션별)
- 분석 기록(`command_latency`, `braking`)은 원본 실험 ID만 참조합니다(복사 없음). 원본 기록을 지우면 참조 분석은
  `source_deleted`·재계산 필요가 됩니다. 충돌 외형·시간 동기화·통신 기록은 `project.yaml` measurements에 있습니다.

## 재개

UI 실행 → 저장 폴더 불러오기(같은 프로젝트) → 기록 시작/재개 → 이어서 실험. 비정상 종료된 실험은 `interrupted`로
표시되며 자동 재주행하지 않습니다. 측정 기록 탭에서 기록 삭제·재계산.

## 빌드·실행

```bash
# PC
cd ~/pinky-soccer && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_msgs pinky_soccer_bringup pinky_soccer_calibration
source install/setup.bash
ros2 launch pinky_soccer_calibration calibration_ui.launch.py      # robot_namespace:=auto pc_ip:=

# Pinky (pinky_soccer_bringup·pinky_soccer_msgs·pinky_soccer_description 소스 복사 후)
colcon build --packages-select pinky_soccer_msgs pinky_soccer_description pinky_soccer_bringup
source install/setup.bash
ros2 launch pinky_soccer_bringup robot.launch.py motor_diagnostics:=true   # 진단 끄면 공식 드라이버
```

수신 주기 확인(실물): `ros2 topic hz /joint_states`, `ros2 topic hz /motor/diagnostics`,
`ros2 topic echo --once /motor/info`, 주기 탭의 10 s 수집.

## 실물 미검증

모든 신규 주행·계측(계단응답·제동·Ping-Pong·명령 태그·모터 진단 블록 주소·피드백 주기 변경·운행 중 발행자 감시)은
MuJoCo/단위 테스트로만 확인했습니다. 아래 모터 식별·좌우 개별 반경도 합성 자료·MuJoCo 자료로만 검증했습니다.

## 모터 응답 분석 창 (오프라인, 로봇 연결 불필요)

모터 탭 → `분석 창 열기`. 저장된 `motor_response` 기록을 읽기만 하며 주행·구독이 없습니다.

- 실험 목록에서 선택(없으면 전체) → `식별·검증`. 용도 `validation` 실험은 식별에 쓰지 않고 검증에만 씁니다.
  검증용 실험이 없으면 결과는 `미검증`입니다.
- 모델(MPC `ramp_predictor`와 같은 5 ms 단계식): `first_order`, `first_order_delay`, `ramp_first_order`,
  `ramp_first_order_delay`. 파라미터: `tau_left/right`, `motor_accel`(내부 램프 기울기), `delay`(유효 지연).
- 시간축: `/motor/diagnostics`와 태그된 명령 수신이 있으면 `robot_clock`(로봇 시각, 단계별
  Goal→Velocity Trajectory→Present Velocity 식별), 없으면 `pc_receive`(PC 수신 시각, 지연에 Wi-Fi·watchdog
  50 ms 유지·드라이버가 섞임). Goal은 공식 드라이버 변환(0.229 rpm 단위 버림·100 rpm 비례 축소)으로 재구성하며
  드라이버 반경/간격은 기본 0.027/0.0961 m 가정입니다(실행 값과 다르면 이득·가속 추정이 치우칠 수 있음).
- 식별 불가 판정: 상대 표준편차 > 25 %, 상관 |ρ| > 0.95, 경계값 도달 → `identification_insufficient`로
  표시하고 값을 확정하지 않습니다(예: 작은 계단에서는 램프가 드러나지 않아 `motor_accel` 불충분).
- 자료 품질: 수신 주기, 끊김, 순서 뒤바뀜, 비정상 값(|w| > 25 rad/s) 개수를 실험별로 표시합니다.
- 그래프: 좌/우 명령·측정 램프·실제 속도·모델 예측·잔차. `반복별 식별 비교`는 실험마다 따로 맞춘 값의
  평균·표준편차·변동계수, `제동 통계`는 명령 속도별 odom 제동 거리·정지 시간·실측 거리.
- 저장: 프로젝트 루트 `identification.yaml`(+ `identification/<ID>.yaml` 사본). 실험 ID, 모델 버전, 당시
  바퀴 보정 리비전·값, 단계별 결과, 검증 결과, `applied_to_mpc: false`. 키가 `motor_identification_schema`라
  `pinky_soccer_dynamics`의 `apply_identification`이 읽지 않습니다 — MPC 자동 적용 없음(별도 승인 단계 필요).
  `profile.yaml`은 바뀌지 않습니다. 모터 응답 기록을 추가/삭제하면 이 결과만 stale이 됩니다.

## 좌우 개별 유효 반경 (검토용)

바퀴 탭 → `좌우 개별 반경 추정`. 공통 결과(`wheels`)는 그대로 두고 `wheel_individual`에 따로 저장합니다.
직진 실험의 추가 관측 `최종 heading 변화`(deg)가 있어야 rL−rR이 구별됩니다(없으면 반경 차와 바퀴 속도 차가
같은 엔코더 값을 만듦). 전진·후진 각 포함 직진 4회 이상 + 회전 실험이 필요하며, 부족하면
`insufficient_observations`. 차이가 2σ보다 클 때만 `identified_candidate`, 아니면 `not_significant`.
leave-one-out 예측 오차를 함께 기록하며, 미끄러짐/바닥 영향과 반경 차를 분리하지 못합니다.
