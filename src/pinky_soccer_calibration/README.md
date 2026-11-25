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
