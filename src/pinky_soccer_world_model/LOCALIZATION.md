# LiDAR·골대 색·지자기로 경기장 위치 추정

2026-11-27: 실물·MuJoCo·Gazebo 기본은 같은 [직사각 외곽·HLS·IMU 시작 보정](STARTUP_LOCALIZATION.md)이다.
`game.launch.py localization:=auto`는 세 환경에서 rectangle을 선택한다(Nav2 비교 옵션은 기존 lidar).
아래는 명시적 `localization:=lidar`의 AMCL 경로 설명이다. 이번 시작 보정 추가는 PC 코드만 변경하며 로봇 소스 교체는 필요 없다.

`game.launch.py`의 명시적 `localization:=lidar`는 세 환경에서 같은 구성을 사용합니다.
기존 방식은 `localization:=odom_only`로 선택할 수 있습니다. AMCL은 경기장
YAML에서 생성한 지도와 `/scan`을 비교하고, 바퀴 odom TF로 스캔 사이 움직임을
예측합니다. `map → odom`은 AMCL만, `odom → base_footprint`는 기존 드라이버만
발행합니다. RViz·월드 모델·판단은 같은 보정된 TF를 사용합니다.

지자기는 AMCL 입자 필터 내부에 직접 추가한 관측이 아닙니다. 별도
`goal_heading`이 **현재 후보 `(x,y,θ)`와 대칭 후보 `(-x,-y,θ+π)`를 비교**하고,
명확하고 반복된 증거가 있을 때 `/initialpose`로 올바른 후보를 재초기화합니다.
X/Y를 그대로 두고 방향만 뒤집지 않습니다. 임의의 큰 위치 오차 전체를
이 기능만으로 복구하는 방식은 아니므로 시작 위치·방향의 대략적인 값은 필요합니다.

## 실행

PC 의존 패키지: `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`.
설치되지 않은 PC에서는 먼저 설치합니다.

```bash
sudo apt install ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_bringup pinky_soccer_sim pinky_soccer_world_model pinky_soccer_viz
source install/setup.bash
```

**실물 로봇:** 갱신한 bringup 패키지를 로봇에도 전달해 ARM64에서 빌드합니다.
기존 bringup/IMU 프로세스를 종료한 뒤 로봇에서:

```bash
ros2 launch pinky_soccer_bringup robot.launch.py imu_enabled:=true adc_enabled:=true imu_magnetic_enabled:=true
```

NDOF 전용 드라이버는 I2C 장치를 단독 소유합니다. 같은 BNO055를 읽는 공식
드라이버·Jupyter·별도 스크립트를 함께 실행하면 안 됩니다. 공식 submodule은
수정하지 않았습니다. `imu_magnetic_enabled:=false`이면 기존 공식 IMU 모드와
단위 릴레이로 돌아가며 지자기 기준 학습은 비활성 상태가 됩니다.

**PC 실물:** 로봇과 같은 domain/DDS 환경, 시간 동기화, 올바른 센서 TF가 필요합니다.

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=real field:=hardboard camera_yaml:=/absolute/path/camera.yaml heading_record:=/absolute/path/run01/heading.yaml
```

미보정 시험만 `allow_uncalibrated:=true`를 명시해 HFOV 가정을 허용할 수 있습니다.
그 경우 방향 기록에도 `explicit_approximate_hfov`가 남으며 실측 보정으로 취급하지 않습니다.
영상이 광학 축과 180° 뒤집혀 있으면 경기 검출기와 같은 `image_rotated_180:=true`를 사용합니다.

**Gazebo / MuJoCo:** 실물과 분리된 domain118, 해당 터미널에서 CycloneDDS XML 해제.

```bash
export ROS_DOMAIN_ID=118
unset CYCLONEDDS_URI
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco field:=hardboard
# 또는 mode:=gazebo
```

기본 IMU 활성화, 기본 정지 대기입니다. 시작 위치가 중앙/+X가 아니면
`initial_x`, `initial_y`, `initial_yaw`(m·rad)를 지정합니다. `our_goal_color`는
공격/수비 지정이며 지도의 파랑=+X·노랑=−X 규칙을 바꾸지 않습니다.
센서 기준이 잡혔는지 확인하고 기존 `/game/playing` 시작 신호를 사용합니다.

## 자동 기준 학습과 확인

1. 바닥이 수평이고 로봇/카메라 장착이 고정된 상태에서 한 골대의 **색 패널 전체**가
   화면에 들어오게 합니다. 캘리브레이션된 CameraInfo/파일과 실행 카메라 TF로
   화면 속 골대의 실제 수평 방위를 계산합니다. 파란색이 보인다고 yaw=0으로 가정하지 않습니다.
2. AMCL 위치 분산 X/Y 각각≤0.04m², yaw≤0.12rad²일 때 두 대칭 후보에서 해당
   골대가 보여야 하는 방향을 비교합니다. 현재 검출은 HSV·넓은 직사각형 패널이며
   색만 맞는 임의의 물체를 확실히 구별하는 학습 검출기는 아닙니다.
3. 카메라/지자기 시각 차이≤0.1초, 자기 보정 상태≥2, 자기장10~100µT,
   수평 성분≥5µT, 가속도 크기가 중력에서±1.5m/s² 이내인 표본만 사용합니다.
4. 골대 방위 잔차≤0.25rad, 후보 간 오차 차이≥0.7rad인 독립 영상5장 이상에서
   자기 방향과 경기장 방향의 차이가 일관되면 기준을 기록합니다.
5. 이후 골대가 가려져도 그 기준을 사용합니다. 방향 차이145° 이상이5회 지속되고
   자기장 크기가 기준의±20% 이내일 때만 대칭 후보로 재초기화합니다. 최소5초 간격을 둡니다.
   보통의 작은 yaw 오차를 자기장으로 강제로 덮어쓰지 않습니다.

```bash
ros2 topic echo /localization/status
ros2 topic echo /imu/status
ros2 run rqt_image_view rqt_image_view /localization/goal_image/compressed
ros2 topic echo /amcl_pose
```

`heading_anchor_ready: true`가 기준 생성 완료입니다. `magnetic_ready`는 유효한
최근 자기 표본, `lidar_pose_ready`는 최근 AMCL 자세/분산 조건입니다. status의
`record` 경로에 최신 기준 YAML과 관측/교정 JSONL이 저장됩니다. 골대 색,
영상 시각/방위, 자기 방향·크기/보정 상태, 선택 자세, 두 후보 잔차, 지도 치수,
카메라 보정/가정 출처를 보존합니다. 기본 경로는 실행 PC의
`~/.ros/pinky_soccer/localization/<실행별 폴더>/heading_reference.yaml`이며 재실행해도
과거 기록을 덮어쓰지 않습니다. `heading_record:=...`로 다른 저장 위치를 지정할 수 있습니다.
다른 장소/세션의 자기 기준은 자동 재사용하지 않습니다.

## 이상 상태와 실물 검증

자기장 급변이 gyro와 맞지 않으면 표본을 버립니다. 0.5초 이상 유효 자기 표본이
끊기거나 시뮬 시간이 되감기면 기준을 무효화하고 색상 관측으로 다시 학습합니다.
자기 기준이 없어도 LiDAR/odom 위치 추정은 계속되지만 180° 식별 완료를 주장하지 않습니다.
AMCL/odom 수신이 끊기면 월드 모델은 경기 허용을 내리지 않습니다. AMCL 재초기화로
큰 위치/방향 변경이 생기면 이전 지도 좌표의 공·상대 추정과 탐색 믿음을 초기화합니다.

센서 장착 상태에서 BNO055 보정 상태를 확인하고, 모터 정지/전후진/회전 및
경기장 여러 위치에서 자기 방향이 안정적인지 검증해야 합니다. 주변 철제 구조물,
모터의 자기장 때문에 나침반이 항상 정확하다고 가정할 수 없습니다. 골대 HSV 범위도
실제 파랑/노랑 패널과 조명에서 조정합니다. 잘린 패널·원형 물체·비슷한 크기의
동색 패널이 여러 개면 기준 학습을 거부합니다.

시뮬레이션 지자기는 IMU 자세에 따라 회전하는 **이상적인 균일 자기장**입니다.
기본 월드 자기장 `[22,0,-42]µT`는 실측값이 아닌 명시적 시험 조건이며 센서 adapter의
`magnetic_field_world_tesla`로 바꿀 수 있습니다. 하드웨어 간섭·노이즈 정확도 검증은 별도입니다.
정답 `/ground_truth`와 `/sim/ball_ground_truth`는 위치 추정 노드가 구독하지 않습니다.
자이로는 자기장 급변 검증에 사용하며, 별도의 wheel+gyro EKF는 이번 구성에 포함하지 않습니다.

기준: [Bosch BNO055 데이터시트](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf),
[Nav2 Jazzy AMCL](https://ros-navigation.github.io/mkdocs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/).
