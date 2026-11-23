# pinky_soccer_bringup

원본/JPEG **한 프레임 요청 서비스** `/camera/capture_frame`와 파일 저장 명령은
[CAMERA_CAPTURE.md](CAMERA_CAPTURE.md)를 참고하세요. 영상 토픽 구독 없이 사용할 수 있습니다.

범퍼를 장착한 Pinky Pro의 구동부와 센서를 **실물 로봇**에서 실행하는 패키지입니다.
공식 `pinky_bringup bringup_robot.launch.xml` 대신 사용하며, 공식 Submodule은 수정하지 않습니다.

로봇 형상은 `pinky_soccer_description`, 시뮬레이션은 `pinky_soccer_sim`이 담당합니다. 이 패키지에는 시뮬레이터 의존성이 없습니다.

## 실행

로봇(라즈베리파이)에서:

```bash
ros2 launch pinky_soccer_bringup robot.launch.py
```

기존 모터·LiDAR·배터리에 **카메라가 기본 추가**됩니다. Jupyter 없이 로봇의
`pinkylib.Camera`를 직접 실행해 ROS 영상 토픽을 발행합니다.
이미 다른 프로그램이 카메라를 사용 중이면 먼저 해당 프로그램에서 카메라를 닫습니다.

모터·LiDAR를 실행하지 않고 카메라만 먼저 확인하려면:

```bash
ros2 launch pinky_soccer_bringup sensors.launch.py
```

기존 구성처럼 카메라 없이 실행하려면 `camera_enabled:=false`를 넘깁니다.
IMU(BNO055)·센서 ADC의 배선/드라이버 호환성을 확인한 로봇에서는 다음처럼 활성화합니다.

```bash
ros2 launch pinky_soccer_bringup robot.launch.py imu_enabled:=true adc_enabled:=true
# 센서만 확인할 때도 같은 인자를 사용할 수 있음
ros2 launch pinky_soccer_bringup sensors.launch.py imu_enabled:=true adc_enabled:=true
```

IMU와 ADC는 실물 호환성 미확인으로 **기본 false**입니다. 활성화하면 실제 공식 드라이버를
실행합니다. 비활성화된 센서나 없는 장비에 대한 가상 측정값은 발행하지 않습니다.

PC에서 드라이버 없이 description만 확인:

```bash
ros2 launch pinky_soccer_bringup robot.launch.py hardware:=false
```

공식 `bringup_robot.launch.xml`과 **동시에 실행하지 않습니다.** 같이 실행하면 robot_state_publisher가 두 개 떠서 TF가 충돌합니다.

## 로봇에 설치

```bash
git clone --recursive https://github.com/cse1598/pinky-soccer.git
cd pinky-soccer
source /opt/ros/jazzy/setup.bash

# 시뮬레이션 패키지(pinky_soccer_sim, 공식 pinky_gz_sim·pinky_mujoco)는 빼고 의존성 설치
rosdep install --ignore-src -y --from-paths \
  src/pinky_soccer_bringup src/pinky_soccer_description \
  src/pinky_pro/pinky_bringup src/pinky_pro/pinky_description \
  src/pinky_pro/pinky_imu_bno055 src/pinky_pro/pinky_sensor_adc \
  --skip-keys "pinky_gz_sim pinky_mujoco"

# 공식 pinky_bringup의 시뮬레이터 의존성까지 따라가지 않도록 명시적으로 선택
colcon build --symlink-install --packages-select \
  pinky_description pinky_bringup pinky_soccer_description pinky_soccer_bringup
source install/setup.bash
```

`rosdep install --from-paths src`처럼 `src` 전체를 지정하면 공식 `pinky_gz_sim`과 `pinky_mujoco`의 의존성(Gazebo 등)까지 설치됩니다. 위처럼 필요한 경로만 지정합니다.

`sllidar_ros2`, `dynamixel_sdk`, `pinkylib`처럼 공식 bringup이 쓰는 드라이버는 공식 Pinky 설정대로 로봇에 설치되어 있어야 합니다.

카메라는 기존 로봇 OS의 `pinkylib`·Picamera2/libcamera를 사용합니다. 동명의 pip 패키지를
임의 설치하거나 현재 동작하는 libcamera를 교체하지 않습니다.
선택형 IMU·ADC는 ARM64에서만 실행 파일이 빌드되며, 로봇에 공식 WiringPi 환경이 필요합니다.
준비된 로봇에서 추가로 빌드합니다(PC에서는 하드웨어 실행 파일을 만들지 않습니다).

```bash
colcon build --symlink-install --packages-select pinky_imu_bno055 pinky_sensor_adc
source install/setup.bash
```

## 공식 launch와의 관계

| 구성 | 처리 |
|---|---|
| `upload_robot.launch.py` (rsp + jsp) | 같은 설정으로 재구성하고, xacro만 `soccer_robot.urdf.xacro`로 교체 |
| `sllidar_ros2` `sllidar_c1_launch.py` | 공식과 같은 인자로 include (`/dev/ttyAMA0`, `rplidar_link`, DenseBoost) |
| `pinky_bringup` `bringup` | 공식 `pinky_params.yaml` 사용. `joint_states` 출력만 `wheel_joint_states_raw`로 remap |
| `pinky_bringup` `battery_publisher` | 그대로 |
| `wheel_joint_state_relay` | 추가 노드 (아래 참고) |
| `sensors.launch.py` | 직접 카메라 발행, 광학 TF, 선택형 공식 IMU·ADC와 단위 변환 노드 추가 |

## 영상·센서 토픽

namespace를 주지 않은 기본 이름입니다. 모든 물리 장치 노드는 로봇에서 실행하고,
PC의 보정·YOLO·판단은 토픽을 구독합니다. 이 launch는 YOLO나 축구 판단 노드를 실행하지 않습니다.

| 토픽 | 타입 | 조건·의미 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | 카메라, 무손실 원본 BGR8 배열 |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 카메라, JPEG 전송용 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내부 보정값; 미보정이면 K/D/R/P가 0/빈 값 |
| `/imu_native` | `sensor_msgs/Imu` | `imu_enabled:=true`; 공식 BNO055 출력, **각속도 deg/s** |
| `/imu_raw` | `sensor_msgs/Imu` | `imu_enabled:=true`; 래퍼가 각속도를 **rad/s**로 변환 |
| `/us_sensor/range` | `sensor_msgs/Range` | `adc_enabled:=true`; 공식 초음파 변환값(m), 실측 보정 필요 |
| `/ir_sensor/range` | `std_msgs/UInt16MultiArray` | `adc_enabled:=true`; IR 3채널 원시 ADC, 거리(m)가 아님 |
| `/batt_state` | `sensor_msgs/BatteryState` | `adc_enabled:=true`; ADC 드라이버의 배터리 상태 |
| `/scan`, `/odom`, `/joint_states`, `/tf` | 기존 타입 | 기존 실물 bringup과 동일 |
| `/battery/voltage`, `/battery/percent` | `std_msgs/Float32` | 기존 배터리 노드 유지 |
| `/cmd_vel` | `geometry_msgs/Twist` | 기존 구동 명령 **입력** 유지 |

카메라 원본·JPEG·CameraInfo는 같은 프레임과 같은 헤더 시각을 사용합니다.
발행자는 sensor-data QoS(best effort)를 사용합니다. 원본 데이터 복사와 JPEG 인코딩은
각 토픽의 구독자가 있을 때만 수행합니다. PC에서는 JPEG로 먼저 확인해 Wi-Fi 사용량을 줄이고,
정밀 보정·원본 기록이 필요할 때 원본을 구독합니다. 원본도 함께 구독하면 그 전송량은 추가됩니다.
`camera_fps`는 타이머 상한이며 실제 FPS/지연 측정 결과가 아닙니다.

시각은 `get_frame()` 반환 직후의 ROS 시각입니다. 센서 노출 시각 자체가 아니며,
센서 융합 시 실제 지연을 측정하고 PC·로봇 시계를 동기화해야 합니다.
`pinkylib.get_frame()`의 180° 회전을 그대로 유지합니다. 보정 파일도 같은 방향·해상도여야 하며
회전하지 않은 영상의 보정값을 그대로 넣으면 안 됩니다.

`front_camera_link → front_camera_optical_frame`은 `(roll,pitch,yaw)=(-π/2,0,-π/2)`의
좌표축 변환만 추가합니다. 영상 좌표는 오른쪽 +X, 아래 +Y, 전방 +Z입니다.
위치·장착 기울기는 기존 URDF의 추정값을 따르며, 외부 보정이 완료됐다는 뜻은 아닙니다.
`sensors.launch.py`만 실행하면 parent인 `front_camera_link`의 로봇 모델 TF는 별도 실행이 필요합니다.

IMU 단위 변환은 **현재 공식 BNO055 코드**(`/16`의 deg/s 출력)에만 맞춘 것입니다.
각속도 공분산도 `(π/180)²`를 곱합니다. 드라이버를 교체하면 단위 래퍼도 확인해야 합니다.
공식 공분산 상수는 실측으로 검증되지 않았습니다. 초음파 변환식·유효 범위·IR 특성도
공식 기본값이므로 캘리브레이션 전 거리/경계 판단의 정확도를 가정하지 않습니다.
같은 I2C 센서에 별도 프로그램과 이 드라이버를 중복 실행하지 않습니다.

범퍼는 현재 수동 기구물이므로 접촉 스위치 토픽을 만들지 않습니다. 킥 장치·장착되지 않은
장비의 가상 노드, IMU 융합 오도메트리, 자동 주행 판단도 이 변경에 포함하지 않습니다.

## 장착 보정 프로필 적용

보정 UI의 `camera_mount`(바닥 기준 광학 중심 높이·roll/pitch)는 `calibration_profile:=/절대경로/profile.yaml`로 실행하면 자체 URDF/TF에 적용됩니다. 측정하지 않은 카메라 X/Y/yaw는 기존 모델 값을 유지하고 공식 submodule은 바꾸지 않습니다. 전체 `camera_pose`가 들어 있는 이전 프로필은 그 전체 pose를 우선합니다. PC 공 검토는 현재 프로젝트 장착값을 바로 쓰지만 실제 경기/시뮬레이터에는 저장한 프로필을 지정해 launch를 다시 실행해야 합니다. 실물에는 real 프로필만 허용합니다.

## PC에서 영상 확인·보정 연결

로봇과 PC의 실제 `ROS_DOMAIN_ID`, DDS 탐색 범위와 네트워크 설정을 맞춘 후 PC에서:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
ros2 topic info /camera/image_raw/compressed --verbose
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo /camera/camera_info --once --qos-reliability best_effort

# 필요 시 PC에 뷰어·압축 transport 설치
sudo apt install ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport
ros2 run rqt_image_view rqt_image_view
```

뷰어에서 `/camera/image_raw`의 `compressed` transport를 선택합니다. YOLO 노드에서는
CompressedImage를 JPEG 디코딩하거나 원본 Image를 BGR 배열로 변환해 사용할 수 있습니다.
보정·기록용 기존 `camera_intrinsics record`도 같은 원본·CameraInfo 토픽을 사용할 수 있습니다.
Jupyter 프로토타입 GUI의 입력 교체와 YOLO 구현은 아직 별도 작업입니다.

ROS 표준 보정 파일을 얻은 뒤 로봇에 복사하고 적용합니다. 시작 시 크기·모델·행렬을 검사하며
크기가 다른 파일을 자동 확대/축소하거나 측정하지 않은 내부값을 생성하지 않습니다.

```bash
ros2 launch pinky_soccer_bringup robot.launch.py \
  camera_info_url:=file:///실제로/저장한/camera.yaml
```

파일이 없거나 잘못되면 카메라 노드는 오류로 종료합니다. 정상 촬영 중 연속 3회 읽기가
실패해도 카메라 노드를 종료하고 장치를 해제합니다. 오래된 영상으로 대체하지 않습니다.
현재 CameraInfo 파일은 시작 시 읽기 전용입니다. `set_camera_info` 서비스는 제공하지 않으므로
공식 보정 GUI를 사용할 때는 `--no-service-check`로 실행하고 SAVE 결과를 검증 후 적용합니다.

## 바퀴 관절 이름 통일

공식 실물 드라이버는 `left_wheel_joint`/`right_wheel_joint`를 발행하지만, URDF·Gazebo·MuJoCo는 `l_wheel_joint`/`r_wheel_joint`를 사용합니다. 그래서 공식 구성에서는 실물 바퀴가 TF에서 회전하지 않습니다. 이 패키지는 공식 코드를 수정하지 않고 URDF 이름으로 통일합니다.

```text
pinky_bringup (bringup) ──joint_states──▶ (remap) /wheel_joint_states_raw
        wheel_joint_state_relay: left_wheel_joint → l_wheel_joint, right_wheel_joint → r_wheel_joint
                                                ▶ /wheel_joint_states
        joint_state_publisher (source_list: wheel_joint_states)
                                                ▶ /joint_states (모든 URDF 관절)
        robot_state_publisher ▶ /tf
```

이름 매핑은 relay 노드의 `from_names`/`to_names` 파라미터로 바꿀 수 있습니다.

## 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `hardware` | `true` | `false`이면 rsp/jsp/relay만 실행 |
| `cam_tilt_deg` | 8 | 카메라가 수평에서 아래로 기운 각도(°). 실물이 약 8° 내려가 있어 시뮬레이션과 같은 값을 사용. 공식 `upload_robot.launch.py`는 0 |
| `screen_tilt_deg` | -25 | |
| `wheel_radius`, `wheel_separation` | 0.027, 0.0961 | 공식 값. URDF·Gazebo는 반지름 0.028 (보정 예정) |
| `lidar_serial_port` | `/dev/ttyAMA0` | |
| `use_sim_time` | `false` | |
| `bumper_enabled`, `bumper_params_file` | `true`, description yaml | |
| `camera_enabled` | `true` | `hardware:=false`이면 이 값과 무관하게 카메라 실행 안 함 |
| `camera_width`, `camera_height`, `camera_fps` | 640, 480, 15.0 | 확인된 해상도, 주기는 설정값이며 실물 측정 필요 |
| `camera_jpeg_quality` | 85 | 1~100 |
| `camera_info_url` | 빈 문자열 | `file:///절대경로.yaml`; 비어 있으면 미보정 |
| `camera_frame_id` | `front_camera_optical_frame` | Image·CameraInfo 헤더와 optical TF의 자식 프레임 |
| `imu_enabled`, `adc_enabled` | `false`, `false` | 실물 호환성 확인 후 개별 활성화 |
| `imu_interface`, `imu_rate` | `/dev/i2c-0`, 100.0 | 공식 BNO055 기본값 |
| `adc_interface`, `adc_rate` | `/dev/i2c-1`, 20.0 | 공식 ADC 기본값 |

## 검증 결과

| 항목 | 결과 |
|---|---|
| `hardware:=false`: rsp/jsp 실행, `base_footprint → bumper_link` = (0.046, 0, 0.018) | 통과 (PC) |
| `bumper_enabled:=false`, `cam_tilt_deg:=0`의 URDF가 공식 실물 description과 같음 | 통과 (PC) |
| relay: 가짜 `left_wheel_joint`=1.57 → `/joint_states`의 `l_wheel_joint`=1.57, `base_link → l_wheel` TF가 y축 90° 회전 | 통과 (PC) |
| relay 단위 테스트 | 통과 |
| 카메라 변환·보정 파일 검사·launch 활성화 조건 | 자동 테스트로 검사 |
| 합성 카메라의 실제 DDS 원본/JPEG/CameraInfo 일치와 장치 해제 | 자동 테스트로 검사 |
| 신규 카메라 ROS 노드·IMU·ADC의 **실물** 실행 | 2026-11-21 메시지 수신 확인. 센서 정확도·실측 보정은 별도 |
| `hardware:=true` | 2026-11-21 실제 로봇 launch·전체 센서/관절 토픽 수신 확인. 주행 명령 시험은 미수행 |

로봇에서 확인할 것:

1. `/scan`, `/odom`, `/joint_states`, `/tf`가 발행되는지
2. `ros2 run tf2_ros tf2_echo base_footprint bumper_link` 결과가 (0.046, 0, 0.018)인지
3. `/cmd_vel`로 주행이 되는지
4. 주행 중 `ros2 topic echo /joint_states`에서 `l_wheel_joint`/`r_wheel_joint` 값이 변하고, RViz에서 바퀴가 회전하는지
5. 먼저 `sensors.launch.py`로 영상 색상·방향·해상도·실제 FPS/지연 및 종료 후 카메라 재사용 확인
6. 호환성 확인 후 IMU·ADC를 하나씩 활성화해 정지/회전 방향·단위 및 초음파 실측 거리·IR 채널 순서 확인

## 자동 시각 동기화 상태

`imu_enabled:=true`일 때 기본 `imu_magnetic_enabled:=true`는 자체 `bno055_magnetic`을
사용합니다. 공식 submodule을 고치지 않고 NDOF로 구동해 기존 `/imu_raw`(SI),
`/imu_native`(deg/s), 추가 `/imu/mag`(tesla)·`/imu/status`(JSON 보정 상태)를 발행합니다.
공식 드라이버/릴레이와는 launch에서 상호 배제합니다. 기존 IMU-only 모드는
`imu_magnetic_enabled:=false`입니다. 다른 IMU 프로세스도 먼저 종료해야 합니다.
새 드라이버의 ARM64 실물 동작·축·자기 간섭은 현장 검증이 필요합니다.
사용 절차는 [위치 추정 안내](../pinky_soccer_world_model/LOCALIZATION.md)를 참고하세요.

실물 launch는 `/calibration/time_sync_status`를 발행하는 read-only chrony 상태 노드를 포함합니다. 최초 1회 PC/라즈베리파이 설정 후 OS가 자동 동기화하고 UI가 상태를 판정합니다. 설치·네트워크 기준과 적용 명령은 [ROS UI 안내](../pinky_soccer_calibration/ROS_UI.md#실물-자동-시간-동기화-최초-1회-설정)를 참고하세요. launch가 시스템 시계를 임의로 바꾸지는 않습니다.

## ADC와 배터리 토픽의 단일 읽기 경로

`adc_enabled:=true`이면 공식 ADC 드라이버만 I2C 버스 1의 주소 `0x08`을 읽습니다. 자체 `battery_state_relay`가 `/batt_state`의 같은 전압 표본으로 `/battery/voltage`와 `/battery/percent`를 5초마다 발행하고, 기존 `pinkylib.Battery` 기반 `battery_publisher`는 실행하지 않습니다. `adc_enabled:=false`이면 기존 배터리 퍼블리셔를 유지합니다.

두 드라이버는 같은 장치에 채널 선택 명령을 쓰고 대기한 뒤 읽으므로, 동시에 실행하면 상대 프로세스가 선택한 채널을 잘못 읽을 수 있습니다. 실물에서 전압 약 6.4V와 잔량 100%가 함께 수신돼 두 경로를 하나로 통합했습니다. 원본 ADC 드라이버는 수정하지 않습니다. 외부에서 같은 I2C 장치를 직접 읽는 별도 스크립트도 중복 실행하지 마세요.

잔량은 기존 pinkylib의 공식을 유지한 **전압 기반 추정값**입니다(6.8V→0%, 7.6V→100%, 범위 제한). 실측된 배터리 SoC나 방전 곡선이 아닙니다. NaN/비양수 전압 또는 2초 이상 오래된 ADC 표본은 발행하지 않습니다. 정확한 전압은 멀티미터와 교차 확인해야 하며, 실제 배터리 특성은 후속 보정 대상입니다.

## 원본 토픽은 보이지만 릴레이가 발행하지 않을 때

2026-11-21 실물에서 PC는 `/imu_native`(100Hz)·`/wheel_joint_states_raw`(30Hz)를 받지만 로봇 내부의 릴레이는 수신하지 못하는 문제가 확인됐습니다. `/imu_raw`, `/wheel_joint_states`, `/joint_states`는 토픽 이름/퍼블리셔가 보여도 메시지가 0개였고, joint_state_publisher는 robot_description을 기다리고 있었습니다.

당시 PC·로봇의 CycloneDDS XML은 무선 인터페이스 고정·`AllowMulticast=false`·상대 장치 peer만 지정한 상태였습니다. 로봇 내부 읽기 검사에서는 원본 IMU/바퀴 메시지가 0개였으나, 임시 설정에 **localhost peer 하나만 추가**하면 7초에 각각 702개/210개가 수신됐습니다. 드라이버 고장이나 단위 변환 오류가 아니라 같은 호스트의 DDS 참가자 발견 실패였습니다.

사용 중인 XML을 먼저 백업하고 PC와 로봇의 기존 `Discovery/Peers`에 다음 항목을 추가합니다. 기존 인터페이스·상대 peer·domain·멀티캐스트 설정은 유지합니다.

```xml
<Peers>
  <Peer Address="localhost"/>
  <!-- 기존 상대 장치 Peer 항목 유지 -->
</Peers>
```

설정은 프로세스 시작 시 읽으므로 로봇 launch를 정상 종료 후 같은 명령으로 다시 실행합니다. 이미 실험 기록 중이면 먼저 기록을 마무리하세요. 복구 후 UI와 같은 통신 조건의 검사 노드에서 `/imu_raw` 100Hz, `/wheel_joint_states` 30Hz, `/joint_states` 20Hz와 나머지 센서 수신을 확인했습니다. 이 조치는 해당 발견 문제를 해결한 것이며, 카메라 내부 보정이나 센서 실측 정확도를 보장하지 않습니다.

CycloneDDS의 [정적 peer 발견 설정](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html)을 참고하세요. launch는 사용자 DDS 파일을 자동으로 덮어쓰지 않습니다.
