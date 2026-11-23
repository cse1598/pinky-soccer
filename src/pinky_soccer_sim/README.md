# pinky_soccer_sim

공 기본 지름은 사용자 실측 **42.7mm**(반경/바닥 위 중심 높이 **0.02135m**)입니다.
Gazebo·MuJoCo의 시각/충돌 구와 인식·필터 기본 반경을 같은 값으로 맞춥니다.
실물 사진의 형광 주황색은 RGB `(1, 0.30, 0)`과 광택으로 근사합니다. 실제 형광의
자외선 반응·스펙트럼·딤플은 모델링하지 않으며 조명/노출에 따라 영상 색은 달라집니다.
질량45.9g·마찰·반발은 여전히 미실측 가정입니다. 지름 변경만으로 실측 물성으로 간주하지 않습니다.

공통 물성은 [`config/materials.yaml`](config/materials.yaml)에서 지정합니다. 기본은 골프공·EVA 계열
폼 요가매트·건조 고무바퀴이며, 두 엔진이 같은 접촉 계산 라이브러리를 사용합니다.
근거/가정·지원 범위·동일 조건 비교 시험은 [MATERIALS.md](MATERIALS.md)를 참고하세요.

`imu_enabled:=true`이면 `/imu/mag`·`/imu/status`도 제공합니다. 실제 자기 간섭이 없는
이상적 균일 자기장을 native IMU 축으로 회전한 시험 센서이며, 실측값이 아닙니다.
경기 launch의 AMCL·골대 색/자기 방향 기준은
[LOCALIZATION.md](../pinky_soccer_world_model/LOCALIZATION.md)를 참고하세요.

카메라가 활성화되면 실물과 동일한 `/camera/capture_frame`
(`pinky_soccer_msgs/srv/CaptureFrame`) 서비스를 제공합니다. 원본/JPEG 한 장 요청·저장 방법은
[CAMERA_CAPTURE.md](../pinky_soccer_bringup/CAMERA_CAPTURE.md)를 참고하세요.

범퍼를 장착한 Pinky Pro의 시뮬레이션(Gazebo, MuJoCo) launch 패키지입니다.

경기장은 `field:=rcj` 또는 `field:=hardboard`로 선택합니다. 기존 `world_name:=rcj_soccer.world`도 유지합니다. [FIELD.md](FIELD.md)와 [제2 하드보드 경기장](HARDBOARD_FIELD.md)에 치수·가정·실행 방법이 있습니다.

로봇 형상은 `pinky_soccer_description`, 실물 실행은 `pinky_soccer_bringup`이 담당합니다. 시뮬레이터 의존성은 이 패키지에만 있으므로 로봇(라즈베리파이)에서는 이 패키지를 빌드하거나 rosdep으로 설치하지 않아도 됩니다.

## 실물과 공통인 센서 인터페이스

두 시뮬레이터는 `sensor_adapter`를 통해 실물 `robot.launch.py`와 같은 공개 센서 토픽을 제공합니다. 내부 렌더링·브리지 토픽은 `/sim/` 아래에 둡니다. 공식 Submodule은 수정하지 않으며 런타임 모델과 자체 브리지만 확장합니다. 계약은 `config/interface.yaml`, 센서 기하/시나리오는 `config/sensors.yaml`, 실측 오차는 `config/measured_sensor_noise.yaml`에 있습니다.

기본 실행은 LiDAR·초음파·IMU에 실측 기반 오차를 적용합니다. 두 엔진의 공통 어댑터에서 적용하며 `sensor_noise_enabled:=false`로 이상적 관측과 비교합니다. 거리별 편차/흔들림·원본 출처·재생성 명령과 한계는 [SENSOR_NOISE.md](SENSOR_NOISE.md)를 참고하세요.

| 공개 토픽 | 형식·의미 | 활성화 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan`, 원본 stamp/frame 유지·실측 오차 적용 | 항상 |
| `/camera/image_raw` | `sensor_msgs/Image`, BGR8, 광학 프레임 `front_camera_optical_frame` | `camera_enabled:=true` (기본) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage`, JPEG | 카메라와 동일 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo`, 원본/JPEG와 동일한 stamp·frame | 카메라와 동일 |
| `/imu_raw` | `sensor_msgs/Imu`, quaternion, rad/s, m/s² (중력 포함) | `imu_enabled:=true` |
| `/imu_native` | `sensor_msgs/Imu`, 실물 공식 드라이버 진단용과 같은 deg/s | IMU와 동일; 판단에는 `/imu_raw` 사용 |
| `/us_sensor/range` | `sensor_msgs/Range`, m, `ultrasonic_link`, 시야각 0.26 rad | `adc_enabled:=true` |
| `/ir_sensor/range` | `std_msgs/UInt16MultiArray`, ADC 3채널 | ADC 활성화 + 거리/ADC 실측 보정표 필요 |
| `/batt_state` | `sensor_msgs/BatteryState`, 공식 ADC의 필드·미지원값 NaN 규약 | ADC와 동일 |
| `/battery/voltage`, `/battery/percent` | `std_msgs/Float32`, V / 0~100 % | 항상 |

카메라 3개 토픽과 `/imu_raw`의 QoS는 실물과 같은 SensorData(Best Effort, Volatile, depth 5)입니다. 초음파·IR ADC·배터리는 실물처럼 Reliable, depth 10입니다. 카메라 기본값은 640×480, 최대 15 Hz, JPEG 품질 85이며 `front_camera_link`에서 실물과 동일한 광학 TF를 발행합니다. 원본의 RGB/BGR와 행 stride를 처리합니다. MuJoCo 소프트웨어 렌더링은 부하에 따라 FPS를 낮출 수 있습니다.

센서 플래그 기본값은 실물과 동일하게 카메라 true, IMU/ADC false입니다. 다음처럼 모든 가상 센서를 켭니다.

```bash
ros2 launch pinky_soccer_sim gazebo_soccer.launch.py imu_enabled:=true adc_enabled:=true
# 또는
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py imu_enabled:=true adc_enabled:=true
```

Gazebo는 네이티브 IMU와 추가한 GPU ray 센서, MuJoCo는 센서 링크 위치에 설치한 네이티브 accelerometer/gyro/framequat와 ray casting을 사용합니다. 초음파는 수평 부채꼴 5개 ray 중 최솟값으로 근사합니다. 감지 범위보다 가까우면 -Inf, 장애물이 없으면 +Inf를 발행하며 음향 반사·재질·다중 경로는 모델링하지 않습니다. IR 3개의 거리 관측은 `/sim/ir_l_link/distance`, `/sim/ir_mid_link/distance`, `/sim/ir_r_link/distance` (`Range`)로 확인합니다.

**정지 실측의 통계적 오차를 반영하며, 모든 동작/재질에서 실물과 동일한 성능을 검증한 것은 아닙니다.** 다음은 명시적인 제한입니다.

- 카메라 가상 FOV는 공식 모델의 1.1519 rad 가정입니다. 가상 CameraInfo는 렌더링 모델 값이며, 실물 CameraInfo에는 실제 내부 보정 YAML을 넣어야 합니다. 기본 모델은 왜곡이 없는 카메라입니다. 실제 왜곡·노출·모션 블러까지 복제하지 않습니다. 실물 영상의 180° 회전과 실제 장착 방향은 후속 실물 검증 대상입니다.
- 센서 위치·방향은 공식 URDF 기준입니다. IR은 공식 링크 +X 방향을 사용하지만 실제 광축과 ADC 채널 L/M/R 순서는 미확인입니다. 거리 검색 범위 0.005~0.8 m도 가상 실험 설정이며 실물 사양이 아닙니다.
- 사용자가 IR 실측 자료가 없음을 확인했습니다. `sensors.yaml`의 `ir.distances`(증가하는 m)와 `ir.adc_values`(0~4095)에 측정값을 넣고 `sensor_config:=/절대/경로.yaml`로 실행하면 ADC 토픽이 켜집니다. 범위 안은 선형 보간, 밖은 표의 끝값으로 포화합니다. 보정표가 없으면 경고하며 ADC 토픽을 발행하지 않습니다.
- IMU는 물리 관측에 실측 정지 잔차의 편차·공분산을 추가합니다. BNO055 융합기의 동적 지연·장기 yaw drift는 미식별입니다. 추가 오차의 covariance를 발행하며 기본 중력은 엔진 설정을 따릅니다.
- 배터리는 설정된 전압·백분율을 유지하는 이상적인 전원입니다. 기본 8.4 V / 100 %는 가상 시나리오이며 실측값이 아닙니다. 소모·충전 모델은 포함하지 않습니다. 타이머는 `/clock`을 따르므로 일시정지 중에 진행되지 않습니다.
- Gazebo GPU ray는 렌더링 형상, MuJoCo ray는 충돌 형상을 봅니다. 시야 가림·메시 차이에 따른 거리는 실험으로 비교해야 합니다.

실물·가상 센서 계층은 카메라 활성화 시 `/camera/capture_frame` 단발 사진 서비스를 동일하게 제공합니다. `SetCameraInfo` 서비스는 제공하지 않으며 실물은 시작 시 YAML을 읽습니다. 애플리케이션용 액션은 없으며 시뮬레이터 관리용 서비스와 ROS 노드별 파라미터 서비스는 환경마다 다를 수 있습니다. 향후 경기 제어 서비스/액션은 공통 제어 계층에서 정의해야 합니다. `/clock`, `/ground_truth`, `/sim/*`는 가상환경 전용이며 판단 프로그램의 필수 입력으로 사용하지 않습니다. 기존 `/cmd_vel`, `/odom`, `/scan`, `/joint_states`, TF는 유지합니다.

### 동일한 수신 프로그램으로 검사하기

시뮬레이터 실행 후 별도 PC 터미널에서 실행합니다. 실제 로봇도 같은 PC 명령을 사용합니다(로봇과 ROS domain·네트워크 설정 일치 필요).

```bash
source install/setup.bash
ros2 run pinky_soccer_sim check_interface --imu --adc --duration 15 \
  --output /tmp/pinky-interface.yaml
```

실물에서 IMU/ADC를 끈 경우 `--imu --adc`를 생략합니다. IR 보정표를 설정한 뒤에는 `--ir`도 추가합니다. 카메라를 끈 경우 `--no-camera`를 사용합니다. 이 도구는 명령을 보내지 않고 토픽 수신·타입·카메라 QoS/인코딩·주요 frame을 검사하고 그래프·서비스 목록·수신 수를 YAML로 저장합니다. 통신 계약 검사이며 보정 정확도나 센서 단위의 물리적 정확성을 증명하지는 않습니다.

### 센서 확장 검증 (2026-11-20)

- Gazebo/MuJoCo 실제 launch에서 카메라 원본/JPEG/CameraInfo·IMU·초음파·IR 거리·배터리 수신 확인. 두 카메라는 640×480 BGR8, 동일 optical frame으로 수신.
- 동일 `check_interface --imu --adc` 명령으로 두 환경 모두 PASS. 실물 래퍼·시뮬레이션 기능 테스트 총 33개 및 lint 통과 (`colcon test-result`: 각각 39/35 검사 결과, 실패 0).
- MuJoCo 정지 중 가속도 크기 약 9.81 m/s², gyro 약 0 및 이동 장애물에 대한 ray 거리 변화 검증. 합성 입력의 실제 DDS 검사로 색상·원본/JPEG/CameraInfo 헤더·IMU 단위 유지·IR 보간 검증.
- 이 PC의 Gazebo는 소프트웨어 EGL과 하드웨어 장치 선택이 충돌했습니다. 설치된 NVIDIA 드라이버가 있는 이 환경에서는 `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`과 소프트웨어 강제 옵션 해제 후 실행 확인했습니다. 이 경로를 다른 PC에 그대로 적용하지 마세요.
- 실물 신규 ROS 노드·IR 채널·실측 기반 카메라/센서 보정은 아직 검증하지 않았습니다. 아래 기존 주행/공 실험 표의 수신 주기는 센서 확장 전 측정 기록입니다.

## 시뮬레이터 오도메트리 보정 (2026-11-24)

실물처럼 시뮬레이터도 명령 변환·엔코더 odom 상수를 보정합니다. 피드백 제어가 아니라 상수 보정입니다.
보정 UI와 같은 실험(0.08 m/s·0.30 rad/s로 odom 1 m / 360°까지, 60초 제한)을 전진·후진·좌회전·우회전
각 3회 돌리고, 줄자 대신 시뮬레이터 정답값(범퍼 앞면 중앙 시작~끝 직선거리, 실제 누적 회전각)으로 잽니다.
계산과 저장은 `pinky_soccer_calibration`의 Session·`distance_rotation_fit`·export를 그대로 써서
실물과 같은 형식의 `profile.yaml`(`environment: rehearsal`, `robot_id: <simulator>-sim`,
`measurement_source: simulator_ground_truth`)이 나옵니다.

| 시뮬레이터 | 보정 전 odom 오차 (직진 / 회전) | 유효 반경 | 유효 간격 | 보정 후 (직진 / 회전) |
|---|---|---|---|---|
| MuJoCo | +1.8 % / +2.7 % | 27.50 mm (모델 28.0) | 98.25 mm (모델 97.1) | 0.0 % / −0.2~−0.3 % |
| Gazebo | +0.1 % / +1.6 % | 27.97 mm (DiffDrive 28.0) | 98.07 mm (DiffDrive 96.1) | 0.0 % / −0.5 % |

(+는 odom이 실제보다 크게 셈. 하드보드 경기장 매트 재질, 센서 잡음 기본 ON에서 측정.)

적용 범위: **구동 상수만** 바꿉니다(사용자 결정 (a)). 바퀴 충돌 형상(물리)은 그대로입니다.
- MuJoCo: 래퍼 `--wheel-radius/--wheel-separation`이 공식 `Physics.radius/track`(cmd_vel 변환·odom)만 덮어씀
- Gazebo: DiffDrive 플러그인 `wheel_radius/wheel_separation`만 변경
- 실물 드라이버의 `wheel_radius/wheel_separation`과 같은 구조라, 같은 프로필 형식으로 세 환경을 다룹니다.

기본 적용: `mujoco_soccer.launch.py`·`gazebo_soccer.launch.py`(및 이를 포함하는 경기 launch)는
`config/calibration/<simulator>_profile.yaml`이 있으면 자동으로 씁니다. 다른 프로필은
`calibration_profile:=<path>`로 지정하고, 보정 없이 공식 상수로 돌리려면 `calibration_profile:=none`.

다시 보정하기 (물리·재질·모델을 바꾼 뒤):

```bash
# 1) 보정을 적용하지 않은 상태로 띄우기
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py field:=hardboard viewer:=false camera:=false \
    spawn_ball:=false calibration_profile:=none
# 2) 다른 터미널 (같은 ROS_DOMAIN_ID)
ros2 run pinky_soccer_sim calibrate_odometry.py --simulator mujoco --install \
    --install-dir ~/pinky-soccer/src/pinky_soccer_sim/config/calibration
# 3) 재빌드 후 다시 띄워 검증
ros2 run pinky_soccer_sim calibrate_odometry.py --simulator mujoco --verify --repeats 2
```

Gazebo는 `gazebo_soccer.launch.py ... gui:=false`, `--simulator gazebo`. 실행기는 위치 맞춤용으로 먼저
0.5 m 후진(기록 안 함)한 뒤 실험해 1 m 주행이 경기장 안에 있도록 합니다. 원본 실험 폴더는
`calibration_results/simulator/`(git 제외)에 남습니다.

## Gazebo

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pinky_soccer_sim
source install/setup.bash

ros2 launch pinky_soccer_sim gazebo_soccer.launch.py
```

다른 터미널에서 주행:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world_name` | `empty.world` | `pinky_gz_sim/worlds` 안의 파일 (`pinky_factory.world`, `pinky_map.world`) |
| `world` | — | world 절대 경로 (`world_name`보다 우선) |
| `gui` | `true` | Gazebo GUI 실행 |
| `x`, `y`, `yaw` | 0 | 로봇 spawn 위치 |
| `cam_tilt_deg` / `screen_tilt_deg` | 8 / -25 | xacro, MuJoCo와 같은 값 (공식 Gazebo launch는 cam 0) |
| `bumper_enabled` | `true` | 범퍼 on/off |
| `bumper_params_file` | description 패키지의 yaml | 범퍼 치수 |
| `spawn_ball` | `true` | 테스트 공 생성 |
| `ball_x`, `ball_y` | 0.3, 0 | 공 위치 |
| `ball_radius`, `ball_mass` | 0.02135, 0.0459 | 주황색 골프공(실측 지름 42.7 mm, 질량 45.9 g·속이 찬 구 관성은 미실측 가정). 공 중심(지면 약 21 mm)이 범퍼 높이 구간(지면 9~51 mm) 안에 있어서 범퍼를 타고 넘지 않습니다 |

### 공식 launch와의 관계

공식 `pinky_gz_sim/launch/launch_sim.launch.xml`은 `pinky_description/launch/upload_robot.launch.py`를 include하는데, 이 파일이 `robot.urdf.xacro` 경로를 하드코딩하고 있습니다. 그래서 공식 launch를 include하면 범퍼 없는 로봇이 생성됩니다. `gazebo_soccer.launch.py`는 같은 구성을 직접 재구성하고, 공식 파일은 경로로만 재사용합니다.

| 구성 | 처리 |
|---|---|
| world | 공식 `pinky_gz_sim/worlds` 재사용 (수정 없음) |
| bridge 설정 | 자체 `config/gazebo_bridge.yaml`; 공식 공통 토픽 유지, 내부 영상·IMU·ray 브리지 추가 |
| Gazebo 플러그인 (DiffDrive, LiDAR, 카메라, IMU, 램프) | 공식 매크로 생성 후 런타임 URDF에서 카메라 설정·선택 센서 확장 |
| robot_state_publisher / joint_state_publisher | `upload_robot.launch.py`와 같은 설정으로 재구성 |
| `GZ_SIM_RESOURCE_PATH` | 공식 경로 + `pinky_soccer_description` share 추가 (CAD 메시용) |
| 영상 | `parameter_bridge` → `/sim/camera/*` → 공통 센서 어댑터 |
| 테스트 공 | `ros_gz_sim create -string`으로 별도 생성 (world 수정 없음) |

공식 Submodule을 업데이트한 뒤에는 위 공식 파일들이 바뀌었는지 확인하고 이 launch에 반영합니다.

```bash
git -C src/pinky_pro diff <이전 커밋> -- \
  pinky_gz_sim/launch pinky_gz_sim/params \
  pinky_description/launch pinky_description/urdf
```

### 검증 결과 (헤드리스, `gui:=false`)

| 항목 | 결과 |
|---|---|
| Gazebo에 `pinky` 1대와 `soccer_ball`만 생성 | 통과 |
| `/clock`, `/odom`(약 47 Hz), `/scan`(약 10 Hz), `/camera/image_raw`(약 20 Hz), `/joint_states` | 통과 |
| `base_footprint → bumper_link` TF = (0.046, 0, 0.018) | 통과 |
| `/cmd_vel` 주행 → `/odom` 갱신 | 통과 |
| 범퍼로 공 밀기: 공이 범퍼를 통과하거나 타고 넘지 않음 | 통과 |
| ±1.0 m/s 급가속·급제동(DiffDrive 가속 제한 2 m/s²) 시 pitch | 범퍼 없음 0.01°, 범퍼 40 g 0.00°, **범퍼 400 g 9.5°(제동 시 범퍼가 바닥에 닿음)** |
| Gazebo GUI 화면 | 사람이 직접 확인해야 함 |

### 알려진 제한

- `namespace`는 아직 지원하지 않습니다. 공식 bridge 설정도 namespace를 지원하지 않으므로, 1대1 시뮬레이션에서 로봇 2대를 띄울 때 따로 구현해야 합니다.
- Gazebo DiffDrive에는 `cmd_vel` 타임아웃이 없습니다. 마지막 명령이 계속 유지되므로 정지하려면 0 명령을 확실히 보내야 합니다.
- `/joint_states`가 약 1 kHz로 발행됩니다. 공식 구성(Gazebo JointStatePublisher + joint_state_publisher)과 같은 동작입니다.
- Gazebo는 fixed joint를 합칩니다. 범퍼 충돌 형상은 `base_link`로 합쳐져 그대로 동작하지만, 범퍼 전용 contact 센서가 필요하면 `disableFixedJointLumping`이 필요합니다.
- 범퍼와 공의 마찰 계수는 Gazebo 기본값을 사용합니다.


## MuJoCo

### 의존성 설치 (최초 1회, 공식 pinky_mujoco README와 같음)

```bash
sudo apt install python3-pip
pip install --break-system-packages "mujoco==3.6.0" "trimesh>=4,<5" "pycollada>=0.9,<1"
```

### 실행

```bash
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py viewer:=false camera:=false
```

### 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `viewer` / `camera` / `use_gpu` | `true` / `true` / `false` | 공식 MuJoCo launch와 같음 |
| `world_name` / `world` | `empty.world` / — | 공식과 같은 규칙 (`pinky_gz_sim/worlds` 또는 `pinky_mujoco/worlds/*.yaml`) |
| `cam_tilt_deg` / `screen_tilt_deg` | 8 / -25 | |
| `bumper_enabled` / `bumper_params_file` | `true` / description 기본 yaml | |
| `spawn_ball`, `ball_x`, `ball_y`, `ball_radius`, `ball_mass` | `true`, 0.3, 0, 0.02135, 0.0459 | Gazebo와 같은 골프공 |
| `model_dir` | `~/.cache/pinky_soccer_mujoco` | 생성된 URDF와 scene.xml. 공식 캐시(`~/.cache/pinky_mujoco`)와 분리 |

### 공식 MuJoCo와의 관계

공식 `pinky_mujoco/launch/launch_sim.launch.py`는 항상 `pinky_description/urdf/robot.urdf.xacro`를 확장합니다. `mujoco_soccer.launch.py`는 같은 구성으로 다음만 바꿉니다.

| 구성 | 처리 |
|---|---|
| URDF 생성 | 공식 `pinky_mujoco.description.expand_urdf()`로 `soccer_robot.urdf.xacro`를 확장 (`is_sim:=false`) |
| 브리지 | 공식 `pinky_mujoco.bridge`를 `mujoco_soccer_bridge`로 감싸 실행. `--urdf-file`, `--world`, `--viewer` 등 공식 인자를 그대로 전달 |
| 물리, odom, LiDAR | 공식 물리 유지. 원본 LiDAR는 `/sim/lidar/scan`으로 remap하여 공통 오차 적용 후 `/scan` 발행 |
| 카메라·센서 | 자체 설정의 renderer와 네이티브 IMU·ray 관측 → 공통 센서 어댑터 |
| 테스트 공 | `mujoco_soccer_bridge`가 생성된 `scene.xml`에 동적 공을 추가 |

공식 `world.py`는 world 물체를 모두 정적 geom으로 만들기 때문에, world 파일에 넣은 공은 밀리지 않습니다. 그래서 `mujoco_soccer_bridge`가 공식 `bridge.build_model`을 감싸서, 생성된 scene의 `<worldbody>` **맨 끝**에 freejoint가 있는 공을 추가합니다.

- 맨 끝에 추가하는 이유: 공식 `Physics.pose()`가 `qpos[0:7]`을 로봇으로 가정합니다.
- 충돌 마스크 `contype=3, conaffinity=3`: 바닥/world(1/2)와 로봇(2/1) 모두와 충돌합니다.
- `group=0`: viewer에 보이고 LiDAR에도 잡힙니다.

이 래퍼는 공식 내부 구현(`bridge.build_model`, scene 구조, 충돌 마스크)에 의존합니다. Submodule을 업데이트한 뒤에는 `colcon test --packages-select pinky_soccer_sim`으로 확인합니다.

### 검증 결과

| 항목 | 결과 |
|---|---|
| 공 추가 단위 테스트 (위치, 순서, 충돌 마스크, 관성, 잘못된 값 거부, 인자 분리) | 통과 |
| 공식 `expand_urdf()`로 범퍼 URDF 생성 (`bumper_link` 포함, Gazebo 플러그인 제외) | 통과 |
| 물리 테스트 (`test_physics_robot_pushes_ball_and_stays_level`) | 통과 |
| launch 헤드리스 실행(`viewer:=false`): `/clock` 100 Hz, `/odom` 50 Hz, `/scan` 10 Hz, `/joint_states` 49 Hz, `/camera/image_raw` 약 10 Hz(소프트웨어 렌더링) | 통과 |
| `base_footprint → bumper_link` = (0.046, 0, 0.018), `/joint_states` 이름 `l_wheel_joint` 등 | 통과 |
| `/cmd_vel` 주행 → `/odom` 갱신 | 통과 |
| viewer 화면 | 사람이 직접 확인해야 함 |

물리 비교 (0.25 m/s로 3 s 전진 → 정지 → 후진 → 정지, 공은 로봇 앞 0.3 m):

| 조건 | 전진 후 로봇 x / 공 x | 공 최대 높이 | 최대 pitch |
|---|---|---|---|
| 범퍼 없음 | 0.338 / 0.395 m (**공 앞에서 멈춤**) | 0.020 m | 1.55° |
| 범퍼 40 g | 0.725 / 0.814 m | 0.020 m | 0.03° |
| 범퍼 400 g | 0.724 / 0.927 m | 0.020 m | 1.68° |

- 범퍼가 없으면 로봇이 공 앞에서 멈춥니다. 공이 몸체 앞 아래쪽에 끼는 것으로 보입니다.
- MuJoCo는 공식 설정상 최고 속도가 0.25 m/s(`mujoco.yaml` `max_linear`)라서 400 g 범퍼의 기울기가 Gazebo(1.0 m/s, 9.5°)보다 작게 나옵니다.

### 알려진 제한

- LiDAR는 지면에서 약 12.5 cm 높이를 수평으로 스캔하므로, 낮은 공(현재 지름 4 cm)은 `/scan`에 잡히지 않습니다. 실물도 마찬가지이므로 공은 카메라로 찾아야 합니다.
- 공의 위치는 ROS 토픽으로 발행하지 않습니다(필요하면 추가).
- 소프트웨어 렌더링에서는 카메라가 약 10~13 Hz로 제한됩니다(공식 동작). GPU가 있으면 `use_gpu:=true`를 사용합니다.
