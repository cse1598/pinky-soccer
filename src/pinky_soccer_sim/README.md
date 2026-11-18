# pinky_soccer_sim

범퍼를 장착한 Pinky Pro의 시뮬레이션(Gazebo, MuJoCo) launch 패키지입니다.

로봇 형상은 `pinky_soccer_description`, 실물 실행은 `pinky_soccer_bringup`이 담당합니다. 시뮬레이터 의존성은 이 패키지에만 있으므로 로봇(라즈베리파이)에서는 이 패키지를 빌드하거나 rosdep으로 설치하지 않아도 됩니다.

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
| `ball_radius`, `ball_mass` | 0.02, 0.03 | **임시값**. 공 중심이 범퍼 높이 구간(현재 지면 0.008~0.028 m) 안에 있어야 범퍼를 타고 넘지 않습니다 |

### 공식 launch와의 관계

공식 `pinky_gz_sim/launch/launch_sim.launch.xml`은 `pinky_description/launch/upload_robot.launch.py`를 include하는데, 이 파일이 `robot.urdf.xacro` 경로를 하드코딩하고 있습니다. 그래서 공식 launch를 include하면 범퍼 없는 로봇이 생성됩니다. `gazebo_soccer.launch.py`는 같은 구성을 직접 재구성하고, 공식 파일은 경로로만 재사용합니다.

| 구성 | 처리 |
|---|---|
| world | 공식 `pinky_gz_sim/worlds` 재사용 (수정 없음) |
| bridge 설정 | 공식 `pinky_gz_sim/params/pinky_bridge.yaml` 재사용 |
| Gazebo 플러그인 (DiffDrive, LiDAR, 카메라, IMU, 램프) | 공식 xacro 매크로에서 그대로 생성 |
| robot_state_publisher / joint_state_publisher | `upload_robot.launch.py`와 같은 설정으로 재구성 |
| `GZ_SIM_RESOURCE_PATH` | 공식 경로 + `pinky_soccer_description` share 추가 (CAD 메시용) |
| image_bridge | `camera/image_raw` 하나만 (공식은 같은 이름의 노드 두 개를 실행) |
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

다음 단계에서 추가합니다.
