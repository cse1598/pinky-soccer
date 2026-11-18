# pinky_soccer_bringup

범퍼를 장착한 Pinky Pro를 **실물 로봇**에서 실행하는 패키지입니다. 공식 `pinky_bringup bringup_robot.launch.xml` 대신 사용합니다.

로봇 형상은 `pinky_soccer_description`, 시뮬레이션은 `pinky_soccer_sim`이 담당합니다. 이 패키지에는 시뮬레이터 의존성이 없습니다.

## 실행

로봇(라즈베리파이)에서:

```bash
ros2 launch pinky_soccer_bringup robot.launch.py
```

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
  src/pinky_pro/pinky_bringup src/pinky_pro/pinky_description

colcon build --symlink-install --packages-up-to pinky_soccer_bringup
```

`rosdep install --from-paths src`처럼 `src` 전체를 지정하면 공식 `pinky_gz_sim`과 `pinky_mujoco`의 의존성(Gazebo 등)까지 설치됩니다. 위처럼 필요한 경로만 지정합니다.

`sllidar_ros2`, `dynamixel_sdk`, `pinkylib`처럼 공식 bringup이 쓰는 드라이버는 공식 Pinky 설정대로 로봇에 설치되어 있어야 합니다.

## 공식 launch와의 관계

| 구성 | 처리 |
|---|---|
| `upload_robot.launch.py` (rsp + jsp) | 같은 설정으로 재구성하고, xacro만 `soccer_robot.urdf.xacro`로 교체 |
| `sllidar_ros2` `sllidar_c1_launch.py` | 공식과 같은 인자로 include (`/dev/ttyAMA0`, `rplidar_link`, DenseBoost) |
| `pinky_bringup` `bringup` | 공식 `pinky_params.yaml` 사용. `joint_states` 출력만 `wheel_joint_states_raw`로 remap |
| `pinky_bringup` `battery_publisher` | 그대로 |
| `wheel_joint_state_relay` | 추가 노드 (아래 참고) |

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

## 검증 결과

| 항목 | 결과 |
|---|---|
| `hardware:=false`: rsp/jsp 실행, `base_footprint → bumper_link` = (0.046, 0, 0.018) | 통과 (PC) |
| `bumper_enabled:=false`, `cam_tilt_deg:=0`의 URDF가 공식 실물 description과 같음 | 통과 (PC) |
| relay: 가짜 `left_wheel_joint`=1.57 → `/joint_states`의 `l_wheel_joint`=1.57, `base_link → l_wheel` TF가 y축 90° 회전 | 통과 (PC) |
| relay 단위 테스트 | 통과 |
| `hardware:=true` | **미검증**. PC에 `sllidar_ros2`가 없어 실행할 수 없음. 로봇에서 확인 필요 |

로봇에서 확인할 것:

1. `/scan`, `/odom`, `/joint_states`, `/tf`가 발행되는지
2. `ros2 run tf2_ros tf2_echo base_footprint bumper_link` 결과가 (0.046, 0, 0.018)인지
3. `/cmd_vel`로 주행이 되는지
4. 주행 중 `ros2 topic echo /joint_states`에서 `l_wheel_joint`/`r_wheel_joint` 값이 변하고, RViz에서 바퀴가 회전하는지
