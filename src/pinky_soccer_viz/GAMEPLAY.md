# 실물·Gazebo·MuJoCo 경기 실행

세 환경 모두 `game.launch.py` 실행 → RViz 경기장 모니터 → 시작 신호 → 자동 플레이 순서입니다.

기본 `headless:=false`이면 **RViz와 카메라 창이 함께 열립니다.** 카메라 창은
왼쪽 **원본**, 가운데 **경기 알고리즘의 공 원/중심 표시**, 오른쪽 **주황색 선택 마스크**를
동일 프레임 시각으로 나란히 표시합니다. 노란 골대/책상이 마스크에서 흰색인지와
가운데 원이 실제 공에 겹치는지 확인하세요. 아직 처리되지 않은 프레임은 WAITING,
0.5초 넘게 끊긴 영상은 STALE로 표시합니다. Q/ESC나 닫기 버튼은 카메라 창만 닫고
경기는 계속하므로 경기 정지는 기존 시작/정지 신호로 합니다. `headless:=true`는 두 GUI를 끕니다.

창을 닫았다가 다시 열려면 같은 ROS domain/DDS 터미널에서
`ros2 run pinky_soccer_viz game_camera_view`를 실행합니다. 모니터는 읽기 전용이며
검출 알고리즘을 따로 실행하거나 주행 명령을 발행하지 않습니다.
원본 패널은 검출 노드가 이미 받은 입력의 `/ball_detection/source_image` 복사본을
사용합니다. PC 모니터 때문에 로봇 원본 토픽의 원격 구독을 하나 더 만들지 않습니다.
인식·월드 모델·BT·제어를 따로 실행할 필요가 없습니다. 시작 전에는 정지 대기합니다.
한 번에 한 환경만 실행하고 기존 판단/제어/시뮬레이터는 해당 터미널에서 종료합니다.

| mode | 센서/물리 실행 | PC 경기 프로그램 | 시간 | RViz |
|---|---|---|---|---|
| real | 로봇의 robot.launch.py 유지 | game.launch.py mode:=real | 시스템 시간 | 자동 |
| gazebo | 경기 프로그램에서 Gazebo 자동 실행 | game.launch.py mode:=gazebo | /clock | 자동 |
| mujoco | 경기 프로그램에서 MuJoCo 자동 실행 | game.launch.py mode:=mujoco | /clock | 자동 |

행동을 하나씩 실행·평가하려면 `verify:=true`를 붙입니다(경기 트리 대신 검증 창에서 고른 행동만
실행). 자세한 내용은 [VERIFY.md](VERIFY.md).

경기장 기본값은 `field:=hardboard`, 원래 경기장은 `field:=rcj`입니다.
이동 제어는 기본 NMPC(`controller:=mpc`, 이전 비례 제어는 `controller:=reactive`)입니다.
속도 상한 기본(`auto`): 실물 전진0.08m/s·회전0.5rad/s, 시뮬레이션은 바퀴 한계(모터 사양 90 %,
약0.27m/s)와 회전2rad/s(더 빠르면 위치 추정이 깨짐). `max_lin`, `max_ang`로 변경합니다.
기본 우리 골대는 노랑, +X 파란 골대 방향으로 공격합니다.

위치 추정 기본은 `localization:=lidar`입니다. 경기장 YAML로 만든 LiDAR 지도와
AMCL을 사용하며, 골대 색/지자기 방향 기준으로180° 대칭 후보를 확인합니다.
기록/센서 상태/실물 보정은 [LOCALIZATION.md](../pinky_soccer_world_model/LOCALIZATION.md).
기존 바퀴 누적 방식만 시험하려면 `localization:=odom_only`를 지정합니다.

## 처음 한 번 PC 빌드

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_bringup pinky_soccer_sim pinky_soccer_world_model pinky_soccer_viz
```

이미 빌드된 나머지 의존 패키지가 설치되어 있다는 전제입니다.

## 실물

로봇에서 센서/모터 드라이버를 한 번 실행합니다. 이미 실행 중이면 유지합니다.
공식·wrapper 워크스페이스를 source한 로봇 터미널에서:

```bash
ros2 launch pinky_soccer_bringup robot.launch.py imu_enabled:=true adc_enabled:=true
```

PC의 **경기 실행/시작 신호 터미널 둘 다** 다음 환경을 사용합니다.
domain 37과 XML의 로봇 IP/네트워크 인터페이스는 실제 로봇 설정과 맞아야 합니다.

```bash
source ~/pinky-soccer/install/setup.bash
export ROS_DOMAIN_ID=37
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

로봇을 경기장 중앙에서 파란 골대를 향하게 놓고 PC 터미널 A에서:

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=real field:=hardboard allow_uncalibrated:=true
```

RViz와 공 인식 영상 창이 열립니다. AMCL이 시작 위치 주변에서 LiDAR 지도를 정합합니다.
미보정 옵션은 실제 측정값 대신 HFOV66°·중앙 주점·왜곡0·실행 URDF 장착 자세를
가정한 시험 모드이며 영상에 APPROXIMATE를 표시합니다. 정확한 보정 후에는
`allow_uncalibrated`를 빼고 실행합니다. 자세한 옵션은 [REAL_GAME.md](REAL_GAME.md).

## Gazebo

로봇과 분리된 domain118을 사용합니다. PC의 **두 터미널 모두**:

```bash
source ~/pinky-soccer/install/setup.bash
export ROS_DOMAIN_ID=118
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
```

`unset`은 이 터미널에서만 로봇 전용 DDS XML 사용을 해제합니다.
실물의 XML/설정 파일은 변경하지 않습니다.

터미널 A:

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=gazebo field:=hardboard
```

Gazebo·RViz·인식·판단·제어가 함께 실행됩니다.

## MuJoCo

Gazebo와 같은 PC 환경(domain118·CYCLONEDDS_URI 해제)을 두 터미널에 사용합니다.
Gazebo를 먼저 종료한 다음 터미널 A:

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco field:=hardboard
```

MuJoCo 뷰어·RViz·인식·판단·제어가 함께 실행됩니다.
시뮬레이션 두 종류 모두 IMU/ADC 어댑터는 기본 활성화합니다.
IR raw ADC는 실측 보정표가 없으면 생성하지 않는 기존 규칙을 유지합니다.

## 공통 시작·정지

터미널 B에서 **환경을 해당 경기와 동일하게 설정한 뒤**:

```bash
# 시작: 로봇/시뮬레이션이 자동으로 탐색·판단·주행
ros2 topic pub --once /game/playing std_msgs/msg/Bool '{data: true}'

# 정지
ros2 topic pub --once /game/playing std_msgs/msg/Bool '{data: false}'
```

종료는 정지 신호를 보낸 뒤 터미널 A의 Ctrl+C입니다.
모드 전환 시 기존 프로그램을 종료합니다. 새 launch는 매번 정지 대기부터 시작합니다.

### 시작 명령이 구독자를 기다릴 때

`Waiting for at least 1 matching subscription(s)...`는 `/game/playing` 구독자를
발견하지 못했다는 뜻입니다. Gazebo 창만 실행한 경우 전체 경기 launch가 필요합니다.
전체 경기 launch가 실행 중이라면 신호 터미널의 ROS domain/DDS 설정을 맞춥니다.
2026-11-21 실제 진단에서는 domain118의 `world_model_node` 구독자와 월드 모델 수신이
정상이었으므로, 다음처럼 신호 명령에 환경을 직접 지정할 수 있습니다.

```bash
# 시뮬레이션 domain118에 시작 신호 (실물용 DDS XML은 이 명령에서만 해제)
env -u CYCLONEDDS_URI ROS_DOMAIN_ID=118 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 topic pub --once /game/playing std_msgs/msg/Bool '{data: true}'

# 같은 환경으로 정지
env -u CYCLONEDDS_URI ROS_DOMAIN_ID=118 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 topic pub --once /game/playing std_msgs/msg/Bool '{data: false}'
```

118은 연결을 보장하는 특별한 값이 아니라 두 프로세스가 같게 사용해야 하는 domain 번호입니다.

## RViz 경기장 모니터

`soccer.rviz`를 자동 로드하고 Fixed Frame은 `map`입니다.
선택한 YAML의 경기장/골대, 추정 로봇/공/상대 위치, 공 불확실성·예측 궤적,
탐색 히트맵 및 행동 목표를 표시합니다. 자기 위치는 실물의 경우 odom 기반입니다.
히트맵은 `/world_model/search_grid_markers` 색상 마커로 표시해 이 PC에서 확인된
RViz Map의 indexed texture 셰이더 오류를 피합니다. 원본 OccupancyGrid도 계속 제공합니다.
경기 상태 모니터의 자기 위치는 기본 LiDAR AMCL 보정 TF를 따릅니다.
골대 색/지자기 방향 기준 상태는 `/localization/status`에서 확인합니다.
모든 GUI를 끄려면 `headless:=true`를 추가합니다.

실물에서 거리 정확도·게임 성공은 보정 및 현장 검증이 필요합니다.
시뮬레이션에서는 근거리 사각·odom 드리프트로 경기 성공이 제한될 수 있습니다.
