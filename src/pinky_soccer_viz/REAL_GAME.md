# 실물 Pinky Pro 시험 경기

현재 기본 rectangle 경기는 [경기 준비·LED 절차](../../docs/MATCH_PREPARATION.md)를 먼저 수행합니다. 준비 신호와 경기 시작은 분리되며 READY 전에는 시작이 차단됩니다. 실물 robot_host 통합 실행에는 새 자체 bringup 배포가 필요합니다.

로봇은 센서/모터 드라이버, PC는 공 인식·월드 모델·행동 트리·제어를 실행합니다.
PC의 `real_game.launch.py`는 시뮬레이터나 실물 드라이버를 추가 실행하지 않습니다.
기본 경기장은 하드보드, 우리 골대는 노랑(-X), 공격은 파랑(+X)입니다.

## 준비와 터미널

기존 PC의 `world_model.launch.py`, `ball_detector`/미리보기, BT, controller는
각 실행 터미널의 Ctrl+C로 종료해 중복 노드·TF·주행 발행을 피합니다.
보정 UI의 주행 실험·키보드 조종도 함께 실행하지 않습니다.
로봇의 기존 `robot.launch.py`는 한 개만 유지합니다.

로봇에서 아직 실행하지 않았다면:

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/local_setup.bash
source ~/pinky_soccer_ws/install/local_setup.bash
export ROS_DOMAIN_ID=37
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 launch pinky_soccer_bringup robot.launch.py imu_enabled:=true adc_enabled:=true
```

PC에서는 처음 한 번 빌드합니다:

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_world_model pinky_soccer_viz
```

PC에서 사용할 **모든 터미널**에 다음 환경을 설정합니다.
domain 37은 2026-11-21 실물 확인값입니다. 로봇 설정을 바꿨다면 함께 맞춥니다.
기존 CycloneDDS XML은 현재 네트워크/로봇 주소가 맞아야 합니다.

```bash
source ~/pinky-soccer/install/setup.bash
export ROS_DOMAIN_ID=37
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

로봇을 **경기장 중앙, 파란 골대 방향**으로 놓은 다음 PC 터미널 A에서:

```bash
ros2 launch pinky_soccer_viz real_game.launch.py field:=hardboard allow_uncalibrated:=true
```

기본 `localization:=lidar`에서는 AMCL이 경기장 지도와 스캔을 정합합니다.
골대 색/지자기 기준을 기록해180° 대칭 후보도 확인합니다([절차](../pinky_soccer_world_model/LOCALIZATION.md)).
`localization:=odom_only`일 때만 첫 odom 수신을 map `(0,0,0)`에 맞춥니다.
이미 켜진 모터 드라이버의 odom을 0으로 리셋하거나 로봇을 재부팅할 필요는 없습니다.
이후 로봇을 손으로 들어 옮겼다면 정지 후 launch를 재실행해 시작 위치를 다시 등록합니다.
다른 시작 위치는 `initial_x`, `initial_y`(m), `initial_yaw`(rad)를 지정합니다.
예: 노랑 진영에서 파랑을 향하면 `initial_x:=-0.3 initial_y:=0.0 initial_yaw:=0.0`.
`our_goal_color:=blue`도 가능하며 축 자체는 바뀌지 않습니다.

RViz와 원본/공 검출 표시/주황 마스크를 나란히 보여주는 카메라 창이 함께 열립니다.
원본과 검출은 같은 노출 시각으로 맞추며 영상에서 공의 녹색 표시를 확인합니다.
`headless:=true`면 두 창 모두 열지 않습니다. 세 환경 공통 실행은 [GAMEPLAY.md](GAMEPLAY.md).
공 인식 영상 `/ball_detection/debug_image`, 추정 탑뷰 `/world_model/topview`는
`ros2 run rqt_image_view rqt_image_view <토픽>`으로 따로 볼 수도 있습니다.
이 단계는 기본 정지 대기입니다.

PC 터미널 B에서 경기 시작:

```bash
ros2 topic pub --once /game/playing std_msgs/msg/Bool '{data: true}'
```

정지:

```bash
ros2 topic pub --once /game/playing std_msgs/msg/Bool '{data: false}'
```

완전히 종료할 때는 정지를 보낸 뒤 터미널 A에서 Ctrl+C를 누릅니다.
기본 전진 상한 0.08 m/s, 회전 상한 0.5 rad/s입니다.
예: `max_lin:=0.12 max_ang:=0.7`로 조정할 수 있습니다.
제어기는 목표/월드 모델 수신이 0.5초 끊기면 0을 보내고,
실물 launch는 odom이 0.5초 끊기면 월드 모델의 playing을 false로 내보냅니다.
로봇 쪽 watchdog도 명령 단절 시 정지합니다.

## 미보정 시험 모드의 의미

`allow_uncalibrated:=true`를 **명시했을 때만**, K가 없는 영상에 다음 가정을 사용합니다:

- 수평 시야각 66°: 공식 시뮬레이션의 1.1519 rad에 근거한 **시험 가정**으로 실제 렌즈 실측값이 아님.
- fx=fy, 주점은 영상 중앙, 왜곡은 0. 해상도가 바뀌면 추정 K도 재계산.
- 장착 위치/각도는 실행 중인 URDF/TF, 공 중심 높이는 기존 골프공 기본 반지름.

영상에 `APPROXIMATE`를 표시하고 노드 로그에도 경고합니다. 카메라 보정 결과나
센서 CameraInfo를 덮어쓰거나 저장하지 않습니다. 유효한 CameraInfo가 들어오면
보정값을 우선 사용합니다. `camera_yaml:=/절대경로/camera.yaml`도 가능합니다.
`allow_uncalibrated` 기본값은 false이며 기존 보정 요구 동작을 유지합니다.

영상 속 주황색 검출과 대략적인 거리로 추적·탐색·공 뒤로 이동·밀기 흐름을 시험할 수 있으나,
보정 전의 거리 정확도·득점 성공은 보장하지 않습니다. 근거리 사각과 odom 드리프트도
기존 한계입니다. 영상의 위아래 방향과 TF가 맞지 않으면
`image_rotated_180:=true`로 광선 변환을 바꿉니다(표시 영상 자체는 그대로).
공을 앞으로 옮겼을 때 `/ball_detection` x가 커지고, 왼쪽으로 옮기면 y가 커지는지 확인합니다.
음수 방향/큰 거리 오차는 내부·외부 보정으로 해결해야 합니다.

## 구동 없이 통신 흐름 확인

```bash
ros2 launch pinky_soccer_viz real_game.launch.py field:=hardboard \
  allow_uncalibrated:=true command_topic:=/game_test/cmd_vel headless:=true
```

이 실행에서 시작 명령을 보내면 제어 결과는 시험 토픽으로만 나갑니다.
다른 주행 발행기가 꺼져 있어야 하며 실제 플레이 시에는 launch를 종료하고
`command_topic` 옵션을 빼고 다시 실행합니다.
