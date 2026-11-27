# Pinky Soccer Referee (v0.2.0 prototype)

상부 RealSense RGB 카메라용 독립 ROS 2 Jazzy 패키지. 기존 로봇의
제어, world model, BT, MPC, TF, odometry를 수정하지 않으며 /cmd_vel을 발행하지 않는다.
실제 영상/원본 실험자료는 저장소에 포함하지 않는다.

## 설치

기존 작업 브랜치를 전환하지 않고 별도 worktree에서 시험한다.

    cd ~/pinky-soccer
    git fetch origin feature/referee-vision
    git worktree add -b referee-test ~/pinky-referee-test origin/feature/referee-vision
    cd ~/pinky-referee-test
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths src/pinky_soccer_referee --ignore-src -r -y
    colcon build --symlink-install --packages-select pinky_soccer_referee
    source install/local_setup.bash

이 패키지는 다른 Pinky 패키지나 submodule을 필요로 하지 않는다.
OpenCV GUI 지원이 있는 Ubuntu 데스크톱 환경에서 show_ui를 활성화한다.

## 실측값과 좌표계

field_length_m, field_width_m은 **흰색 외곽선 중심 간 실측 거리**이다.
초록색 바닥 전체 규격과 혼동하지 말 것. 사용자 지정 기본값은 1.670 x 1.070 m이다.
이 수치는 이 패키지의 흰색 선 중심 기준으로 해석하며 실제 선 중심 치수는 별도 확인한다.
원점은 흰색 사각형의 기하학적 중심, +x는 파란 골대 방향, +y는 탑뷰 상단이다.
양쪽 골대 색을 초기화 때 확인하고 필요하면 전체 보정 맵과 좌표계를 180도 회전한다.
노란 골대가 항상 왼쪽이다. 두 색이 확인되지 않으면 초기화를 재시도한다.
기존 map TF와 자동 연결하지 않는다. 각 골대의 색은 득점 팀이 아닌 공이 들어간 골대의 색이다.
ball_radius_m 기본값 0.02135는 기존 프로젝트 README의 42.7 mm 직경에 기반한다.

## 영상 재생으로 먼저 테스트

    ros2 run pinky_soccer_referee referee_node --ros-args \
      -p video_path:=/absolute/path/realsense_20261126_224946.mp4 \
      -p field_length_m:=1.670 -p field_width_m:=1.070 \
      -p show_ui:=true -p allow_approximate_rules:=true

영상 재생 모드는 매체 프레임 간격으로 속도를 계산하고 ROS 수신 시각을 헤더로 사용한다.
종료 시 watchdog이 객체 상태를 stale로 갱신한다. 실제 검증에는 측정한 치수를 넣어야 한다.
allow_approximate_rules는 실험용 판정에만 사용한다.

## RealSense 실시간 모드

RealSense ROS 드라이버를 별도로 실행하고 실제 토픽을 확인한다.

    ros2 topic list | grep -E 'image_raw|camera_info'

    ros2 run pinky_soccer_referee referee_node --ros-args \
      -p image_topic:=/camera/camera/color/image_raw \
      -p camera_info_topic:=/camera/camera/color/camera_info \
      -p field_length_m:=1.670 -p field_width_m:=1.070 \
      -p show_ui:=true

실제 이미지 토픽명과 치수를 수정한다. 초기 30 프레임 이후 녹색 영역과 흰색 선을
자동 검출하며, 실패하면 10 프레임 간격으로 재시도한다. CameraInfo가 있으면
해당 해상도와 모델을 검증하고 내부 파라미터를 사용한다. 없으면 고정 중심/초점
가정의 plumb-line 방사 왜곡 추정으로 대체한다. require_camera_info:=true로
대체를 금지할 수 있다. remap 테이블은 초기화에만 계산한다. 실제 경기장 외곽과 골대까지
남도록 초록색 외곽 + margin_m의 확장 캔버스를 사용한다. 카메라를 움직였거나
해상도를 변경했다면 재보정한다. 카메라 움직임 자동 감지는 아직 없다.

## ROS 인터페이스

| Topic | Type | Meaning |
|---|---|---|
| /referee/state | pinky_soccer_referee/msg/RefereeState | 전체 상태, stamp, validity, ball/robots, score |
| /referee/ball | pinky_soccer_referee/msg/TrackedObject | 공의 x,y,vx,vy |
| /referee/robot_1 | pinky_soccer_referee/msg/TrackedObject | 임시 트랙 1 |
| /referee/robot_2 | pinky_soccer_referee/msg/TrackedObject | 임시 트랙 2 |
| /referee/events | pinky_soccer_referee/msg/RefereeEvent | GOAL/OUT/UNVERIFIED_EXIT, sequence, goal color |
| /referee/top_view | sensor_msgs/msg/Image | 보정된 RGB 영상 + 상태/추적 표시 |

모든 위치/속도 단위는 m, m/s. 헤딩은 rad, 각속도는 rad/s이며 +X 기준 반시계가 양수이다.
후방 LED 장축에 수직인 방향 중 몸체를 향하는 방향을 전방으로 사용한다.
흰색 LED는 헤딩만, 노랑/파랑 LED는 헤딩과 팀을 추정한다. 파란 바퀴 오인식을 줄이기 위해 밝기,
직사각형 충실도, 크기, 장축과 몸체 축의 관계, 후방 위치 및 몸체 인접 여부를 확인한다.
불확실한 경우 heading_valid=false 및 NaN, team=unknown, team_valid=false를 발행한다.
각속도는 연속 유효 헤딩에서만 계산한다. visible=false이면 위치를, velocity_valid=false이면 속도를 사용하지 않는다.
로봇 ID는 임시 트랙이며 LED 팀 정보로 재연결을 보조한다. 같은 색 또는 미확인 상태에서 ID 교환 가능성이 있다.
UI의 사다리꼴 짧은 변은 헤딩이며 중심 십자는 로봇의 영상상 기하학적 원점 추정치다.
헤딩 미검출 시 방향 없는 박스를 표시한다. 로봇이 정지해 있으면 화살표를 그리지 않는다.
UI 화살표는 속도 방향이며 로봇 헤딩이 아니다. 이벤트/상태는 reliable transient local QoS
(재구독 시 과거 이벤트는 sequence와 stamp로 구분). 프레임이 끊기면 VIDEO_STALE을 발행한다.

    ros2 topic echo /referee/state
    ros2 topic echo /referee/events --qos-durability transient_local
    ros2 topic hz /referee/state
    ros2 run rqt_image_view rqt_image_view

| Service | Type | Function |
|---|---|---|
| /referee/get_state | pinky_soccer_referee/srv/GetState | 최신 상태 조회 |
| /referee/is_out | std_srvs/srv/Trigger | 마지막 확정 이벤트가 OUT인지 |
| /referee/is_goal | std_srvs/srv/Trigger | 마지막 확정 이벤트가 GOAL인지 |
| /referee/reset_play | std_srvs/srv/Trigger | 점수 유지, 다음 판정 준비 |
| /referee/reset_score | std_srvs/srv/Trigger | 점수 초기화 |
| /referee/recalibrate | std_srvs/srv/Trigger | 매핑 삭제 및 영상 재보정 |

    ros2 service call /referee/get_state pinky_soccer_referee/srv/GetState '{}'
    ros2 service call /referee/is_goal std_srvs/srv/Trigger '{}'
    ros2 service call /referee/reset_play std_srvs/srv/Trigger '{}'
    ros2 service call /referee/recalibrate std_srvs/srv/Trigger '{}'

## 판정 정책 (초안)

흰색 라인 전체를 공의 반지름만큼 완전히 벗어난 상태가 3개 연속 프레임에서
관측되면 이벤트 1회 발행. 골라인 방향으로 벗어나고 골대의 y 범위 안에 공 전체가
있으면 GOAL, 나머지 경계는 OUT이다. 골대 위치가 불명인 골라인 이탈은 UNVERIFIED_EXIT.
공이 사라지는 것만으로는 득점/아웃으로 판단하지 않는다.
판정은 reset_play까지 latch되며, 다음 공이 경기장 내부에서 관측되면 arm한다.
실제 골대 입구와 골라인이 일치하는지 및 경기 규칙은 운영자가 확인해야 한다.
현재 영상기반 프로토타입은 실제 골문 진입을 별도로 센싱하는 방식이 아니다.

기본적으로 두 골대 입구 범위를 실측 입력하고 CameraInfo를 사용해야 규칙을 활성화한다.
예: yellow_goal_y_min/y_max, blue_goal_y_min/y_max (m, 경기장 원점 기준).
yellow_goal_side, blue_goal_side는 정규 좌표계에서 -1,+1로 고정된다. 측정 전에는
allow_approximate_rules:=true를 명시하여 시험용 색상 골대 검출에 기반한 판정을 활성화할 수 있다.

## v0.2 추적/표시 옵션

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| field_length_m | 1.670 | 흰 외곽선 중심 간 길이(m) |
| field_width_m | 1.070 | 흰 외곽선 중심 간 폭(m) |
| robot_match_gate_m | 0.12 | 예측 위치 매칭 거리 상한(m) |
| robot_reacquire_frames | 2 | 재탐색 후보 연속 확인 프레임 수 |
| blue_led_min_light | 125 | 파랑 LED HLS 명도 하한(0~255) |
| velocity_arrow_min_mps | 0.035 | 속도 화살표 표시 최소 속도(m/s) |

매 프레임 전체 영역에서 검출 후 거리+예측 bbox IoU로 전역 매칭한다.
매칭 실패 시 트랙을 무효화하고 미사용 후보가 2프레임 일관되면 재초기화한다.
재획득 프레임은 reacquired=true, velocity_valid=false로 표시하여 순간 위치 점프를
실제 속도로 발행하지 않는다. tracking_confidence와 heading_confidence는
휴리스틱 품질 지표이며 실제 확률이 아니다. 골대 라인 구간은 양쪽 모두 노랑으로 강조한다.

**메시지 변경:** TrackedObject에 team, team_valid, angular_velocity_valid,
heading_confidence, tracking_confidence, reacquired를 추가했다.
이전 v0.1 메시지로 빌드한 모든 구독 노드는 새 버전으로 재빌드해야 한다.

실제 카메라 영상에서 로봇의 바퀴 축 중심은 아직 실측하지 않았다.
높이별 시차 및 실제 LED 발광 색/밝기/포화 검증도 필요하다.
후방 LED 색/배치가 실제 기구와 일치하는지 확인한다.

## 검증 및 한계

    python3 -m pytest src/pinky_soccer_referee/test -q

단위 테스트는 판정 latch, 공 누락, 완전한 라인 통과, 골대 미검출, 로봇 추적과 속도 유효성을 확인한다.
이 브랜치 생성 환경에서는 ROS 빌드 및 연결 테스트를 실행하지 못했다.
기존 대화에서 실험한 HSL/외곽선 보정과 동일한 개념을 신규 코드로 구현했으며,
동일한 영상에서 새 코드가 동일 결과를 내는지는 재생 시험이 필요하다.

실물 시험: (1) 고정 영상 재생, (2) RealSense 실시간 영상 확인, (3) 최소 10점
바닥 위치 오차 검증, (4) 로봇 높이에 의한 시차 보정, (5) 공 이동/가림/골대 통과/아웃
각각 기록, (6) 실제 입력률 및 처리 지연 측정, (7) 흰색·노랑·파랑 LED 상태, 바퀴 오인식, 360도 회전, 들어 옮기기 검증.
정량 검증 전에는 ground-truth **추정치**로만 사용하고 자동 경기 제어나 안전 기능에 연결하지 않는다.
