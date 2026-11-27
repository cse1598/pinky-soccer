# Pinky Soccer Referee (v0.1.0 prototype)

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
초록색 바닥 전체 규격과 혼동하지 말 것. 기본값 0은 실행 시 오류를 발생시켜
실측값 누락을 방지한다. 예제의 1.70 x 1.10 m는 임의 테스트 값이며 실측값이 아니다.
원점은 흰색 사각형의 기하학적 중심, +x는 탑뷰 우측, +y는 탑뷰 상단이다.
기존 map TF와 자동 연결하지 않는다. 각 골대의 색은 득점 팀이 아닌 공이 들어간 골대의 색이다.
ball_radius_m 기본값 0.02135는 기존 프로젝트 README의 42.7 mm 직경에 기반한다.

## 영상 재생으로 먼저 테스트

    ros2 run pinky_soccer_referee referee_node --ros-args \
      -p video_path:=/absolute/path/realsense_20261126_224946.mp4 \
      -p field_length_m:=1.70 -p field_width_m:=1.10 \
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
      -p field_length_m:=1.70 -p field_width_m:=1.10 \
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

모든 위치/속도 단위는 m, m/s. 속도는 위치차의 지수평활 추정이며 헤딩/각속도는
미구현으로 heading_valid=false, NaN을 발행한다. visible=false이면 위치를,
velocity_valid=false이면 속도를 사용하지 않는다. 초기 로봇 ID는 좌우 위치순이며
가림/교차/재획득 뒤 ID가 바뀔 수 있다. 팀 고정 식별은 후속 ArUco 방식으로 구현한다.
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
yellow_goal_side, blue_goal_side는 기본 -1,+1이다. 측정 전에는
allow_approximate_rules:=true를 명시하여 시험용 색상 골대 검출에 기반한 판정을 활성화할 수 있다.

## 검증 및 한계

    python3 -m pytest src/pinky_soccer_referee/test -q

단위 테스트는 판정 latch, 공 누락, 완전한 라인 통과, 골대 미검출, 로봇 추적과 속도 유효성을 확인한다.
이 브랜치 생성 환경에서는 ROS 빌드 및 연결 테스트를 실행하지 못했다.
기존 대화에서 실험한 HSL/외곽선 보정과 동일한 개념을 신규 코드로 구현했으며,
동일한 영상에서 새 코드가 동일 결과를 내는지는 재생 시험이 필요하다.

실물 시험: (1) 고정 영상 재생, (2) RealSense 실시간 영상 확인, (3) 최소 10점
바닥 위치 오차 검증, (4) 로봇 높이에 의한 시차 보정, (5) 공 이동/가림/골대 통과/아웃
각각 기록, (6) 실제 입력률 및 처리 지연 측정, (7) 로봇 ArUco ID 및 yaw 추가.
정량 검증 전에는 ground-truth **추정치**로만 사용하고 자동 경기 제어나 안전 기능에 연결하지 않는다.
