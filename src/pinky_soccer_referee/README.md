# Pinky Soccer Referee (v0.2.5 prototype)

상부 RealSense RGB 카메라용 독립 ROS 2 Jazzy 패키지. 기존 로봇의
제어, world model, BT, MPC, TF, odometry를 수정하지 않으며 /cmd_vel을 발행하지 않는다.
실제 영상/원본 실험자료는 저장소에 포함하지 않는다.

## 현재 로봇 검출·표시 (2026-11-27 후속 지시 반영)

위치 튐 후속 수정: 짧은 미검출에서 위치 칼만 필터를 버리지 않고0.3초 이내의
동일 트랙에 이어서 사용한다. 신규 검출의 기존 면적 조건은 유지하되 이미 추적 중인
예측 ROI 안에서는 작은 검은 몸체 조각도 중심 측정에 활용한다. 근거리 재획득에
이동 가능 거리 조건을 적용해 다른 후보로 즉시 바뀌지 않도록 했다.
정지 중 관측된 약2cm 중심 흔들림을 고려해 노드의 위치 측정 잡음 초기값은
`robot_position_sigma_m:=0.008`이다. 실제 속도 상한을 추가한 것은 아니다.
RGB 조건·Morphology·원 지름·LED/헤딩 비활성화는 유지한다.
이 후속 수정은 사용자 요청에 따라 추가 검수·테스트·주행을 수행하지 않았다.

실행 파일은 설치 과정에서 복사되므로 소스 변경 후에는 아래 빌드가 필요하다.
`colcon build --symlink-install --packages-select pinky_soccer_referee` 후 기존 심판을
종료하고 재실행한다. 재실행만으로 이전 설치 실행 파일이 갱신되는 것은 아니다.

검은 몸체는 `max(R,G,B) <= 65`와 `max(R,G,B)-min(R,G,B) <= 20`을
동시에 만족하는 픽셀로 추출한다. 최초 검출과 몸체 ROI 추출은 동일한
`black_robot_mask()`를 사용하며 기존7×7 CLOSE와 Contour 처리는 유지한다.
실행 시 `robot_black_max_channel:=65 robot_black_max_spread:=20`으로 조정할 수 있다.
이 파라미터는 기존 설정 방식과 같이 재실행 시 적용한다.

사용자 후속 지시에 따라 실행 경로를 **몸체 중심·이동 속도 전용**으로 바꿨다.
LED 색/팀 인식, LED 기반 헤딩 추정은 실행하지 않는다. 표시도 사다리꼴·박스·헤딩
수치 대신 중심점·지름10cm 고정 원·이동 속도 벡터·속력 수치를 사용한다.
원의 픽셀 반지름은 `round(0.05 * pixels_per_meter)`이며 기본550px/m에서는28px이다.
검출된 몸체 크기나 칼만 필터 상태에 따라 원 크기를 변경하지 않는다.
위치/속도 추적과 기존 칼만 필터는 그대로 사용한다. 기존 ROS 메시지 형식은 유지하고
팀/헤딩/각속도는 unknown 또는 invalid로 제공한다. 이전 LED/헤딩 보조 함수와 파라미터는
호환성을 위해 남아 있으나 현재 referee 노드의 몸체 추출 경로에서는 호출하지 않는다.
아래의 이전 LED·사다리꼴 설명은 과거 구현에 관한 것이며 현재 실행에는 적용되지 않는다.

RGB 마스크 단계에서61 tests 통과 및 RGB 녹화2개588개 표본의 전후 비교를 수행했다.
균등16프레임에서 확인한 로봇31개는 양쪽 모두 검출됐으며, 차이가 난 프레임을
직접 확인해 바닥 그림자를 이전 검출기만 로봇으로 잡는 사례를 확인했다.
공 검출 출력은588표본 모두 동일했다. 이 결과는 LED 인식 제거 전의 마스크 변경 비교다.
**이후 사용자 지시로 추가 검증을 중단했으므로 몸체 전용 경로와 원 표시의 최종 상태는 미검증이다.**
자료는 로컬 `docs/validation/referee_black_rgb_20261127/`에 보존했다.

## 경기 준비·시작 UI (2026-11-27)

상부 심판 영상 아래의 버튼에서 기존 로봇 준비 상태 관리자를 조작한다.
카메라 초기화 전이나 영상 수신이 끊긴 경우에도 제어 패널은 표시된다.
로봇이 보고하는 상태·팀·준비 완료 여부·LED·실패 사유를 표시한다.

| 버튼 / 키 | 동작 |
|---|---|
| PREPARE / P | 기존 로봇 위치 초기화→팀 시작 위치 이동 요청 |
| START / G | READY 또는 준비 완료 이력이 있는 STOPPED에서 경기 시작 |
| STOP / S 또는 Esc | 준비·경기 중지, 진행 중인 UI 요청보다 우선 |
| RETRY / R | 로봇 준비 재시도 |

최신 `/preparation/status`의 `ready=true`, `localization_valid=true`를 확인해야 START를 허용한다.
상태가1.5초 이상 오래되거나 서비스가 없으면 준비/시작은 비활성화한다.
중복 준비 요청을 막으며 서비스 실패·3초 응답 지연은 STOP을 요청한다.
STOP 후 이전 요청이 늦게 반영되어 움직이는 상태가 보고되면 STOP을 재요청한다.
카메라에서 추정한 로봇 Pose나 팀 색으로 준비 완료를 대신 판정하지 않는다.

버튼은 로봇의 기존 `std_srvs/Trigger` 서비스를 비동기로 호출한다.
`/cmd_vel`, `/game/playing`, 로봇 위치/TF를 직접 발행하지 않는다.
득점/아웃 이벤트를 기존 경기 이벤트로 변환하거나 전략·제어기를 변경하는 기능은 이번 연결에 포함하지 않는다.
준비 버튼은 상부 카메라 보정이 아닌 **로봇의 기존 준비 절차**를 시작한다.

PC 빌드:

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_referee pinky_soccer_viz
source install/setup.bash
```

로봇 PC 스택은 기존 보정·MPC 설정으로 별도 실행한다. 상부 심판과 함께 쓸 때는
`game.launch.py` 명령에 `referee:=false`를 추가해 기존 world-model 심판을 끈다.
경기 전략을 START로 실행하려면 `verify:=false`, GoToPose 수동 검증을 계속하려면 `verify:=true`다.
`preparation:=true`는 유지한다. 두 프로그램의 ROS_DOMAIN_ID와 DDS 환경도 일치해야 한다.

상부 심판 실행:

```bash
ros2 launch pinky_soccer_referee referee.launch.py mode:=realsense
```

기본 로봇 namespace는 없음(`/preparation/...`). 로봇 스택이 `robot_name:=yellow`이면:

```bash
ros2 launch pinky_soccer_referee referee.launch.py \
  mode:=realsense robot_namespaces:=yellow
```

두 로봇은 `robot_namespaces:=yellow,blue`로 지정한다. PREPARE/STOP은 둘 다 요청하며
START는 둘 다 최신 READY일 때만 허용한다. 골대 색/트랙 ID로 namespace를 추측하지 않는다.
`match_controls:=auto`가 기본이며 realsense/topic에서 켜지고 영상 재생(video)에서는 꺼진다.
읽기 전용은 `match_controls:=false`, 영상 재생에서도 명시적으로 제어하려면 true다.

검증: 기존 영상·규칙 포함63 tests, 두 PC 패키지 빌드, 설치 노드 생성/패널 렌더링 및
격리 ROS 가짜 서비스의 준비→READY 게이트→시작→중지 확인. 실제 UI 클릭으로 핑키를
주행시키는 종단 간 시험은 미검증이다. 핑키 하드웨어 소스 재전송은 필요 없다.

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

## 통합 launch (권장)

RealSense ROS 드라이버는 RealSense 모드에서만 별도로 설치되어 있어야 한다.

    sudo apt install ros-jazzy-realsense2-camera

카메라와 심판 노드를 한 번에 실행 (기존 별도 카메라 노드는 먼저 종료):

    ros2 launch pinky_soccer_referee referee.launch.py mode:=realsense

실험용 골/아웃 판정까지 활성화:

    ros2 launch pinky_soccer_referee referee.launch.py mode:=realsense allow_approximate_rules:=true

카메라는 RGB만 활성화한다. 기본 color_profile은 1280x720x30이며 장비에서
지원되는 프로파일로 변경할 수 있다. camera_namespace와 camera_name은 기본 camera이며
변경하면 심판 구독 토픽도 자동으로 일치한다. 여러 카메라가 있으면 serial_no로 선택할 수 있다
(설치된 RealSense wrapper의 serial_no 형식에 맞게 입력). show_ui 기본 true,
allow_approximate_rules 기본 false. 치수 기본값은 1.670 x 1.070 m이다.
모든 launch 옵션 확인:

    ros2 launch pinky_soccer_referee referee.launch.py --show-args

MP4 영상만 실행 (카메라 드라이버 불필요):

    ros2 launch pinky_soccer_referee referee.launch.py mode:=video video_path:=/absolute/path/video.mp4

기존에 실행 중인 카메라 토픽을 사용:

    ros2 launch pinky_soccer_referee referee.launch.py mode:=topic image_topic:=/camera/camera/color/image_raw camera_info_topic:=/camera/camera/color/camera_info

mode:=topic은 카메라를 추가로 실행하지 않는다. 모드별 launch 실물 실행 검증은 아직 필요하다.

## 영상 재생으로 먼저 테스트 (직접 실행)

    ros2 run pinky_soccer_referee referee_node --ros-args \
      -p video_path:=/absolute/path/realsense_20261126_224946.mp4 \
      -p field_length_m:=1.670 -p field_width_m:=1.070 \
      -p show_ui:=true -p allow_approximate_rules:=true

영상 재생 모드는 매체 프레임 간격으로 속도를 계산하고 ROS 수신 시각을 헤더로 사용한다.
종료 시 watchdog이 객체 상태를 stale로 갱신한다. 실제 검증에는 측정한 치수를 넣어야 한다.
allow_approximate_rules는 실험용 판정에만 사용한다.

## RealSense 실시간 모드 (직접 실행)

통합 launch 대신 수동 실행하려면 RealSense ROS 드라이버를 별도로 실행하고 실제 토픽을 확인한다.

    ros2 topic list | grep -E 'image_raw|camera_info'

    ros2 run pinky_soccer_referee referee_node --ros-args \
      -p image_topic:=/camera/camera/color/image_raw \
      -p camera_info_topic:=/camera/camera/color/camera_info \
      -p field_length_m:=1.670 -p field_width_m:=1.070 \
      -p show_ui:=true

실제 이미지 토픽명과 치수를 수정한다. 초기 30 프레임 이후 녹색 영역과 흰색 선을
자동 검출하며, 실패하면 10 프레임 간격으로 재시도한다. 기본 auto 모드에서는 CameraInfo(유효한 경우), 추가 보정 없음, plumb-line 방사 왜곡 모델을 비교한다.
초기 프레임의 중앙값 영상을 이용하며, 실제 라인 정합 오차를 검증한 모델만 사용한다. require_camera_info:=true로
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
휴리스틱 품질 지표이며 실제 확률이 아니다. 노란 골대 입구는 노랑, 파란 골대 입구는 파랑으로 강조한다.

**메시지 변경:** TrackedObject에 team, team_valid, angular_velocity_valid,
heading_confidence, tracking_confidence, reacquired를 추가했다.
이전 v0.1 메시지로 빌드한 모든 구독 노드는 새 버전으로 재빌드해야 한다.

실제 카메라 영상에서 로봇의 바퀴 축 중심은 아직 실측하지 않았다.
높이별 시차 및 실제 LED 발광 색/밝기/포화 검증도 필요하다.
후방 LED 색/배치가 실제 기구와 일치하는지 확인한다.

### v0.2.2 LED 판별

LED의 기하학적 위치 검출과 색 판별을 분리한다. 밝기 기반 공통 후보를 몸체 근처로 제한하고,
길게 이어지는 바닥 라인을 검사한다. 선택된 LED 회전 사각형의 중심부(가로·세로 70%)만 색 판별에 사용한다.
노랑은 HLS H=18~38, S>=145, L>=100 및 RGB 채널 차이 조건을 만족하는 픽셀이 75% 이상이어야 한다.
파랑은 H=90~135, S>=125, L>=blue_led_min_light 및 B 채널 우세 조건을 만족하는 픽셀이 70% 이상이어야 한다.
그 외(흰색/off, 애매한 색)는 UNKNOWN. 팀 색은 5회 연속 확인 후 발표하고 UNKNOWN 관측 시 해제한다.
색 변화만으로 위치 추적을 끊지 않으며, LED 검출 여부에 관계없이 동일한 몸체 중심을 사용한다.
실물 LED 및 라인 겹침에서의 성능은 검증 필요. 메시지 타입 및 launch 인수는 v0.2.1과 동일하다.

## v0.2.3 보정 및 헤딩 안정화

기존 흰색 외곽선 기준 1.670 x 1.070 m, 노란 골대 좌측, 청록 외곽선, 팀별 골라인을 유지한다.

- 초기 warmup 이후 3프레임마다 샘플을 수집하여 기본 9장 중앙값 영상으로 보정한다.
- distortion_mode:=auto는 CameraInfo(지원 모델/해상도), none(추가 렌즈 보정 없음), plumb_line(직선성 추정)을 비교한다.
- 각 변의 p90 픽셀 잔차 및 실제 보정 영상의 흰 테이프 정합을 확인한다.
  기본 상한 5 px를 넘거나 충분한 라인이 없으면 초기화를 다시 수행한다.
- 골대 입구는 골라인 외측 좁은 영역의 색 점유율로 측정한다.
  자동 검출 길이 0.18~0.55 m 및 좌우 길이비 1.20 이하를 초기 가정으로 둔다.
  실제 입구가 다르면 자동 검출 범위를 조정해야 한다. 실측값 파라미터는 유지한다.
- LED 후보는 이전 헤딩과 후방 LED 상대 위치 주변도 탐색한다.
- 로봇 위치/선속도와 헤딩/각속도는 소형 constant-velocity Kalman filter로 추정한다.
  재획득 시 상태를 재초기화하며 위치 점프를 속도로 계산하지 않는다.
- LED가 잠깐 누락되면 최대 0.25초 동안 UI에 PRED 헤딩을 표시한다.
  예측만 있는 프레임에서는 ROS heading_valid=false, heading=NaN이며 각속도도 무효다.
  새 측정과 예측을 구분하며, 장시간 누락 시 사각형으로 복귀한다.
- 디버그 UI는 각 변 p90 오차, 보정 모델, LED 후보 수, 헤딩 상태 및 신뢰도를 표시한다.

    ros2 launch pinky_soccer_referee referee.launch.py mode:=realsense allow_approximate_rules:=true debug_ui:=true

| 추가 인수 | 기본값 |
|---|---|
| distortion_mode | auto |
| calibration_frames | 9 |
| calibration_max_error_px | 5.0 |
| debug_ui | false |

비교 실험은 distortion_mode:=camera_info, distortion_mode:=plumb_line 또는 distortion_mode:=none으로
강제할 수 있다. none도 Homography는 적용한다. 카메라 이동 후에는 recalibrate 서비스 호출.
위 임계값은 초기 설계값이며 ROS 및 실제 LED/경기장 검증 필요. 메시지 정의는 v0.2.2와 동일하다.

## v0.2.4 몸체 좌표계 LED 탐색 및 디버그 기록

기존 보정, 골대 및 공 검출 알고리즘과 ROS 메시지·서비스를 유지한다.
기존 LED contour 탐색에 실패하거나 방향 후보가 충돌하면, 몸체 장축 양 끝과
이전 헤딩 주변을 몸체 좌표계에서 재탐색한다. 밝은 횡방향 띠의 실제 길이/두께,
끝점, 몸체 안쪽의 어두운 픽셀, 라인 연장 여부를 확인한다.
LED 색상 분류는 기존 v0.2.2의 엄격한 조건을 유지한다.
현재 프레임 영상 증거가 없는 경우 측정으로 처리하지 않는다.
헤딩 예측의 0.25초 제한과 위치 재획득 정책은 그대로 유지한다.

    ros2 launch pinky_soccer_referee referee.launch.py mode:=realsense allow_approximate_rules:=true debug_ui:=true debug_log:=true record_video:=true

| 인수 | 기본값 | 기능 |
|---|---|---|
| debug_ui | false | 보정 오차, LED 후보 수, contour/body_end/missing 및 헤딩 상태 표시 |
| debug_log | false | 프레임별 수치·검출 탈락 사유·캘리브레이션 기록 |
| record_video | false | 원본, 보정, 오버레이 영상 기록 |
| debug_dir | ~/pinky_referee_logs | 날짜별 세션 디렉터리의 상위 경로 |
| record_fps | 30.0 | 실시간 녹화 MP4 명목 FPS (영상 모드는 원본 FPS) |

각 세션에는 frames.jsonl(영상 파일/프레임 인덱스 및 시간), tracking.csv,
detection.jsonl(후보별 필터 탈락 횟수 집계 및 선택된 LED의 위치), performance.csv,
events.csv, calibration_*.yaml 및 raw.mp4, rectified.mp4, overlay.mp4가 옵션에 따라 생성된다.
raw는 초기화 단계도 포함하며 보정 영상은 캘리브레이션 완료 이후만 기록한다.
영상 해상도가 바뀌면 새 파일로 분할한다. 프레임별 시각은 MP4의 명목 FPS가 아니라
frames.jsonl의 t와 stamp를 기준으로 해석한다.
저장은 제한된 비동기 큐를 사용하며 과부하 시 기록 프레임을 누락할 수 있다.
종료 시 Ctrl+C를 사용해 큐를 정리하고 터미널에서 dropped_frames 및 오류를 확인한다.
MP4 세 영상 동시 인코딩은 추가 부하가 있으므로 성능 검증 시 record_video를 끈 상태도 측정한다.
LED 후보의 탈락 사유는 집계이며 원본 영상과 함께 분석한다.
ROS 및 실물 영상 검증 전의 초기값/프로토타입이다.

## v0.2.5 경사진 LED 및 위치 중심 안정화

LED 법선과 검은색 몸체 장축이 불일치해도, LED 한쪽의 몸체 점유율이 65% 이상이고
양쪽 점유율 차가 35%p 이상이면 장축 조건을 완화한다. 바닥 테이프 연장 검사와
LED 크기·충실도 조건은 유지한다. 몸체 기반 보조 탐색은 장축/단축 양 끝을 모두 검사한다.
헤딩 부호는 기존과 동일하게 LED를 후방으로 간주한다. 실물 전방 기준을 확인해야 한다.

발행 위치는 검은색 최대 contour의 면적 모멘트 중심으로 바꾼다.
LED 탐색용 회전 사각형 중심은 별도 유지한다. 이 중심은 바퀴 축의 실측 위치가 아니며,
정확한 물리 원점과 높이별 시차 보정은 별도 작업이 필요하다.
기존 Kalman filter, 경기장 보정, 골대/공 검출, 메시지 및 launch 옵션은 유지한다.

팀 판별에서는 최대 RGB >=245 및 최소 RGB >=200인 밝은 무채색 포화 픽셀을 제외한다.
남은 유효 픽셀은 전체의 35% 이상이어야 하며 기존 HLS/RGB 색 조건도 만족해야 한다.
전체 LED 샘플 대비 노랑 55%, 파랑 50% 이상의 색 증거도 요구한다.
흰색/OFF 및 과노출로 색 정보를 잃은 LED는 UNKNOWN이다. 헤딩과 색은 분리한다.

Pinky Pro 공식 LED 서비스로 발광 밝기를 수동 시험할 수 있다.
심판 노드 자체는 로봇 명령을 발행하지 않는다. 연결 대상 로봇의 ROS_DOMAIN_ID 및
네임스페이스를 확인하고 실행하며 기존 LED 서버가 있으면 중복 실행하지 않는다.

    ros2 service list | grep -E 'set_led|set_brightness'
    ros2 service call /set_brightness pinky_interfaces/srv/SetBrightness "{brightness: 10}"
    ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'fill', r: 255, g: 255, b: 0}"
    ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'fill', r: 0, g: 0, b: 255}"
    ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'clear'}"

밝기는 5, 10, 15, 20 등에서 카메라 영상으로 비교한다.
정확한 밝기 설정 범위는 설치된 pinkylib 구현에서 확인하며, 색상 분리가 불충분하면
카메라 노출과 LED 출력값도 조정한다. 실제 노란색/파란색 LED 데이터의 검증은 아직 필요하다.

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

2026-11-28: 상단 `SCORE Yellow / Blue`는 팀 득점이다. 파란 골대 진입은
Yellow 득점, 노란 골대 진입은 Blue 득점이다. ROS `yellow_goal_entries` /
`blue_goal_entries`는 기존처럼 각 색상 골대에 들어간 횟수를 유지한다.

2026-11-28 준비 버튼: PREPARE/RETRY는 상태·서비스 조건으로 회색 비활성화하지
않고, 클릭할 때 로봇별 연결 대기/준비 중/READY 이유를 표시한다. PREPARE는
IDLE/ERROR/STOPPED/RECOVERING 대상만 요청하고, READY 대상 재준비는 RETRY를
사용한다. 이미 CALIBRATING/MOVING_TO_START인 대상은 중복 요청하지 않는다.
경기 중에는 STOP 후 준비한다. START는 모든 지정 로봇의 fresh READY/유효 위치
조건을 유지한다. robot_namespaces가 비었고 전역 준비 서비스가 없으면 Yellow/Blue
경기 서비스를 자동 연결한다. 단일 이름 로봇은 robot_namespaces:=yellow처럼 명시한다.
이 변경은 사용자 요청으로 빌드만 반영했으며 UI 동작 시험은 수행하지 않았다.
