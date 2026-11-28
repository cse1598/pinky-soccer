# Pinky Soccer 판단 계층 구조

이 문서는 월드 모델·행동 트리·제어·UI 계층의 구조와 규칙이다.
에이전트 협업 규칙은 `.agents/RULES.md`(로컬), 센서 계약은 `src/pinky_soccer_sim/config/interface.yaml`, 경기장은 `src/pinky_soccer_sim/FIELD.md`가 기준이다.

## 전제

- PinkLAB Pinky Pro(차동구동) 1대1 자율 축구, ROS 2 Jazzy.
- 센서: 로봇 자체 카메라(공·상대), 2D 라이다, 휠 odom, IMU, 초음파(보조). 천장 카메라 없음. 시야가 좁아 **월드 모델**이 핵심이다.
- 공: 주황색 골프공 (사용자 실측 지름 42.7 mm, 형광 주황색. 질량 45.9 g은 미실측 가정).

## 5단 구조 + UI

```
1. 관측   /camera/image_raw · /scan · /odom · /imu_raw · /us_sensor/range   (기존 센서 계약)
          ball_detector → /ball_detection (PointStamped, base_link)
2. 추정   rectangle_localization(외벽·초기 HLS·IMU·odom) + world_model_node
            → /world_model    (pinky_soccer_msgs/WorldModel, map, ~30 Hz)
3. 판단   bt_node (BehaviorTree.CPP v4 + BehaviorTree.ROS2)
            → /behavior_goal  (pinky_soccer_msgs/BehaviorGoal, 10–20 Hz)
4. 계획   mpc_controller(pinky_soccer_mpc, 기본: 목표 자세 NMPC, 바퀴 한계 = 모터 사양 90 %)
          또는 reactive_controller(pinky_soccer, controller:=reactive) → /cmd_vel
          (Nav2 원거리 계획은 예정)
5. 제어   command_watchdog(0.5 s) → drive/cmd_vel
          실물: pinky_soccer_bringup / 시뮬레이션: pinky_soccer_sim (둘 다 같은 watchdog 사용)

UI       world_model_viz     → /world_model/markers (MarkerArray, RViz2)
         behavior_verifier   ↔ bt_node verify_mode (verify:=true, 행동 하나씩 실행·평가)
         world_model_topview → /world_model/topview (Image, OpenCV)
         game_camera_view   ← ball_detection/source_image · debug_image · debug_mask
입력     /game/playing (std_msgs/Bool, 경기 시작/정지)
심판     game_referee → /game/goal, /game/score, /game/kickoff (골 → 정지 → 킥오프 자리, 시뮬 공 센터 복귀)
```

원칙
- 여러 로봇: 로봇마다 `robot_name` 네임스페이스(`/<이름>/...`, TF는 `/<이름>/tf`), 프레임 이름은 같다.
  경기 전체 토픽 `/game/*`, `/clock`, `/sim/*`만 전역. 노드는 상대 토픽 이름을 쓰고, 절대 이름을 쓰는 노드
  (goal_heading, RViz)는 launch 재매핑(`pinky_soccer_viz/robot_stack.py`). 노드 이름 키 파라미터 파일은
  네임스페이스 안에서 적용되지 않으므로 `/**/` 와일드카드 또는 launch가 만든 사본을 쓴다.
- 행동 트리는 센서를 직접 구독하지 않고 `/world_model`(과 심판 `/game/kickoff`)만 본다.
- 트리는 **목표만** 바꾸고, 따라가기는 계획·제어 층이 연속적으로 처리한다.
- 월드 모델은 각 객체의 관측/추정/분실 상태와 공분산을 함께 낸다. UI는 관측 = 진한 실선, 추정 = 흐린 점선 + 불확실성 타원, 분실 = 마지막 위치 X로 일관되게 그린다.
- `/clock`, `/ground_truth`, `/sim/*`(`/sim/ball_ground_truth` 포함)는 시뮬레이터 전용이다. 판단 입력으로 쓰지 않는다 (UI 평가 표시와 시험 판정만 허용).

## 공격 전용 실행 경로

`pinky_soccer_viz attack.launch.py`는 센서/World Model/준비 관리자/심판을 재사용하고
`attack_only.xml` → `AttackBall` → `attack_controller`(A* + 기하 추종)로 연결한다.
이 실행에서는 MPC/LQR 노드를 생성하지 않는다. 수비는 없으며 bounded push와
재관측 후진, 도착 완료 유지, 외벽/골대/원본 LiDAR 기반 footprint 검사를 사용한다.
심판의 OUT/UNVERIFIED_EXIT는 정지, GOAL은 준비 관리자 소유의 시작 위치 복귀 후
READY 대기다. 재시작은 사람의 START와 중앙 공 관측/판정 armed 확인이 필요하다.
MuJoCo 상부 RGB 영상도 같은 영상 심판을 사용한다. Ground Truth는 평가 전용이고
로봇 센서 위치/공 추정 대신 사용하지 않는다. 기존 기본 실행과 보정 원본은 보존한다.
실행·설계값·cutaway 표시·검증 한계는 [ATTACK_SOCCER.md](ATTACK_SOCCER.md)를 참고한다.

## 토픽 (2026-11-21 확인)

| 토픽 | 타입 | 발행 | 비고 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | 센서 계층 | BEST_EFFORT, VOLATILE, depth 5. 프레임 `front_camera_optical_frame` |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 센서 계층 | JPEG |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 센서 계층 | 미보정이면 K/D/R/P가 0 |
| `/scan` | `sensor_msgs/LaserScan` | 센서 계층 | 프레임 `rplidar_link` (base_link 기준 yaw π, 지면 0.125 m) |
| `/odom` | `nav_msgs/Odometry` | 센서 계층 | `odom → base_footprint`, TF도 발행 |
| `/imu_raw` | `sensor_msgs/Imu` | 센서 계층 | `imu_enabled:=true`일 때만. rad/s |
| `/us_sensor/range` | `sensor_msgs/Range` | 센서 계층 | `adc_enabled:=true`일 때만 |
| `/cmd_vel` | `geometry_msgs/Twist` | 기본 준비 게이트(내부 MPC/reactive 명령 전달) | watchdog 입력 |
| `/mpc/predicted_path`, `/mpc/status` | `nav_msgs/Path`, `std_msgs/String`(JSON) | mpc_controller | NMPC 예측 궤적·진단 |
| `/ball_detection` | `geometry_msgs/PointStamped` | ball_detector | `base_link`, 공 중심 |
| `/world_model` | `pinky_soccer_msgs/WorldModel` | world_model_node / fake_world_model | `map` |
| `/behavior_goal` | `pinky_soccer_msgs/BehaviorGoal` | bt_node | |
| `/game/playing` | `std_msgs/Bool` | 사람 (`ros2 topic pub`), 골 시 game_referee(false) | 전역 |
| `/game/kickoff` | `std_msgs/Bool` (transient local, 0.5 s마다 재발행) | game_referee | true면 모든 로봇이 킥오프 자리로 |
| `/game/goal`, `/game/score` | `std_msgs/String` (JSON) | game_referee | 골 이벤트(골대·득점 팀·점수), 점수(latched) |
| `/world_model/markers` | `visualization_msgs/MarkerArray` | world_model_viz | 경기장은 `/world_model/field_markers` (transient_local) |
| `/world_model/topview` | `sensor_msgs/Image` | world_model_topview | bgr8 |
| `/world_model/search_grid` | `nav_msgs/OccupancyGrid` | world_model_viz | RViz Map 표시용 `ball_search_grid` 복사 |
| `/sim/ball_ground_truth` | `geometry_msgs/PoseStamped` | `fake_world_model`, `mujoco_eval_bridge`(평가용) | 프레임 `world`/`map`. 판정·UI 평가 전용 |
| `/ground_truth` (`/<이름>/ground_truth`) | `geometry_msgs/PoseStamped` | MuJoCo 브리지, Gazebo `gazebo_truth_relay` | 프레임 `world`, 로봇 자세. 표시·평가·심판·시뮬 장애물 전용 |
| `/behavior_test/request`, `/behavior_test/status` | `std_msgs/String` (JSON) | 행동 검증 창 ↔ bt_node(`verify_mode`) | 검증 모드 전용. 형식은 `pinky_soccer_bt/verify_mode.hpp` |
| `/behavior_test/models` | `std_msgs/String` (JSON, transient local) | bt_node(`verify_mode`) | 노드 포트/기본값, 실행 가능한 대상 |
| `/sim/set_ball` | `geometry_msgs/Point` | 행동 검증 창(시뮬), game_referee(골 후 센터) | 공을 x, y로 옮김. MuJoCo eval bridge / `gazebo_truth_relay`가 처리 |

카메라 서비스 `/camera/capture_frame` (`pinky_soccer_msgs/srv/CaptureFrame`)는 실물·두 시뮬레이터에서 동일합니다. raw/jpeg 중 요청한 새 프레임 한 장과 같은 시각의 CameraInfo를 반환하며 기존 판단 메시지/토픽은 유지합니다. 상세 계약·저장 CLI는 [CAMERA_CAPTURE.md](../src/pinky_soccer_bringup/CAMERA_CAPTURE.md)를 참고합니다.

보정 UI의 **공 인식 검토**는 경기 `robust_ball.BallImageDetector`, `ball_vision.project_ball`/`annotate_ball`,
`BallFilter`를 재사용합니다. JPEG·CameraInfo·실행 TF로 검토하며 판단 입력이나 주행 명령을 새로 발행하지 않습니다.
미보정 HFOV 가정과 실제 내부 보정 출처를 구분하고, map/odom이 없으면 움직이는 base_link에서 필터를 돌리지 않습니다.

공 검출은 공통 HLS/원호 피팅이며 하단 ROI 좌표를 원본 영상으로 복원한다. 기존 중심 광선/공 반경
평면 투영 함수는 유지한다. 선택한 human_verified 공 XY 보정은 base_link 축으로 투영 후 한 번만
적용한다. UI 비교는 raw 관측에 한 번 적용하며 기존 K/D·장착·보정 원본을 수정하지 않는다.
손/얼굴 전용 분류는 사용자 요청으로 범위 밖이다. `docs/BALL_HLS_DISTANCE.md` 참조.

`calibration_ui -webcam`은 시작 옵션 전용 로컬 리허설입니다. DDS/로봇 구독·주행 발행 없이 같은
기록/보정 함수를 사용하며, 웹캠 전용 rehearsal 프로젝트로 분리합니다. 측정한 바닥 높이/기울기로 정의한
고정 `webcam_floor`에서 공 거리/필터를 검토하며 이를 로봇 TF·odom으로 발행하거나 적용하지 않습니다.

상대 인식은 측정시각 TF/외벽·고정구조 제거/인덱스 연속 군집 평균점과 단일 Alpha-Beta 추적이다.
`world_model.opponents`는 최대1대이며 theta는 실제 방위각이 아니다. UI 원은 고정 외곽 크기,
화살표는 이동 속도 벡터다. 미확정 일반 군집은 별도 `world_model/obstacles`(기존 WorldModel 타입)로
MPC의 기존 snapshot에 전달하여 상대 확정 조건과 회피 입력을 분리한다. 비용/충돌 반경/제약은 유지한다.
`opponents/status`는 구독 시 생성하는 진단이다. 상세·미검증 사항은 [OPPONENT_TRACKING.md](OPPONENT_TRACKING.md).

기본 rectangle launch의 최종 명령/경기 허용/LED는 `match_preparation`이 소유한다.
위치 추정이 일시 무효/지연되면 `RECOVERING`에서 정지·초록 점멸하며 기다리고,
위치가 검증되면 이전 준비/경기 단계로 복귀한다. `localization/recovery` Bool은
오래 상실한 외벽 정합을 기존 좌표계와 일치하는 새 관측3개로 복구하도록 허용한다.
STOP/심판 중지는 자동 복귀를 취소한다. 위치 유효 상태의 경로 막힘은 기존
충돌 검사 하에 약10cm 후진을 우선하고 재계획한다. 위치 무효 상태에서는 이동하지 않는다.
`preparation/prepare`→기존 보정→기존 GoToPose/MotionResult→odom 정지 확인→READY→별도 시작이다.
BT 출력은 `preparation/strategy_goal`로 받아 PLAYING에서만 기존 behavior_goal로 전달한다.
localizer LED는 끄고 보정 출력/50Hz 제어 명령을 최종 준비 게이트로 통과시킨다. World Model과 심판은
승인된 preparation/playing을 본다. 기존 /game/playing은 시작/중지 요청으로 호환한다.
알고리즘/제어 수식/전략 XML/공식 서브모듈은 보존한다. [운영·검증·배포](MATCH_PREPARATION.md).

PART B 공통 정책은 현재 `mpc_controller`의 GoToPose 경로에 연결된다. 제어용 연속 pose와 원본
map pose를 구분하고, 독립 완료 정책/Active-Pending buffer/raw LiDAR 제동 monitor를 사용한다.
MuJoCo·실물 통합은 활성, Gazebo는 기존 경로다. BT의 선택적 `motion/policy`(String JSON, goal_id 결합)와
기존 `mpc/status.motion_state`로 완료 모드/최신 종료 상태를 전달한다. 기존 typed 메시지는 유지한다.
검증된 다음 GoToPose의 실제 인수에서만 STOP을 생략하며, 만료/취소/실패는 정지한다.
solver 비용·제약·물리·주기·watchdog는 유지한다. 신규 제어기는 공통 정책과 timed-trajectory adapter를
연결해야 한다. [PART B 구현/검증과 미검증 범위](CONTROL_PART_B.md).

## 좌표계

- 2026-11-28 실물 rectangle의 기본 방위 입력은 wheel odometry다.
  `localization_heading_source:=odom|imu|auto`로 선택하며 auto는 실물 odom,
  시뮬레이션 imu다. odom에서는 IMU 수신·시간 매칭을 요구하지 않는다.
  LiDAR 두 후보와 초기 골대 색상으로 map 방향을 확정하고, 이후 odom 회전량으로
  예측한 후보를 외벽 정합으로 보정한다. 각 로봇은 자기 위치/카메라 TF에서 골대
  조준각을 계산하고 자기 odom→map 방위 기준을 유지한다. IMU 방식은 선택적으로 보존한다.

- 공통 시작 기본(`localization:=rectangle`, 실물·MuJoCo·Gazebo): 같은 PC `rectangle_localization`이
  외벽 두 후보를 유지하고 최소각 골대를 조준·정지 후 HLS 한 색으로 IMU 양축을 계산한다. 완료 후 map→odom 단독 발행,
  `localization/pose`를 월드 모델의 기존 TF 모드에 전달한다. 이 경로에 AMCL/goal_heading을 중복 실행하지 않는다.
  시작은 센서 수신만 하며 `localization/calibrate` Trigger/R 신호 뒤 PC gate가 보정 회전을 소유한다.
  완료 후 골대 영상처리를 끄고 LiDAR·IMU·odom으로 추적하며 MPC 50 Hz 명령을 준비 게이트까지 전달한다. 최종 이동 허용·팀 LED는 준비 상태 관리자가 결정한다.
  단기 가림은 제한된 odom 예측, 장기 상실은 명시적 재초기화로 처리한다. 원본 scan·MPC solver/안전/주기는 유지한다.
  LED는 기존 로봇 `set_led`/`set_lamp` 서비스만 사용한다. 상세는 `pinky_soccer_world_model/STARTUP_LOCALIZATION.md`.
  `game.launch.py localization:=auto`는 세 환경 rectangle, 명시 lidar/odom_only는 기존 경로다.
  Nav2 비교 옵션은 기존 lidar를 유지한다. Ground Truth는 평가·UI만 사용하며 공통 추정 입력이 아니다.

- `map` = 경기장 좌표계: 원점 경기장 중앙, 바닥 상단 z=0, **+X 파랑 골대 / −X 노랑 골대**, 단위 m·rad.
- 시뮬레이터 `world` 원점 = 경기장 중앙 = `map` 원점.
- TF: `map → odom`(선택 localizer 또는 world_model_node의 odom_only 모드) `→ base_footprint → base_link → … → front_camera_optical_frame`. `localization:=lidar`에서는 AMCL만 map→odom을 발행한다. 센서 전용 launch에는 map이 없다.
- 공격 방향은 `our_goal_color`(yellow|blue) 파라미터로 정한다. 기본값 `yellow`(−X 수비, +X 공격). 코드에서 "+X = 상대 골대"로 가정하지 않는다.
- 경기장 치수·골대 위치는 `pinky_soccer_field`로만 읽는다 (원본 `rcj_field.yaml`). 코드에 하드코딩하지 않는다.
- 제2 경기장 원본은 같은 위치의 `hardboard_field.yaml`(내부 1820×1220mm·외벽 200mm). 시뮬레이션·통합 판단/표시 launch는 `field:=rcj|hardboard`로 선택하고, 월드 모델·두 UI의 `field_spec`을 같은 YAML로 맞춘다. 기본 경기장과 원본 위치는 유지한다.
- 경기장 지도는 대칭이므로 AMCL은 대략적인 초기 자세가 필요하다. `goal_heading`이 실제 골대 색/방위와 보정된 지자기를 반복 확인해 기준을 기록하고, 명확한180° 별칭이면 X/Y까지 대칭인 후보로 `/initialpose` 재초기화한다. 입자 필터 직접 자기장 융합이나 wheel+gyro EKF는 아직 아니며, 자세/지자기 단절·불확실성 보호와 상세 절차는 `src/pinky_soccer_world_model/LOCALIZATION.md` 기준.

## 패키지

| 패키지 | 역할 | 언어 |
|---|---|---|
| `pinky_soccer_field` | 경기장 치수·기하 로더, AMCL용 지도 생성 | Python |
| `pinky_soccer_msgs` | TrackedObject, WorldModel, BehaviorGoal | msg |
| `pinky_soccer_world_model` | ball_detector, world_model_node, fake_world_model | Python |
| `pinky_soccer_dynamics` | 바퀴 단위 동역학 모델, 보정 파라미터(출처 표시), `/cmd_vel` 명령 변환 | Python |
| `pinky_soccer_mpc` | acados NMPC: 목표 자세 이동 제어기 `mpc_controller`(경기 기본), 검증 도구 | Python |
| `pinky_soccer_bt` | bt_node, BT 노드, `trees/*.xml` | C++ |
| `pinky_soccer` | reactive_controller (기존 축구 제어 패키지) | Python |
| `pinky_soccer_viz` | world_model_viz, world_model_topview, 런치 | Python |
| `BehaviorTree.ROS2` | git submodule — 수정 금지 | C++ |

## 보호 영역

1. `src/pinky_pro/`, `src/BehaviorTree.ROS2/`(submodule) 안의 파일과 submodule 포인터는 사람의 지시 없이 바꾸지 않는다. 확장은 상속·wrapper로 한다. 라이브러리 수정이 꼭 필요하면 멈추고 사람에게 알린다.
2. `refs/`는 읽기 전용 참고 코드다 (git·colcon 제외, `.ignore`로 검색은 허용). 코드를 통째로 복사하지 않고 구조와 아이디어만 참고한다. B-Human은 자체 라이선스가 있다.
3. `.agents/`, `AGENTS.md`, `CLAUDE.local.md`는 커밋하지 않는다.
4. 커밋 전 `.githooks/pre-commit`이 위 규칙과 대용량 파일(10 MB 초과)을 검사한다. `--no-verify`로 우회하지 않는다. 클론 후 `git config core.hooksPath .githooks`.

## 참고 코드에서 볼 곳

| 목적 | 위치 |
|---|---|
| Striker/Goalkeeper 트리 구조, 경기 상태 처리 | `refs/robocup_demo/src/brain/` |
| 공 칼만 필터, 마찰 기반 예측 | `refs/BHumanCodeRelease/` Modeling 아래 Ball 관련 모듈 |
| 관측 영역 기록 그리드 (ball search) | `refs/BHumanCodeRelease/` Modeling 아래 FieldCoverage |

## 사람에게 확인받는 것

- 보호 영역 관련 변경이 필요해 보일 때
- `pinky_soccer_msgs` 필드를 바꿀 때 (모든 층에 영향)
- 기존 패키지(`pinky_soccer_description`, `_bringup`, `_sim`, `_calibration`)를 고칠 때
- 경기장 치수 원본(`rcj_field.yaml`)의 위치를 옮길 때


MuJoCo 공격 경기의 `sim_ball_reset`은 심판 종료 이벤트 후 모든 경기 로봇의 준비/정지
상태를 확인하고 `/sim/set_ball`로 중앙 배치만 수행하는 시뮬레이션 운영 노드다.
로봇 제어·World Model에는 참 위치를 입력하지 않으며 경기 START는 사람이 입력한다.
공 굴림저항 override와 창 옵션은 `attack.launch.py` 범위에만 적용한다.


공 하단의 방향-only 영상 관측은 거리 필터에 가짜 점을 만들지 않고
`world_model/ball_contact`를 통해 BT에 전달한다. `referee_ball_assist` opt-in 시
로봇의 거리 관측 단절을 유효한 영상 심판 공 위치로 보조할 수 있다(사용자 2026-11-28 허용).
Ground Truth는 여전히 평가 전용이다. 반복 시험의 자동 START는 시험 스크립트에만 있다.

`ball_source:=referee`는 사용자 요청(2026-11-28)에 따른 영상 심판 공 위치 전용 모드다.
심판의 유효한 XY를 World Model에 그대로 전달하며 로봇 공 관측/추가 위치 예측을 섞지 않는다.
관측 소실/만료는 LOST로 출력한다. 자기 위치 추정 및 기존 메시지 형식은 유지한다.
