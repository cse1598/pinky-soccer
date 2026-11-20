# Pinky Soccer 판단 계층 구조

이 문서는 월드 모델·행동 트리·제어·UI 계층의 구조와 규칙이다.
에이전트 협업 규칙은 `.agents/RULES.md`(로컬), 센서 계약은 `src/pinky_soccer_sim/config/interface.yaml`, 경기장은 `src/pinky_soccer_sim/FIELD.md`가 기준이다.

## 전제

- PinkLAB Pinky Pro(차동구동) 1대1 자율 축구, ROS 2 Jazzy.
- 센서: 로봇 자체 카메라(공·상대), 2D 라이다, 휠 odom, IMU, 초음파(보조). 천장 카메라 없음. 시야가 좁아 **월드 모델**이 핵심이다.
- 공: 주황색 골프공 (기본값 공인 규격 지름 42.67 mm, 45.9 g. 실제 공은 실측 후 조정).

## 5단 구조 + UI

```
1. 관측   /camera/image_raw · /scan · /odom · /imu_raw · /us_sensor/range   (기존 센서 계약)
          ball_detector → /ball_detection (PointStamped, base_link)
2. 추정   AMCL(경기장 지도, 예정) + robot_localization(EKF, 예정) + world_model_node
            → /world_model    (pinky_soccer_msgs/WorldModel, map, ~30 Hz)
3. 판단   bt_node (BehaviorTree.CPP v4 + BehaviorTree.ROS2)
            → /behavior_goal  (pinky_soccer_msgs/BehaviorGoal, 10–20 Hz)
4. 계획   Nav2(원거리, 예정) + reactive_controller(pinky_soccer 패키지)
            → /cmd_vel
5. 제어   command_watchdog(0.5 s) → drive/cmd_vel
          실물: pinky_soccer_bringup / 시뮬레이션: pinky_soccer_sim (둘 다 같은 watchdog 사용)

UI       world_model_viz     → /world_model/markers (MarkerArray, RViz2)
         world_model_topview → /world_model/topview (Image, OpenCV)
입력     /game/playing (std_msgs/Bool, 임시 경기 시작/정지)
```

원칙
- 행동 트리는 센서를 직접 구독하지 않고 `/world_model`만 본다.
- 트리는 **목표만** 바꾸고, 따라가기는 계획·제어 층이 연속적으로 처리한다.
- 월드 모델은 각 객체의 관측/추정/분실 상태와 공분산을 함께 낸다. UI는 관측 = 진한 실선, 추정 = 흐린 점선 + 불확실성 타원, 분실 = 마지막 위치 X로 일관되게 그린다.
- `/clock`, `/ground_truth`, `/sim/*`(`/sim/ball_ground_truth` 포함)는 시뮬레이터 전용이다. 판단 입력으로 쓰지 않는다 (UI 평가 표시와 시험 판정만 허용).

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
| `/cmd_vel` | `geometry_msgs/Twist` | reactive_controller | watchdog 입력 |
| `/ball_detection` | `geometry_msgs/PointStamped` | ball_detector | `base_link`, 공 중심 |
| `/world_model` | `pinky_soccer_msgs/WorldModel` | world_model_node / fake_world_model | `map` |
| `/behavior_goal` | `pinky_soccer_msgs/BehaviorGoal` | bt_node | |
| `/game/playing` | `std_msgs/Bool` | 사람 (`ros2 topic pub`) | |
| `/world_model/markers` | `visualization_msgs/MarkerArray` | world_model_viz | 경기장은 `/world_model/field_markers` (transient_local) |
| `/world_model/topview` | `sensor_msgs/Image` | world_model_topview | bgr8 |
| `/world_model/search_grid` | `nav_msgs/OccupancyGrid` | world_model_viz | RViz Map 표시용 `ball_search_grid` 복사 |
| `/sim/ball_ground_truth` | `geometry_msgs/PoseStamped` | `fake_world_model`, `mujoco_eval_bridge`(평가용) | 프레임 `world`/`map`. 판정·UI 평가 전용 |
| `/ground_truth` | `geometry_msgs/PoseStamped` | MuJoCo 전용 | 프레임 `world`, **로봇 자세만**. Gazebo 브리지에는 없음 |

## 좌표계

- `map` = 경기장 좌표계: 원점 경기장 중앙, 바닥 상단 z=0, **+X 파랑 골대 / −X 노랑 골대**, 단위 m·rad.
- 시뮬레이터 `world` 원점 = 경기장 중앙 = `map` 원점.
- TF: `map → odom`(AMCL 또는 world_model_node의 odom_only 모드) `→ base_footprint → base_link → … → front_camera_optical_frame`. 2026-11-21 기준 센서 계층에는 `map` 프레임이 없다.
- 공격 방향은 `our_goal_color`(yellow|blue) 파라미터로 정한다. 기본값 `yellow`(−X 수비, +X 공격). 코드에서 "+X = 상대 골대"로 가정하지 않는다.
- 경기장 치수·골대 위치는 `pinky_soccer_field`로만 읽는다 (원본 `rcj_field.yaml`). 코드에 하드코딩하지 않는다.
- 경기장 지도는 대칭이므로 AMCL은 초기 자세가 필요하다.

## 패키지

| 패키지 | 역할 | 언어 |
|---|---|---|
| `pinky_soccer_field` | 경기장 치수·기하 로더, AMCL용 지도 생성 | Python |
| `pinky_soccer_msgs` | TrackedObject, WorldModel, BehaviorGoal | msg |
| `pinky_soccer_world_model` | ball_detector, world_model_node, fake_world_model | Python |
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
