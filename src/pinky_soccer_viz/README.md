# pinky_soccer_viz

실물·Gazebo·MuJoCo 경기 launch는 기본 RViz와 `game_camera_view`를 함께 엽니다.
카메라 창은 원본/실제 공 검출 표시/HSV 마스크를 동기화해 보여주고 수신 중단도 표시합니다.
`headless:=true`이면 GUI를 열지 않습니다. 자세한 사용법은 [GAMEPLAY.md](GAMEPLAY.md).

세 환경 공통 실행은 [GAMEPLAY.md](GAMEPLAY.md)를 참고합니다.
`ros2 launch pinky_soccer_viz game.launch.py mode:=real|gazebo|mujoco field:=hardboard`
에서 mode 하나를 선택하면 전체 경기 프로그램·RViz가 실행되고 `/game/playing`을 기다립니다.
실물 미보정 시험에는 `allow_uncalibrated:=true`를 추가합니다.

실물 경기: [REAL_GAME.md](REAL_GAME.md). PC에서
`ros2 launch pinky_soccer_viz real_game.launch.py field:=hardboard allow_uncalibrated:=true`
로 공 인식·판단·저속 제어를 실행하고 `/game/playing`으로 시작/정지합니다.

월드 모델 UI 두 가지와 판단 계층 전체를 띄우는 launch, 시뮬레이션 평가 도구입니다. PC 전용입니다 (로봇에 설치하지 않음).

## 실행

```bash
# 로봇·시뮬레이터 없이: fake_world_model + bt_node + reactive_controller + 두 UI + RViz2 + rqt_image_view
ros2 launch pinky_soccer_viz sim_viz.launch.py
ros2 launch pinky_soccer_viz sim_viz.launch.py gui:=false            # 화면 없이 토픽만

# 시뮬레이터 + RCJ 경기장 + 전체 판단 계층 (use_sim_time)
ros2 launch pinky_soccer_viz sim_field.launch.py                     # MuJoCo + 뷰어 + RViz2
ros2 launch pinky_soccer_viz sim_field.launch.py simulator:=gazebo
ros2 launch pinky_soccer_viz sim_field.launch.py field:=hardboard     # 제2 하드보드 경기장
ros2 launch pinky_soccer_viz sim_field.launch.py headless:=true eval_ball_truth:=true
ros2 topic pub --once /game/playing std_msgs/msg/Bool "{data: true}"
```

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `gui` (sim_viz) | `true` | RViz2, rqt_image_view |
| `simulator` (sim_field) | `mujoco` | `mujoco` 또는 `gazebo` |
| `field` (sim_field/sim_viz) | `rcj` | `rcj` 또는 `hardboard`, 물리 월드·월드 모델·두 UI가 같은 YAML 사용 |
| `headless` (sim_field) | `false` | MuJoCo `viewer:=false` / Gazebo `gui:=false`, RViz2 끔 |
| `eval_ball_truth` (sim_field) | `false` | MuJoCo만. 아래 평가용 브리지로 `/sim/ball_ground_truth` 발행 |
| `ball_x`, `ball_y` (sim_field) | 0.3, 0.0 | 공 생성 위치 |
| `our_goal_color` | `yellow` | |
| `snapshot_dir`, `snapshot_period` | 빈 값, 0 | 탑뷰 PNG 저장(초 주기) |

`sim_field.launch.py`는 `pinky_soccer_sim`의 launch를 include하고 `field` 선택을 전달합니다. 그 위에 `ball_detector`, 기본 LiDAR AMCL·골색/지자기 방향 확인, `world_model_node`(TF), `bt_node`, `reactive_controller`, UI를 띄웁니다. `localization:=odom_only`로 기존 방식을 선택할 수 있습니다. 월드 모델과 두 UI의 `field_spec`도 선택한 YAML로 맞춥니다. [위치 추정](../pinky_soccer_world_model/LOCALIZATION.md)·[제2 경기장 제작 치수](../pinky_soccer_sim/HARDBOARD_FIELD.md)를 참고하세요.

## UI 표시 규칙 (두 UI 공통, `style.py`)

| 대상 | OBSERVED | ESTIMATED | LOST |
|---|---|---|---|
| 나 | 실선 로봇 + 방향 화살표 | 위치 공분산 타원 추가 | — |
| 공 | 진한 원 | 흐린 원 + 공분산 타원(2σ) | 마지막 위치에 X |
| 상대 | 진한 박스 | 흐린 박스 + 마지막 관측 후 경과 시간 | 숨김 |

추가: 경기장(`pinky_soccer_field`: 외벽, 흰 선, 노랑/파랑 골대, 페널티 구역, 중앙 원, 중립점), 카메라 시야 부채꼴(`camera_info`의 K로 계산, 없으면 60° 가정), 공 예측 궤적(현재 속도로 1.5초, 점선), `ball_search_grid` 히트맵, `/behavior_goal`의 target(자홍 +)과 `planned_path`, 현재 branch/action. 시뮬레이터 실제 위치(`/ground_truth` 로봇, `/sim/ball_ground_truth` 공)는 회색 점으로 겹칩니다 (`show_ground_truth`, 평가용, 판단 입력 아님).

### world_model_viz (RViz2)

- `/world_model/field_markers` (MarkerArray, `transient_local`, 시작 시 한 번)
- `/world_model/markers` (MarkerArray, `/world_model`마다)
- `/world_model/search_grid` (OccupancyGrid, RViz Map 표시용 복사)
- `/world_model/search_grid_markers` (MarkerArray, 기본 RViz 히트맵). 일부 GPU의 indexed texture 셰이더 오류를 피하도록 셀별 색상 CUBE_LIST 사용. 기존 Map 토픽도 유지.
- `rviz/soccer.rviz`: 고정 프레임 `map`, 위에서 본 직교 뷰.

### world_model_topview (OpenCV)

`/world_model/topview` (`sensor_msgs/Image` bgr8, 10 Hz). `px_per_m`(기본 200). 오른쪽 패널: branch, action, 공 상태, 공 불확실성(2σ 타원 장축 길이 m), 마지막 관측 후 경과 시간, `/world_model` 수신 Hz. 글자는 OpenCV 기본 글꼴이라 영어만 씁니다.

## 평가 도구 (시뮬레이션 전용)

- `mujoco_eval_bridge`: `pinky_soccer_sim`의 MuJoCo 브리지를 그대로 실행하되, 실행 중에 `pinky_mujoco.bridge.Bridge.publish_state`에 공 위치 발행을 덧붙입니다 (`/sim/ball_ground_truth`, PoseStamped, frame `world` = 경기장 중앙). 공식·sim 패키지 파일은 수정하지 않습니다. `sim_field.launch.py eval_ball_truth:=true`가 sim launch의 브리지 Node를 이것으로 바꿉니다(launch_ros 내부 속성 사용 — 버전이 바뀌면 오류 메시지가 납니다).
- `soccer_trial`: `/game/playing` true를 보내고 공 실제 위치가 상대 골대 입구(x > 1.095, |y| < 0.30, 우리 노랑 기준)를 넘는지 sim 시간으로 판정합니다. 결과(골/자책골/시간 초과, 걸린 시간, 분기별 횟수, 공 상태 비율, 2초 간격 궤적)를 JSON으로 저장합니다.

```bash
ros2 launch pinky_soccer_viz sim_field.launch.py headless:=true eval_ball_truth:=true ball_x:=0.5 ball_y:=0.3
ros2 run pinky_soccer_viz soccer_trial --ros-args -p use_sim_time:=true -p timeout_sec:=120.0 -p result_file:=/tmp/trial.json
```

## 테스트

`colcon test --packages-select pinky_soccer_viz`: 세 공 상태의 탑뷰 렌더링, 관측 공 색, 히트맵 표시/마커 좌표·색상, 공분산 타원 축, 경기장·동적 마커 생성과 직렬화 (11개, flake8 포함).

## 한계

- RViz2·rqt 실행 및 OpenGL 초기화, 히트맵 마커 사용 후 indexed texture 셰이더 오류 해소를 확인했습니다. 실제 경기 중 표시의 현장 시각 검증은 필요합니다.
- 페널티 구역의 둥근 모서리는 사각형으로 그립니다.
- 히트맵은 경기장 선 위에도 반투명으로 덮입니다.
