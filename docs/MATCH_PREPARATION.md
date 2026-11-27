# 경기 준비·LED 통합 (2026-11-27)

기본 rectangle 경기 launch에 공통 `match_preparation`을 연결했다. 사용자가 준비 신호를 보내면
기존 센서 기반 Localization → 기존 GoToPose 시작 위치 이동 → 실제 odom 정지 확인 → READY 순서로 진행한다.
READY 이후 별도의 시작 신호가 필요하다. 실물·MuJoCo·Gazebo의 준비 코드는 같다.

**검증 범위:** MuJoCo 양 팀의 실제 이동과 시작/중지, 기존 경기 BT 재개를 확인했다.
Gazebo 양 팀은 이미 시작 위치에 있는 경우 절차를 통과했지만, 다른 위치에서의 이동은 기존
`no pose progress for 4 s` 실패로 ERROR에 정지했다. 실제 Pinky/SSH 배포는 미검증이다.
실패를 READY로 바꾸거나 제어·물리를 조정하지 않았다.

## 기존 구조와 변경 범위

- PC `game.launch.py` → real_game/sim_field → 공통 robot_stack으로 인식·World Model·BT·제어기·UI를 실행했다.
- 실물 센서/모터/watchdog는 별도의 로봇 `pinky_soccer_bringup robot.launch.py`, LED는 로봇에서
  `ros2 run pinky_led led_server`로 실행했다. PC에는 pinkylib가 없어 그 하드웨어 서버를 실행할 수 없다.
- 기존 localizer가 보정 회전/명령 게이트와 LED를 함께 소유하고, Localization 완료 시 팀 LED로 바꿨다.
- 현재 기본 흐름에서는 localizer의 LED 클라이언트를 끄고, 상태 관리자가 LED의 유일한 요청자가 된다.
  로봇에서는 기존 공식 LED 서버를 그대로 사용한다. 공식 서브모듈은 수정하지 않았다.

기존 직사각 정합·최소각 골대 선택·HLS·IMU 기준·공/거리/상대 알고리즘은 파일 단위로 보존했다.
새 이동 제어기/Skill은 없다. 준비 관리자는 기존 `BehaviorGoal(action=GoToPose, goal_id)`를 보내고,
기존 제어기의 같은 goal_id `MotionResult`를 사용한다. MPC 비용/제약/안전 반경/주기는 바꾸지 않았다.
World Model의 팀 설정 입력과 BT의 킥오프 위치 입력은 운영 연결을 위해 추가했다. 경기 XML 전략은 그대로다.

## 상태와 소유권

| 상태 | 허용되는 동작 | LED |
|---|---|---|
| IDLE | 센서 수신·준비 대기, 최종 명령0 | OFF |
| CALIBRATING | 기존 localizer의 보정 회전만 | 초록1Hz 점멸 |
| MOVING_TO_START | 준비용 기존 GoToPose만 | 초록1Hz 점멸 |
| READY | 이동 성공+fresh odom 정지0.3초 확인, 시작 대기 | 팀 색 지속 |
| PLAYING | 기존 BT/검증 목표와 제어 명령 전달 | 팀 색 지속 |
| STOPPED | 정지, 준비 완료 이력/유효 위치가 있으면 시작으로 재개 가능 | 팀 색 지속 |
| ERROR | 준비/이동/위치 상실 원인을 표시하고 정지 | 빨강1Hz 점멸 |

처음에는 센서를 받으면서 **zero command로 정지를 유지**한다. 준비 신호 전 map TF를 생성하거나
보정 회전을 시작하지 않는다. 이전의 ‘대기 중 cmd_vel 자체도 무발행’ 조건은 이번 정지 게이트로 확장됐다.

```
BT → preparation/strategy_goal → [PLAYING에서만 통과] → behavior_goal → 기존 제어기
                                                                  ↓
localization/controller_cmd_vel → 기존 localizer 신선도 게이트/보정 회전
  → preparation/localized_cmd_vel → 상태별 명령 게이트 → cmd_vel → 기존 watchdog/모터
```

50Hz 제어 명령은 callback마다 즉시 전달하며 상태 타이머20Hz로 재샘플링하지 않는다.
명령이0.5초 이상 끊기거나 상태가 움직임을 허용하지 않으면0을 보낸다.
World Model의 playing 입력은 상태 관리자의 `preparation/playing`으로 연결한다. 기존 `/game/playing`
true는 시작 요청으로 받을 수 있으나 READY 이력과 유효 Localization 없이는 승인하지 않는다.
심판도 기본 준비 경로에서는 승인된 playing만 본다. 경기 시작/정지를 서비스로 보내면 기존 전역 신호도 전달한다.

준비 중 중복 준비는 거부한다. 초기화 이전/예전 record의 READY와 다른 goal_id의 결과는 무시한다.
Localization 실패/상실·서비스 거부·제어 결과 누락·이동 timeout은 ERROR이며 자동 재시도/임의 위치 확정은 없다.
이동 SUCCESS 뒤에는 즉시 STOP을 보내고 fresh odom의 |v|≤0.01m/s, |w|≤0.05rad/s를0.3초 확인한다.
그 사이 일반 경기 목표는 차단된다. 정지 확인을 위해 새 목표를 반복 생성하지 않는다.

기존 득점 후 kickoff는 준비 완료 이력이 있는 로봇만 같은 GoToPose 시작 위치로 돌아가도록 연결했다.
켜질 때 남아 있는 kickoff 메시지로는 움직이지 않는다. 반복 true를 새 요청으로 재실행하지 않는다.
이 분기는 자동 검사 통과, 실제 득점 후 end-to-end 시험은 미검증이다.
시뮬 lateral/longitudinal 시험 상대도 기본 준비 모드에서는 승인된 경기 시작 전에는 정지한다.
`preparation:=false`의 기존 장애물 시험 모드는 이전 실행 방식을 유지한다.

## 시작 위치·팀·제어 옵션

기본은 기존 `multi_robot.kickoff_pose(team, True)`의 노란팀(-0.4,0,0), 파란팀(+0.4,0,π) m/rad다.
스폰 pose와 준비 목표를 분리했다. **Ground Truth로 준비 위치에 순간이동하지 않는다.**
경기장 YAML·기존 footprint/goal/wall 충돌 검사로 목표의0.02m 여유를 검증한다. 충돌하는 설정은 거부한다.

`preparation_config:=/path/to/params.yaml`로 `config/preparation.yaml` 형식을 전달한다.

- `start_goal_offset=-1`: 기존 킥오프 위치. 0 이상: 자기 골라인에서 경기장 안쪽으로 이격한 거리(m).
- `start_pose=[x,y,yaw]`: 명시적 목표가 필요할 때만 설정(map m/rad). 기본/offset 방식은 상대 골대 방향을 본다.
- `start_avoid_walls=true`, `start_avoid_opponents=true`, `start_avoid_ball=false`.
- `move_timeout=60s`, `preparation_timeout=90s`와 odom 정지 문턱/유지시간 설정 가능.

선택된 기존 controller/backend/속도 제한을 그대로 사용한다. 준비 이동에는 MotionResult를 제공하는
기존 GoToPose 경로가 필요하다. 현재 검증한 것은 MPC/acados다. reactive/Nav2 준비 이동은 미검증이며
결과를 제공하지 않는 제어기에서 임의로 SUCCESS를 만들지 않는다.
기본 rectangle 경로에서 준비 관리자가 동작한다. 명시적인 lidar/odom_only/Nav2 비교 경로는 기존 방식이다.

팀은 launch `team:=Y|B` 또는 준비 창에서 선택한다. IDLE/ERROR/STOPPED에서만 바꿀 수 있다.
변경 시 이전 준비 완료를 무효화하고 IDLE로 돌아간다. World Model의 자기/상대 골대와 BT 킥오프 위치도
같이 갱신한다. 골대에서 관측한 색과 소속 팀은 별개다. 준비 관리자의 team/status가 운영 기준이다.

## LED와 실행 위치

관리자는 PC에서 기존 `pinky_interfaces/SetLed`의 fill 요청만 비동기로 보낸다.
기존 set_lamp 서버만 있으면 SetLamp(mode=1)로도 같은 색을 보낸다. 점멸은 타이머에서 색/OFF를 교대한다.
늦은 LED 응답/오류는 주행 callback을 기다리게 하지 않는다. LED 서비스가 없어도 위치 추정·정지 제어는 유지하고
상태에 unavailable/failed/timeout을 표시한다. 색은 RGB0..255의 green(0,96,0), Y(96,96,0), B(0,0,96), red(96,0,0)이다.

로봇 `ensure_led` 래퍼는 namespace별 프로세스 잠금을 사용한다. 이미 set_led/set_lamp가 있으면 재사용하고,
없을 때에만 기존 `pinky_led led_server`를 시작한다. 공식 드라이버를 복제하거나 PC에서 pinkylib를 호출하지 않는다.
다른 LED 요청자를 별도로 실행하지 않는다. 통합 모드에서 localizer의 LED 제어는 꺼져 있다.

- MuJoCo: **기존 robot_lamp visual mesh**에 전용 emissive material을 부여하고 같은 set_led 서비스를 제공한다.
  geom의 명시 RGBA가 material을 덮지 않도록 처리했다. geom/collision/질량/관성은 추가·변경하지 않는다.
- Gazebo: 고정 녹색/고정 world를 쓰던 공식 시각 플러그인을 런타임 사본에서 제외한다.
  기존 lamp visual에 명시적 material을 부여하고 실제 visual entity ID의 material만 갱신한다.
  [공식 visual_config 처리](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/user_commands/UserCommands.cc),
  [색상 렌더 갱신 경로](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/rendering/RenderUtil.cc)를 사용한다.
- 시뮬 서비스는 공통 fill/clear를 지원한다. 개별 set_pixel 표시는 구현하지 않았다.

실제 하드웨어 서버/원격 SSH는 미검증이다. PC의 LED 상태는 요청/응답 상태이며 물리 광출력의 측정값은 아니다.

## 실행과 UI

PC 빌드:

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --symlink-install --packages-select \
  pinky_soccer_bringup pinky_soccer_sim pinky_soccer_bt pinky_soccer_world_model pinky_soccer_viz
source install/setup.bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco team:=Y
# mode:=gazebo team:=B 도 같은 절차. robots:=2 등 기존 옵션 유지.
```

기존 카메라·Localization·지도 창과 함께 작은 **Pinky 경기 준비** 제어창을 연다.
팀 선택 → 경기 준비 → 상태/시작 위치 확인 → READY에서 활성화되는 경기 시작 → 경기 중지.
준비 재시도는 새 보정 세션이며 실행 중에는 중복 클릭이 비활성이다. ESC는 이 제어창에서 정지다.
기존 Localization 창의 R/S도 준비/정지 서비스로 연결했다. 창을 닫는 것과 경기 정지는 별개다.

headless/터미널(이름을 쓰면 /yellow 같은 namespace를 앞에 붙인다):

```bash
ros2 service call /preparation/prepare std_srvs/srv/Trigger '{}'
ros2 topic echo /preparation/status
# READY 이후에만 성공
ros2 service call /preparation/start std_srvs/srv/Trigger '{}'
ros2 service call /preparation/stop std_srvs/srv/Trigger '{}'
ros2 service call /preparation/retry std_srvs/srv/Trigger '{}'
# false=노랑, true=파랑. 정지 상태에서 변경하면 준비를 다시 해야 함.
ros2 service call /preparation/set_team std_srvs/srv/SetBool '{data: true}'
```

`preparation/status`는 상태·팀·자기 골대·목표·센서/odom·보정 상태/색상·도착·이유·LED 응답을
latched JSON으로 제공한다. `preparation/led`는 원하는 색/점멸과 현재 위상을 제공한다.
`verify:=true`는 기존 단일 행동 검증 모드를 유지하므로 경기 시작 후에도 Main 전략을 실행하지 않는다.
그때는 검증 창의 행동 요청을 사용한다. 기존 직접 보정/비교 도구는 `preparation:=false`로 실행한다.
두 경기 로봇(`opponent_mode:=game`)은 각 namespace에서 준비가 필요하다.

### 실제 Pinky를 한 명령으로 실행

**이번 변경은 핑키 측 자체 bringup 패키지 전달/빌드가 필요하다. 공식 Pinky 소스 수정은 없다.**
새 robot.launch/ensure_led/CMake/package.xml이 들어 있는 `pinky_soccer_bringup`을 핑키의 기존 workspace에 반영한 후,
핑키에서 `source /opt/ros/jazzy/setup.bash`와 기존 workspace 환경을 불러와
`colcon build --symlink-install --packages-select pinky_soccer_bringup`을 수행한다.
기존 센서/카메라/모터 보정 파일은 유지한다. 실제 배포는 아직 수행하지 않았다.

이미 robot bringup을 실행하고 있다면 PC는 attach 모드로 사용한다:

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=real team:=Y
```

센서와 LED 서버까지 한 PC 명령에서 시작하려면 SSH 주소/원격 workspace를 지정한다:

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=real team:=Y robot_name:=yellow \
  robot_host:=pinky@ROBOT_IP robot_workspace:=pinky-soccer \
  robot_profile:=/REMOTE/PATH/profile.yaml \
  calibration_profile:=/PC/PATH/profile.yaml camera_yaml:=/PC/PATH/camera.yaml
```

SSH 로그인/ROS 환경은 미리 설정되어 있어야 한다. 새 도구는 비밀번호 입력/파일 배포를 수행하지 않는다.
ROS_DOMAIN_ID를 전달하고 기존 원격 DDS 설정은 덮어쓰지 않는다. 원격 bringup과 PC의 namespace/보정 프로필이
일치해야 한다. 기존 bringup이 실행 중이면 robot_host를 비워 재사용하고 중복 bringup을 시작하지 않는다.
`robot_host:=local`은 핑키에서 전체 스택을 실행하는 경우다. 원격 child는 해당 launch의 프로세스 그룹만
종료하며 다른 로봇 프로세스에 killall/pkill을 사용하지 않는다. SSH 경로는 실제 연결 미검증이다.

## 검증 결과

| 항목 | 결과 |
|---|---|
| launch 직후/준비 전 센서 수신 | MuJoCo/Gazebo 수신, idle 비영속도 명령0건 |
| 준비 전 경기 시작 | service 응답 false, 상태 IDLE 유지 |
| MuJoCo Y: 초기x=-0.2→시작x=-0.4 | 실제 이동·정지 확인 후 READY, 팀색/시작/STOPPED |
| MuJoCo B: 초기x=+0.2→시작x=+0.4 | 같은 절차 통과, 파랑 LED 응답 |
| MuJoCo 기존 전략 | verify=false에서 시작 후 GoToBlockPoint/StopRobot 목표 확인 |
| READY 대기 | 명시적 시작까지1초 이상, 구간의 비영속도 명령0건 |
| Gazebo Y/B: 이미 시작 위치인 배치 | 보정·도착/정지·READY·시작/정지·팀 LED 서비스 응답 통과 |
| Gazebo 다른 시작 위치 | GoToPose no pose progress for 4 s → ERROR, READY 차단 |
| 중앙 MuJoCo 시험 | Localization ABORTED → ERROR. 초기 공과 겹친 배치였으나 원인은 미확정 |
| 중복 준비/이전 READY/늦은 결과/위치 상실 | 자동 검사 통과, READY로 잘못 진행하지 않음 |
| 득점 kickoff/팀 변경/각50개 명령 전달 | 자동 검사 통과. 실제 득점 E2E는 미검증 |
| LED 비동기 콜백 비용 | MuJoCo B 평균0.0137/P95 0.1326/최대0.2427ms. 해당 부하에서의 측정이며 실물 주기 보증 아님 |
| 공/거리/상대/Localization·제어 보존 | 파일/기존 diff22검사 PASS, 최종 기능 회귀227 PASS(LED 검사 포함, linter2 제외) |
| 준비·LED·원격 명령·기존 경기 도구 | 최종 선택19 PASS, 패키지5개 빌드 PASS |
| 기존 BT CTest | test_soccer_logic/test_soccer_tree 2개 묶음 PASS |

MuJoCo 기존 램프의 다섯 색을 [외부 렌더](validation/preparation_20261127/render/lamp_colors.png)에서 확인했다.
자기 전방 카메라는 색별 픽셀 차이0, 물리 배열8종이 동일했다.
상대 한 대가 있는0.6m 간격 시험에서도 공/골대 HLS 픽셀 수가 색별로 동일했다.
이 배치에서는 LED가 전방 영상에 나타나지 않아 변화가 없었던 것으로, 모든 거리/가림에서의 오검출 부재를 증명하지 않는다.
Gazebo도 [기존 램프의 다섯 색](validation/preparation_20261127/gazebo_render/gazebo_lamp_colors.png)을
검증 카메라로 확인했고 OFF를 포함해 서비스5/5가 승인됐다. 실제 world와 이름 없는 로봇의 실제 SDF 모델명을
사용해 visual ID를 찾는다. 추가 visual/링크는 없다. 초기 서비스 발견/모델 생성 중에는 실패를 반환하며
관리자는 비동기로 재시도한다. 전방 카메라 HLS 픽셀 수는 GREEN/YELLOW/BLUE/RED에서
노랑9·파랑5388·주황2572로 같았고 OFF 초기 프레임은10/5392/2569였다.
초기 프레임 차이의 원인은 분리하지 않았으며 해당 배치에서 새로운 색 오인은 관측하지 않았다.
전 거리/가림/2대 경기에서의 오검출 부재는 미검증이다. [렌더 수치](validation/preparation_20261127/gazebo_render/render.json).

실제 이동/상태/LED 요청 타임라인은 [운영 그래프](validation/preparation_20261127/workflow.png)에 있다.

[실행 원본/요약](validation/preparation_20261127/native/summary.json)에는 성공·실패·보정 실패 시험을 모두 남겼다.
Gzipped JSON/launch log는 평가용이며 인식/제어 입력으로 사용하지 않는다.
실물 주행/LED 광출력·원격 종료·실제 무선 지연·UI 수동 조작·다양한 초기 방향·2대 경기 전체 운영은 미검증이다.
0.01m/s 정지 문턱 등은 실제 odom/휠 정지 노이즈를 확인한 뒤 근거를 갖고 조정한다.

## 변경 파일과 재현

- viz 신규 `preparation_core.py`, `match_preparation.py`, `preparation_view.py`, `remote_robot.py`, config/preparation.yaml.
- viz robot_stack·game/real_game/sim_field launch·localization_view·game_referee·obstacle_driver·setup/package/docs.
- World Model node: 팀 설정 입력만 추가. BT bt_node_main: 준비 시작 Pose를 킥오프 설정에 연결. 경기 XML 불변.
- 자체 bringup: ensure_led.py와 robot.launch/CMake/package. 시뮬: led_visual.py·MuJoCo bridge 시각 연결·Gazebo runtime material/plugin·CMake/package.
- 준비/LED/remote 테스트, evaluate_preparation.py/check_led_render.py/check_gazebo_led.py와 이 보고서/검증 자료.
- 기존 인식/제어 단독 벤치2개는 `preparation:=false`를 명시해 기존 검사 범위를 유지한다.

재현 예:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/evaluate_preparation.py \
  --sim mujoco --strategy --initial-x -0.2 --domain 222 --output /tmp/prep-mu-new
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/evaluate_preparation.py \
  --sim gazebo --initial-yaw -0.12 --domain 223 --output /tmp/prep-gz-new
MUJOCO_GL=egl .venv/bin/python scripts/check_led_render.py \
  --scene ~/.cache/pinky_soccer_mujoco/scene.xml --output /tmp/led-mu-new
.venv/bin/python scripts/check_gazebo_led.py --output /tmp/led-gz-new
```

시뮬 시험은 로컬 DDS/Gazebo partition을 분리한다. Ground Truth는 평가기에서만 읽는다.
