# 공격 전용 경기 실행

`attack.launch.py`는 기존 MPC/LQR 경기 실행과 분리한 실행 경로다. 기존 버전과 보정
원본은 보존한다. 수비는 하지 않으며 공 탐색 → 공 뒤로 접근 → 골대 방향으로 밀기를
반복한다. 이동에는 경량 A* 경로와 기하학적 추종기를 사용한다. 최적화 solver를 실행하지 않는다.

## 경기 절차

1. 실행 직후 IDLE: 센서와 심판 영상만 수신하고 이동하지 않는다.
2. 심판 UI **PREPARE/P**: 기존 LiDAR·골대 색·IMU 초기화를 수행하고 자기 팀 시작 위치로 이동한다.
3. READY: 정지하고 팀 색 LED를 켠다. 시작 위치 기본값은 기존 킥오프 위치 ±0.4m이다.
4. 공을 중앙에 놓고 **START/G**: 심판의 관측·판정 활성화를 확인한 다음 공격 BT를 시작한다.
5. **OUT**: 현 위치에서 정지한다. MuJoCo에서는 두 로봇의 정지가 확인되면 공을 중앙에 자동 배치한다. START를 누르면 재개한다. 실물에서는 사람이 공을 중앙으로 옮긴다.
6. **GOAL**: 두 로봇 모두 자기 시작 위치로 복귀해 READY에서 기다린다. 공을 중앙에 놓고
   다시 START를 눌러야 한다. MuJoCo에서는 두 로봇의 복귀·정지가 확인되면 공을 (0,0)으로 자동 배치한다. 자동 재시작은 하지 않는다.
7. **STOP/S**는 준비·공격·복귀를 중단한다. 초기화 실패·이동 실패는 다음 단계로 넘기지 않는다.

소속 팀은 `team:=Y|B`, 두 실물 로봇은 각 namespace로 실행하고 심판의
`robot_namespaces:=yellow,blue`를 지정한다. 골대 색 판정 결과와 팀 선택은 별개다.
심판 연결이 끊기거나 무효가 되면 주행을 중단한다. 초기화를 완료하지 않으면 시작할 수 없다.

재시작에는 현재 심판에서 공이 중앙 반경 0.15m 안에 보이고, 서비스 `reset_play` 이후
판정이 다시 armed가 되어야 한다. 수신이 늦은 과거 종료 상태를 시작 성공으로 사용하지 않는다.
보이지 않는 공을 아웃으로 판정하지 않는다. 놓친 뒤 경기장 밖에서 다시 보이는 공은
`UNVERIFIED_EXIT`로 정지시키며, 관측하지 못한 골라인 통과를 득점으로 만들지 않는다.

## 이동·공 처리

- 보정된 좌우 바퀴 반지름/간격, 드라이버 변환, 모터 ramp와 실측 τ를 재사용한다.
  실물 τ의 기본 파일은 `pinky_soccer_mpc/config/real_motor_response.yaml`이다.
  해당 패키지의 geometry/보정 로더만 사용하며 MPC 노드·비용함수·solver는 사용하지 않는다.
- 경로는 LiDAR 내부 장애물과 외벽/골대 형상을 피한다. 주행 중 명령은 보정된 모터
  모델로 정지까지 예측하고 실제 footprint로 다시 검사한다. 상대 확정 여부와 무관하게
  유효 LiDAR 포인트를 장애물 검사에 사용한다. 원본 LaserScan은 변경하지 않는다.
- 바퀴 명령은 기존 모터 회전수·가속 한계로 제한한다. 정지 지연 모델에는 τ에 watchdog
  한 주기(50ms)를 더한다. 이 50ms는 보수적인 설계값이지 새 실측 지연값이 아니다.
- 새 추종기는 위치 35mm 이내에서 정지/방향 정렬로 전환한다. 정지 확인은 위치 60mm 이내,
  방향 0.09rad 이내, 저속 상태 0.15초다. 25mm의 위치 여유는 정지 중 추정 노이즈/관성을
  재추종으로 바꾸지 않기 위한 히스테리시스이며 실제 도착 오차 측정과 구분한다.
  기존 MPC의 도착 조건을 수정하지 않는다. 허용 범위를 벗어나면 성공을 보고하지 않는다.
  같은 요청이 완료된 뒤에는 다시 목표로 진입해 전후진 보정하지 않는다.
- 목표 뒤로 가는 복귀 경로는 방향을 한 번 선택해 계획된 후진으로 이동할 수 있다.
  골대 앞에서는 먼저 몸체 축을 따라 벗어난 뒤 경로를 따른다. 원형 계획 경계의 과대평가를
  실제 footprint 검사로 보완하며, collision/정지 검사를 끄지 않는다. 이미 원하는 여유 안으로
  추정되는 경우에도 물리 침범이 없고 기존 여유를 줄이지 않는 탈출만 허용한다.
- 복귀 경로는 중앙 재시작 위치를 비워 둔다. 화면의 로봇 표시 원 지름 10cm를 충돌 반경으로 쓰지 않는다.
- 공이 카메라 사각지대에 들어가면 최대 0.22m/1.25초까지만 밀고 0.16m 후진해 재관측한다.
  이 후진은 관측을 위한 의도된 BT 동작이다. 공을 무기한 잡고 있다고 가정하지 않는다.
- 심판에서 얻은 로봇/공 위치는 로봇의 pose·ball 추정값으로 덮어쓰지 않는다.
  공격 BT 입력은 기존 센서 기반 World Model이다. 시뮬 Ground Truth는 시험 기록에만 사용한다.

## MuJoCo: 상부 카메라 + 기존 영상 심판

새 작업은 `feat/attack-soccer`의 별도 worktree에서 작성했다. 영속 소스 위치:
`~/pinky-soccer/.worktrees/attack` (기존 LQR 브랜치는 변경하지 않음).

```bash
cd ~/pinky-soccer/.worktrees/attack
source /opt/ros/jazzy/setup.bash
source ~/pinky-soccer/install/setup.bash
colcon build --symlink-install --packages-select \
  pinky_soccer_dynamics pinky_soccer_field pinky_soccer_mpc \
  pinky_soccer_world_model pinky_soccer_bt pinky_soccer_referee \
  pinky_soccer_sim pinky_soccer_viz
source install/setup.bash
export ROS_DOMAIN_ID=193
ros2 launch pinky_soccer_viz attack.launch.py mode:=mujoco
```

통합 실행은 MuJoCo 상부 카메라의 `/referee_camera/image_raw`와 CameraInfo를 기존
`pinky_soccer_referee`의 `mode:=topic` 입력에 연결한다. 실물 RealSense나 SSH를 사용하지 않는다.
위 예시는 실물과 분리된 ROS domain 193을 사용한다. 카메라는 1280×720, 최대 10Hz이며 구독자가 없으면 렌더하지 않는다.
정상적인 화면 좌표 보정 및 색 마스크/군집/추적/이벤트 판정을 그대로 거친다.
물체의 참 위치를 영상 검출 결과 대신 넣지 않는다.

상부 카메라는 경기장 중앙 위 2m의 가상 카메라다. 최초 방향 보정 후 **상부 영상에서만
골대 지붕과 그림자를 숨기는 cutaway 표시**를 사용해 골라인 뒤의 공을 볼 수 있게 했다.
물리 모델·골대 충돌·바닥 마찰·로봇 카메라의 조명/위치/해상도는 변경하지 않는다.
기존 MuJoCo 뷰어와 로봇 카메라에는 원래의 지붕/가림이 유지된다. 이 cutaway에서의 검출
성공은 실물 골대 가림 검증을 대신하지 않는다.

MuJoCo 모델 몸체는 회색 재질이므로 이 실행에서만 RGB 검은색 상한을 100으로 지정한다.
실물의 65/채널편차20 설정은 유지한다. 실제 경기장은 흰 선 1.670×1.070m,
MuJoCo는 선택된 경기장 YAML의 흰 선 규격을 사용한다. LiDAR는 계속 외벽 규격을 사용한다.
골대 판정 구간은 선택된 YAML의 골대 폭 ±절반을 사용한다. 실물에서 이 규격과 실제
흰 선·골대 위치가 맞는지는 사용자가 확인해야 한다.

```bash
# 두 로봇: 양쪽 공격 BT + 한 영상 심판. UI의 PREPARE/START가 양쪽 서비스를 호출한다.
ros2 launch pinky_soccer_viz attack.launch.py mode:=mujoco robots:=2

# 움직이지 않는 상대를 장애물로 두기
ros2 launch pinky_soccer_viz attack.launch.py mode:=mujoco robots:=2 opponent_mode:=static

# 화면 없는 로컬 검증
ros2 launch pinky_soccer_viz attack.launch.py mode:=mujoco headless:=true
```

Gazebo 추가 개발은 보류하며 이 새 실행은 real/MuJoCo만 허용한다.

## 실물 실행 (현재 미검증)

핑키의 기존 센서·모터·LED bringup을 먼저 실행한다. PC에서:

```bash
cd ~/pinky-soccer/.worktrees/attack
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=37  # 로봇과 같은 domain (현재 실물 구성)
CALROOT="$HOME/pinky-soccer/ros_20261123_195718_b5b380/exports/6fa01d7d19d64855b878e2613e1d5824"
ros2 launch pinky_soccer_viz attack.launch.py mode:=real team:=Y \
  calibration_profile:="$CALROOT/profile.yaml" \
  camera_yaml:="$CALROOT/camera.yaml" \
  identification:="$PWD/src/pinky_soccer_mpc/config/real_motor_response.yaml"
```

실측 프로필·카메라 YAML 없이 실물 실행을 허용하지 않는다. 위 실행은 PC의 RealSense
심판도 실행한다. 심판을 따로 실행 중이면 `referee_enabled:=false`를 추가하되 별도 심판에도
골대 구간·`stop_on_reacquired_exit:=true`·로봇 namespace를 동일하게 지정한다.
실물용 별도 제어 알고리즘이나 공식 Pinky/BT 서브모듈 수정은 없다.

## 검증과 한계

- 보정 모터 모델 기반 위치/회전/계획된 후진/완료 후 위치 노이즈/장애물 우회·막힌 경로 테스트 통과.
- 심판 규칙·UI 서비스 회귀와 신규 테스트 28개, BT tree 14개 및 기존 logic/catalog 테스트 통과.
- MuJoCo 영상 심판으로 아웃→정지→중앙 재배치→재시작 및 득점→복귀→READY를 확인했다.
  규칙 시험은 `/sim/set_ball`로 관측 가능한 순서로 공을 이동시킨 fixture이며 자율 득점과 구분한다.
- 자율 공격(공 이동 fixture 없음) → 영상 GOAL → 자기 시작 위치 복귀/READY까지 통과했다.
  시작~득점 8.38초, 득점~READY 7.47초(시뮬레이션 시간), 최고 명령 0.267m/s였다.
  복귀 위치 오차는 평가용 참 위치 기준5.8mm였다. 기록된 평면 yaw 오차는 약4.75도였다.
- 정적 상대를 둔 약0.8m 준비 이동 우회가 통과했다. 상대와 최소 중심 거리25.5cm,
  두 로봇 footprint를 덮는 보수적 원 기준 여유7.6cm, 최종 위치 오차7.8mm였다.
- 별도의 이동 상대 GoToPose 시험도 통과했다. 이 시험은 시험 소유 BT만 잠시 중단하고
  기존 목표 인터페이스로 고정 목표를 준 제어 시험이며 자율 공격 시험과 다르다.
  상대가 실제로21cm 이상 움직인 것을 평가용 참 위치 기록으로 확인했다. 최소 중심 거리31.0cm,
  보수적 원 기준 여유13.1cm, 최종 위치 오차6.1mm였다(단일 횡이동 사례).
- 자율 왕복의 제어 처리 시간은 중앙값6.2ms/P95 13.6ms/최대34.9ms였다(상태 발행 전 측정).
  최대값은20ms 타이머 간격을 넘으므로 모든 tick이 정확히50Hz라는 주장은 하지 않는다.
- 상대 추정 속도를 별도 예측 장애물로 사용하는 실험은 정적 상대 앞의 불필요한 정지로
  채택하지 않았다. 최종 구현은 측정 시점 LiDAR 포인트와 관측 변경 시 재계획을 사용한다.
- 실물 사용 불가 지시 이후 SSH/RealSense/실물 주행은 수행하지 않았다. 속도·도착 연속성·
  모터 데드존·골대 가림·상대 접촉과 다양한 공 위치는 실물에서 검증해야 한다.
- 중앙 시작/하나의 정적 경기장 관측 결과만으로 모든 위치에서 자율 득점하거나 무충돌이라고 보장하지 않는다.

검증 도구 `scripts/verify_attack_sim.py`는 ROS domain 193에서만 실행하며 센서·명령·심판 이벤트와
평가용 참 위치를 JSONL로 기록한다. 참 위치는 제어 입력으로 발행하지 않는다.
최종 자료는 `docs/validation/attack_autonomous_4/`, `attack_rules_4/`,
`attack_obstacle_1/`, `attack_moving_2/`이고 요약/그래프는 `attack_summary/`다.
이전 실패 시험도 삭제하지 않았다. 원시 실험 자료는 `docs/validation/`에 로컬로 보존한다. 추가 커밋/푸시는 아직 수행하지 않았다.


## 주요 변경 파일

- `trees/attack_only.xml`, `bt_nodes.hpp/cpp`: 공격/제한된 사각지대 밀기/재관측 후진.
- `attack_motion.py`, `attack_controller.py`: 경로/도착 유지/보정 추종·정지 footprint 검사.
- `attack_match.py`, `match_preparation.py`: 심판 활성화 확인·아웃 정지·득점 복귀·사람의 재시작.
- `overhead_camera.py`, `mujoco_eval_bridge.py`: 심판 전용 상부 RGB 카메라.
- `attack.launch.py`, 공통 launch/robot_stack: 기존 동작을 보존하는 새 실행 선택.
- 심판 launch/node/core/controls: sim 시계·안전 정지용 미확인 이탈 옵션·버튼 게이트.
- `scripts/verify_attack_sim.py`, 관련 테스트: 정지/재시작/득점/이동·회피 검증.

## MuJoCo 경기 편의 설정 (2026-11-28)

- 기본 창은 **심판 탑뷰, 월드 모델 RViz, MuJoCo 뷰어** 세 개다.
  추가 창은 `show_camera:=true`, `show_localization:=true`,
  `show_preparation:=true`로 선택한다. RViz도 `show_rviz:=false`로 끌 수 있다.
  `headless:=true`는 모든 GUI를 끈다. 기존 game launch의 기본 창 구성은 유지한다.
- 공격 launch의 MuJoCo 공 굴림 감속은 `ball_rolling_deceleration:=0.10` (m/s²)이다.
  기존값 0.35보다 쉽게 구른다. 이 값은 사용자 요청에 따른 초기 조정값이며 실측값이 아니다.
  공유 재질 YAML, 바퀴 접지, 공 크기/질량, 실물 및 Gazebo 설정은 유지한다.
  기존 접촉 커널의 독립 MuJoCo 굴림 시험(초기속도 1m/s)에서 4초 후 이동은
  0.35 설정 1.4291m, 0.10 설정 3.2002m였다. 변경 후 전체 자율 경기 성능은 미검증이다.
- `auto_reset_ball:=true`가 기본이다. 시뮬레이션 전용 관리자가 심판의 GOAL/OUT/
  UNVERIFIED_EXIT 이벤트에만 반응한다. GOAL은 모든 경기 로봇의 READY, OUT은 STOPPED/READY를
  기다리며, 새롭고 유효한 위치 상태와 odom의 저속 상태가 0.4초 지속된 뒤 기존 `/sim/set_ball`로
  공을 (0,0)에 한 번 배치한다. 동작 중·오류·오래된 데이터에서는 배치하지 않는다.
  START는 기존 심판의 중앙 공 관측 확인 후 사용자가 눌러야 한다. 자동 시작은 없다.
  이 기능은 실물에서 실행하지 않는다. 끄려면 `auto_reset_ball:=false`를 사용한다.

검증: 새 reset gate와 이동 회귀 13 tests PASS, sim/viz 2개 패키지 빌드 PASS.
분리된 ROS domain 194의 2로봇 상태 fixture에서 GOAL은 양쪽 READY까지,
OUT은 양쪽 정지까지 기다리고 이벤트당 (0,0) 배치를 한 번만 발행함을 확인했다.
실제 MuJoCo 전체 경기에서 자동 재배치 후 버튼 재시작과 GUI 3창의 눈으로 확인은 미검증이다.

목표 지점이 일시적으로 점유됐을 때 즉시 FAILURE/준비 ERROR로 전환하는 처리는 제거했다.
이동 도중 장애물은 기존 A*로 우회하고, 경로를 만들 수 없으면 정지 상태에서 새 관측으로
재계획한다. 물리적 충돌 검사와 기존 이동 시간제한은 유지한다. 장애물이 실제 목표를
계속 차지하면 그 위치에 도착했다고 보고하지 않는다. 이동 회귀11 tests/빌드 PASS,
사용자 현재 장면에서 복귀 재시험은 미실행이다.
정상 READY 후 START [G], ERROR에서 재준비는 RETRY [R] 또는 PREPARE [P]다.


## 양로봇 반복 경기 개선

1초 정체 감시/독립 탈출, 격자 시작점 연결, 벽 근처 접근 목표, 하단 공의 방향 관측,
접촉 전 상대 속도 감속 및 선택적 심판 공 보조는 [ATTACK_MATCH_ITERATION.md](ATTACK_MATCH_ITERATION.md)를 따른다.
이 최신 절차에서는 OUT 이후에도 두 로봇이 시작 위치로 복귀한 뒤 MuJoCo의 공을 중앙 배치한다.
실물의 중앙 배치는 사람이 하고 START는 계속 수동이다. 3점 선승 완료 여부는 개별 판의 결과를 확인한다.

## 공 위치 입력 선택

공 위치를 항상 영상 심판에서 받으려면 `ball_source:=referee`를 지정한다.
실물과 MuJoCo에서 같은 옵션이다. 두 로봇의 자기 위치 추정 방식은 바뀌지 않는다.

```bash
ROS_DOMAIN_ID=193 ros2 launch pinky_soccer_viz attack.launch.py \
  mode:=mujoco robots:=2 ball_source:=referee
```

- `ball_source:=referee`: 유효하고 공이 보이는 `/referee/state`의 XY를 그대로 사용한다.
  로봇 카메라의 거리/방향 입력과 혼합하지 않고 World Model에서 추가 위치 예측을 하지 않는다.
  속도는 심판의 유효한 속도 값만 사용한다. 관측이0.4초 이상 오래됐거나 무효/미검출이면 LOST다.
  로봇 카메라 입력으로 자동 전환하지 않는다.
- `ball_source:=robot`: 로봇 카메라와 기존 필터를 사용한다.
- `ball_source:=assist`: 로봇 카메라 우선, 단절 시 심판 보조.
- 기본 `ball_source:=auto`: 기존 `referee_ball_assist:=true/false` 설정과 호환한다.
  명시적인 `ball_source`가 이전 보조 bool 옵션보다 우선한다.

`world_model/ball_contact` JSON의 `input_mode`와 `metric_source`로 실제 선택을 확인할 수 있다.
옵션 변경은 launch 재실행 시 적용된다. 기존 카메라 거리 보정값과 심판 영상 알고리즘은 수정하지 않는다.
검증: 두 패키지 빌드·launch 인자 확인·전용 로직2 tests·기존 영상/World Model 로직 회귀 PASS.
ROS 노드 통합 테스트2개는 자동 승인 검토 시간 초과(재시도 포함)로 실행하지 못했다.
주행이나 영상을 직접 보는 검증은 수행하지 않았다.
