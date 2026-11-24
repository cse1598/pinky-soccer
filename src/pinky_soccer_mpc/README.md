# pinky_soccer_mpc

Pinky Pro용 acados NMPC 패키지입니다. 1단계(acados 연결·검증)와 **3단계 목표 자세 이동 NMPC**
(`mpc_controller`, 경기·검증 launch의 기본 제어기)가 있습니다. 장애물 회피(5단계)는 이후 단계입니다.

## 이동 제어기 (`mpc_controller`, `pose_mpc.py`)

`/behavior_goal`의 목표 자세(x, y, θ)와 `/world_model`의 자기 자세를 받아 매 주기(20 Hz) NMPC를 풀고,
첫 바퀴 속도 명령을 명령 변환기(드라이버 자신의 반경·간격)로 `/cmd_vel`에 냅니다.
예측 궤적은 `/mpc/predicted_path`(검증 창의 보라 점선), 상태는 `/mpc/status`(JSON).

- 모델: `pinky_soccer_dynamics` 바퀴 모델, 입력 = 좌우 바퀴 목표 각속도, 지평 1.5 s / 30 단계.
- **비용 (최소 시간형)**: 매 단계 남은 시간 추정 T(목표 쪽으로 회전 + 직진 + 목표 방향으로 회전,
  현재 속도 한계 기준) + 거리의 L1형 항 + 목표 근처에서만 커지는 방향 항 + 작은 가속도 항.
  끝 단계: T, 위치·방향, 정지(바퀴 속도 0).
  - 거리 제곱 비용은 목표에 가까울수록 느려지는(지수 접근) 비례 제어와 같은 성질이라 쓰지 않습니다.
  - T를 끝에만 두면 "지금 돌든 나중에 돌든 같다"가 되어 계속 미루다 멈췄습니다(MuJoCo에서 확인) →
    매 단계에 둠.
- **방향도 목표**: 목표 근처에서 방향 비용이 켜져 목표 방향을 맞추며 들어가고, 정지 판정은
  위치 2 cm·방향 0.05 rad(히스테리시스 2배). BT `GoToPose`도 위치 5 cm와 방향 0.15 rad가 모두
  맞아야 SUCCESS입니다.
- **제약 (제품 사양 90 %)**: 바퀴 |Ω| ≤ 0.9 × 103 rpm = 92.7 rpm(9.71 rad/s),
  |Ω̇| ≤ 0.9 × 74.9 = 67.4 rad/s². 값과 출처는 `pinky_soccer_dynamics/config/dynamics.yaml`의
  `limits`. 103 rpm은 ROBOTIS XL330-M288-T 5 V 무부하 속도(e-Manual)이며, 모터 모델·전압은
  공식 드라이버 MAX_RPM 100에서 추정한 것으로 로봇에서 읽어 확인하지 않았습니다. 매뉴얼에는 물리적
  가속도 한계가 없어, 모터가 실제로 따르는 공식 드라이버 Profile Acceleration 200(74.9 rad/s²)을
  기준으로 했습니다. 선속도·각속도 상한(launch `max_lin`·`max_ang`, BehaviorGoal.max_speed)은
  매 풀이마다 바뀌는 soft 제약입니다.
- **후진**: 순수 이동(GoToPose)은 후진을 허용해 더 빠른 쪽을 고릅니다. 공 관련 행동
  (`forward_only_actions`: GoBehindBall, PushBall, GoToBlockPoint, FaceBall, TurnToLastSeen)은
  카메라가 공을 계속 보도록 v ≥ 0으로 제한합니다. 후진으로 공을 놓쳐 BT가 행동을 계속 바꾸는 것을
  MuJoCo에서 확인했습니다.
- **비볼록 대책**: 처음 풀 때와 0.25 s마다 "회전→직진→회전" 기본 궤적(전진형·후진형)을 초기해로
  넣어 몇 번 풀어 보고 가장 싼 해를 이어 갑니다. 정지 상태에서 목표가 바로 뒤에 있는 경우 등
  국소해에 머무는 것을 막습니다. 풀이 시간은 보통 12~15 ms, 재검사 주기에 최대 약 40 ms입니다.
- **바퀴 속도 상태**: `wheel_feedback:=true`면 `/joint_states`(대상별 부호) 측정값을, 아니면 보낸
  명령과 바퀴 응답 모델로 추정합니다. 개루프 추정만 쓰면 MuJoCo에서 지그재그(±60°)가 생겼습니다.
  시뮬레이션은 기본 true입니다. 실물은 오른쪽 부호가 실측 기록으로 −1임이 보고되었지만
  (`.agents` 공유 메모리, 2026-11-23) 기본 false로 두었습니다.
- **안전**: 목표·자세가 0.5 s 넘게 끊기거나 STOP 목표이거나 솔버가 2회 연속 실패하면 0 명령을
  냅니다. 로봇 쪽 command watchdog는 그대로입니다.

### 왜 느렸나 (이전 `reactive_controller`)

1. **launch 상한**: `game.launch.py` 기본 `max_lin` 0.08 m/s·`max_ang` 0.5 rad/s가 시뮬레이션에도
   그대로 적용되었습니다. 바퀴 한계(0.267 m/s)의 30 %입니다. 이제 `auto`로 바뀌어 실물은 0.08/0.5를
   유지하고(사용자 확인 전 안전값), 시뮬레이션은 바퀴 한계를 씁니다.
2. **비례 제어식** `v = min(k·d, v_max)·cos²(e)`, k = 1.2/s: d < v_max/k ≈ 0.2 m부터 지수적으로 감속해
   (시정수 0.83 s) 마지막 20 cm에 약 1.6 s를 씁니다. 방향 오차 0.5 rad면 cos² 때문에 23 % 감속,
   0.9 rad를 넘으면 멈추고 제자리 회전합니다.
3. **MuJoCo 브리지의 축별 제한** v 0.25 m/s·ω 1.5 rad/s: 실물 드라이버에는 없는 제한입니다
   (실물은 한쪽 바퀴가 100 rpm을 넘을 때만 두 바퀴를 같은 비율로 줄임). 제자리 회전 최대가 실물
   5.9 rad/s 대비 1.5 rad/s였습니다. 이제 `real_driver_limits:=true`(기본)로 실물 방식을 따릅니다.
4. 선속도와 각속도를 따로 제한해 바퀴 한계를 다 쓰지 못했습니다(회전하며 전진할 때 낭비).

바퀴 모델 폐루프 비교(같은 바퀴 모델, 정지→목표 자세 도착, 위치 3 cm·방향 0.08 rad):
비례 제어(상한 0.25 m/s·2 rad/s)는 3.25~5.55 s, NMPC는 2.25~3.10 s (22~58 % 단축).

### 회전 속도 상한 2 rad/s (시뮬레이션 기본)

바퀴 한계만 두면 제자리 회전이 5.4 rad/s까지 올라갑니다. 이 속도에서는 MuJoCo의 LiDAR AMCL과
골대 색·지자기 방향 확인(`goal_heading`)이 180° 대칭 자세로 잘못 재초기화해 위치 오차가 2.3 m까지
났습니다(3회). 2 rad/s에서는 재초기화 0회, 오차 최대 4.7 cm였습니다. 그래서 시뮬레이션 기본
`max_ang`은 2.0이고, `max_ang:=0`이면 상한을 없앱니다. 위치 추정이 빠른 회전을 견디게 되면
올릴 수 있습니다.

## acados 연결 (`acados_setup.py`)

acados는 `external/acados` 서브모듈(커밋 `c15f50a9a`, upstream main과 같음)입니다. acados를 쓰는 코드는
`pinky_soccer_mpc.acados_setup.acados()`로만 불러옵니다. 이 함수가

1. acados 위치를 찾고(`ACADOS_SOURCE_DIR` → 이 파일 위쪽의 `external/acados`) 환경변수를 설정
2. **NumPy 호환**: acados_template(`acados_ocp.py`)이 NumPy 2 전용 별칭 `np.bool`을 씁니다.
   Jazzy의 NumPy 1.26에는 없으므로, 없을 때만 `np.bool = np.bool_`(NumPy 2의 정의와 동일)를 추가합니다.
   upstream main(2026-11-22)에도 아직 그대로이며 setup.py가 NumPy 2를 요구하지도 않습니다.
   서브모듈·NumPy 버전은 바꾸지 않습니다. upstream이 `np.bool_`로 고치면 이 처리는 지웁니다.
3. acados 공유 라이브러리를 미리 적재해 `LD_LIBRARY_PATH` 설정 없이 생성 솔버가 동작

생성 C 코드는 `~/.cache/pinky_soccer_mpc`(또는 `PINKY_MPC_CACHE`)에 만들고 저장소에는 두지 않습니다.

## 검증

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run pinky_soccer_mpc check_acados
```

핑키 바퀴 동역학(`pinky_soccer_dynamics`)으로 점대점 OCP를 생성·컴파일·SQP로 풀고
반환 상태 0, KKT 잔차, 입력 한계, 바퀴 가속 경로 제약, NumPy 모델 재시뮬레이션 일치, 목표 도달을 확인합니다.
`test/test_acados_verify.py`가 같은 검사를 자동으로 합니다(casadi/acados가 없으면 건너뜀).
`test/test_pose_mpc.py`: 바퀴 모델 폐루프로 자세 도착(방향 포함)·시간 상한, 바퀴 속도/가속 90 % 한계,
전진 전용·회전 상한, 목표가 뒤에 있을 때 멈춤 회귀를 확인합니다.

주의: acados의 경로 비선형 제약(`con_h_expr`)은 1..N−1 단계에만 걸립니다. 실제로 로봇에 보내는
첫 입력의 제약은 `con_h_expr_0`/`lh_0`/`uh_0`로 따로 넣어야 합니다(검증에서 실제로 위반을 확인 후 수정).

## 실행

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco verify:=true          # 기본 controller:=mpc
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco controller:=reactive   # 이전 비례 제어
```

실물: `mode:=real`은 상한 0.08 m/s·0.5 rad/s가 기본입니다(필요하면 `max_lin`, `max_ang`로 올림,
최대 0.30 m/s·6 rad/s). `calibration_profile:=<profile.yaml>`이면 실측 반경·간격을 모델에 씁니다.
첫 실물 시험은 사람이 비상정지를 잡고 저속으로 합니다.

## 빌드

casadi·acados_template은 `.venv`에만 있으므로 `.venv` 파이썬으로 빌드합니다.

```bash
.venv/bin/python -m colcon build --symlink-install --packages-select pinky_soccer_dynamics pinky_soccer_mpc
```
