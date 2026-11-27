# PART B — 공통 위치 제어 요구사항 구현

2026-11-27 [codex]. 범위는 추정 연속성, 행동 완료/인계, 궤적 전환과 독립 안전 감시다.
기존 acados MPC의 비용함수·동역학·구동 한계·100 ms 최적화/50 Hz 명령·watchdog는 유지한다.
신규 LQR/Backstepping 제어기나 경기 전략은 이번에 구현하지 않았다.

## 실제 적용 구조

`motion_management.py`의 상태/완료/버퍼 정책과 `collision_monitor.py`는 ROS·MPC solver에
의존하지 않는 공통 코드다. `motion_runtime.py`가 현재 MPC/GoToPose에 연결한다.
다른 위치 제어기를 추가할 때도 동일 정책을 사용하되 후보 생성·재검증 adapter를 연결해야 한다.
기존 다른 Skill의 동작과 공·상대 인식, World Model, 원본 센서·TF는 변경하지 않는다.

통합 `robot_stack`은 MuJoCo/실물에서 `motion_management=true`, Gazebo에서는 false다.
Gazebo 전용 코드 업데이트/추가 시험 및 실물 배포·주행은 하지 않았다.
노드를 단독 실행할 때의 기본값은 false라 기존 실행과 호환된다.

## 1. 추정 노이즈에 강인한 제어

- 바퀴/odom으로 연속 이동을 예측하고, 유효한 IMU 각속도가 있으면 실제 장착 TF로 변환해 yaw 예측에 사용한다.
- World Model의 절대 map pose로 SE(2) 혁신량을 보정한다. 작은 관측 흔들림은 완화하고,
  일정한 절대 오차는 계속 보정하므로 전역 오차가 영구적으로 남지 않는다.
- 목표와 후보 궤적도 모두 map 기준이다. 제어용 pose만 별도로 유지하며 원본 World Model/TF를 덮어쓰지 않는다.
- 25 cm/0.7 rad 이상 단절은 정상 보정으로 흡수하지 않고 제어를 정지한다. 일관된 새 관측 3개로 회복한다.
- 오래된 source stamp·역순 측정·clock 단절을 처리한다. 원본 pose는 도착 및 안전 검사에 함께 사용한다.
- 작은 같은-ID 목표 변경은 히스테리시스로 유지하되 0.3 s 이상 안정적으로 유지되는 변경은 반영한다.
- 목표 부근 반복 명령 부호 반전과 무진전을 감지한다. 다음 행동이 준비되지 않았으면 실패로 정지하며,
  시간이 지났다는 이유로 성공하지 않는다. 기존 4 s 무진전 보호도 유지한다.

연속 보정 기본 τ=0.12 s, 최대 보정0.20 m/s·0.8 rad/s, 제동 모델 a_dec=0.3 m/s² 등은
**초기 설계값이며 실물 측정으로 확정한 수치가 아니다.** 과도한 필터로 원본 장애물 반응을 늦추지 않는다.

## 2. 완료 판정과 다음 행동 인계

제어 목표와 solver 정밀도는 유지한다. 기존 `Arrival`와 새 완료 정책은 분리되어 있다.

| 모드 | 동작 |
|---|---|
| LEGACY | 기존 Arrival/명령 경로 사용. 공통 추정·버퍼·monitor 적용을 해당 GoToPose에서 우회하는 비교 모드 |
| STRICT_STOP | 기존 위치/각도 허용치와 정지0.3 s 유지. 영역 내 정지 예측을 확인해 제동하고 실제 정지를 확인 |
| PRACTICAL_STOP | 명시된 task 위치/각도 허용치 안에서 정지. 기본2 cm/3° 유지, 기본 정지 확인0.1 s |
| PASS_THROUGH | 경유점에 접근할 때 다음 목표의 궤적을 미리 준비. 통과 방향/횡오차/방위/실제 통과와 준비 완료를 모두 확인 |
| HANDOFF | task 위치/방위 조건을 만족하고 다음 목표의 terminal-feasible 궤적이 실제 검증·활성화된 경우 인계 |

정지 완료는 바퀴≤0.2 rad/s, |v|≤0.01 m/s, |ω|≤0.05 rad/s 및 fresh feedback를 요구한다.
5 mm/1°의 제동 latch 히스테리시스로 작은 노이즈에 제동/재출발이 반복되지 않게 한다.
현재 위치만 보고 급히 제동하지 않고, 모터 상태에서 예측한 zero-input 정지 위치/방위도 허용치 안인지 확인한다.
완료 순간에는 원본과 제어 pose 모두 선언한 허용치 안에 있어야 한다. 일괄 허용 오차 완화는 하지 않는다.

BT GoToPose ports:

- `completion_mode`: DEFAULT(통합 기본 STRICT_STOP), LEGACY, STRICT_STOP, PRACTICAL_STOP, PASS_THROUGH, HANDOFF
- `task_position`, `task_heading`, `task_dwell`: PRACTICAL/인계 task 조건(m, rad, s)
- `next_ready`, `next_x`, `next_y`, `next_theta`: 다음 GoToPose의 실행 계약

새 ROS 메시지 타입/필드 대신 기존 BehaviorGoal/MotionResult/WorldModel을 유지한다.
BT는 ID가 결합된 `motion/policy` String(JSON)을 목표와 함께 전달한다.
고급 completion ports는 BT XML/ROS 검증 요청에서 선택한다. 기존 목표/회피 UI 구성은 유지했다.
종료 실제 상태(x,y,yaw,v,ω,stamp,confidence,오차,이유)는 기존 `mpc/status.motion_state`에 제공한다.
다음 제어는 이상적인 이전 목표가 아닌 최신 예측/관측 상태에서 시작한다.

`next_ready=true`만으로 성공하지 않는다. 같은 모델·실제 현재 상태로 다음 궤적의 종단/입력/회피를
독립 replay하고 적용 시점에 준비된 경우에만 `handoff_ready`를 반환한다.
BT는 이 이유가 있는 PASS/HANDOFF 성공에서만 자동 STOP을 생략한다. 취소/실패/halt는 정지한다.
다음 GoToPose의 목표/회피 옵션이 일치해야 기존 제어권을 인수하고, 0.2 s 안에 인수되지 않으면 정지한다.
드리블·슛 등 임의 Skill의 실행 가능 조건은 아직 연결하지 않았다. 현재 실제 인계는 GoToPose→GoToPose다.
현재 준비된 timed-trajectory adapter는 acados다. SLSQP에서는 stop 모드는 사용할 수 있지만,
PASS/HANDOFF 요청은 adapter가 없다는 실패를 반환한다. 새 제어기에는 이 adapter 계약을 별도로 연결해야 한다.

예시: 기존 BT의 Sequence 안에서 사용한다.

```xml
<GoToPose x="-0.2" y="0" theta="0" completion_mode="PASS_THROUGH"
          next_ready="true" next_x="0.1" next_y="0" next_theta="0"/>
<GoToPose x="0.1" y="0" theta="0" completion_mode="STRICT_STOP"/>
```

## 3. 궤적 연속성과 실시간 안전

- Active/Pending은 목표 버전과 ROS/monotonic 시각으로 관리한다. 취소·오래된 결과는 적용하지 않는다.
- 새 후보는 실제 현재 모터 상태와 마지막 **실제 적용 입력**에서 독립 재검증 후 activation 경계에서 전환한다.
- 안전하고 도달 시간 이득이 작은 유사 경로/좌우 반전 후보는 기존 궤적을 우선한다. 유의미한 목표 변경,
  큰 추종 오차, 충돌 위험은 히스테리시스보다 우선한다. MPC 자체의 10 Hz 제어 최적화는 계속 실행한다.
- 새 계산 대기 중에는 검증된 잔여 wheel 입력만 최대0.3 s 사용한다. 새 입력이 없다고 무조건 이전 첫 입력을
  유지하지 않는다. stale sensor·invalid plan·3회 연속 누락·만료 시 기존 보호 또는 정지를 적용한다.
- 입력 변경은 예정된 slot에만 수행해 predictor가 가정한 held input과 맞춘다.
- 위치/방위/실제 바퀴 속도의 연결은 공통 ramp 모터 맵을 replay하여 확인한다. 입력 증분/속도/비홀로노믹
  제약과 활성화된 회피를 다시 검사한다. XY 곡선을 보간해 실행 가능하다고 가정하지 않는다.
  별도의 C2/jerk 연속성 또는 v≈0에서 곡률 연속성을 증명한 것은 아니다.
- `CollisionMonitor`는 planner 결과를 기다리지 않고 50 Hz 명령 경로에서 원본 LaserScan과 동적 상대 정보를 검사한다.
  상대 확정 여부와 관계없이 raw 장애물에 반응한다. 전략적 회피 OFF와 하드웨어 보호를 구분한다.
- scan 시각의 정확한 TF를 사용한다. TF가 늦게 오면 최신 scan 한 장만 대기해 재시도하며 latest TF로 대체하지 않는다.
  0.3 s 이상 오래된/유효하지 않은 안전 데이터는 정지한다. 원본 scan은 수정하지 않는다.
- `v*tau + v²/(2*a_dec) + margin`, 모델의 delay/제동 sweep, 회전 footprint, 접근 속도 및 위치 불확실성을 고려한다.
  위험에 따라 재계획 요청·속도 제한·정지를 적용한다. sigma>5 cm는 감시 범위 부족으로 정지한다.
- 보호가 입력을 바꾸면 오래된 pending generation을 취소하고 실제 적용 입력으로 다음 예측을 시작한다.
- 제동 예측은 공통5 ms 모터 맵과 수치 일치하는 vectorized ramp/lfilter로 계산한다.
  별도의 무거운 Python 예측 루프로 solver worker를 방해하지 않도록 했다.

## 실행 및 검증

일반 실행은 기존 launch를 유지한다. 경기 준비 신호 전에는 기존 준비 게이트가 이동을 차단한다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco verify:=true preparation:=false
```

현재 환경은 MPC를 `.venv` Python으로 빌드해야 한다. 시스템 Python 빌드는 casadi를 찾지 못한다.

```bash
.venv/bin/python -m colcon build --symlink-install --packages-select pinky_soccer_mpc
colcon build --symlink-install --packages-select pinky_soccer_bt pinky_soccer_viz
```

기존 검증 요청의 `ports`에 위 completion ports를 사용할 수 있다. LEGACY 비교는 `completion_mode=LEGACY`다.
`scripts/run_control_baseline.py --completion-mode STRICT_STOP|PRACTICAL_STOP|LEGACY`와
`scripts/verify_part_b_transition.py --mode PASS_THROUGH|HANDOFF --output NEW_DIR`는 MuJoCo 자동 시험이다.
후자는 격리 도메인226에서 실제 BT 두 GoToPose를 실행하고 인계 전후 명령을 검사한다.

구체적인 검증 수치는 `validation/control_part_b_20261127/results.json`과 tests 로그를 참조한다.
실물·Gazebo·모든 동적 회피 상황의 회귀 검증이나 임의 Skill 인계를 완료했다고 보고하지 않는다.
기존 MPC의 모든 soft deadline 문제를 이 작업으로 해결한 것도 아니다.

## 변경 파일과 한계

추가: `_mpc/motion_management.py`, `motion_runtime.py`, `collision_monitor.py`, 두 정책/adapter 테스트,
`scripts/verify_part_b_transition.py`와 본 문서.
수정: `_mpc/mpc_controller.py`·package.xml/README, BT의 `bt_nodes.hpp/.cpp`·`verify_mode.cpp`·catalog/테스트,
`_viz/robot_stack.py`, 기존 baseline 실행 도구의 completion 선택 인자, README/아키텍처 문서.
공식 submodule·물리/경기장·카메라·보정 원본·MPC solver 비용/제약 소스는 수정하지 않았다.

실물에서 필터/불확실성·제동 모델의 초기값 및 센서 지연을 검증해야 한다.
이번 기능 시험은 제어기 간 순위나 평균 성능 개선율의 근거가 아니다.
다양한 동적 장애물과 센서 노이즈의 실물 분포, 완전한 경기 전략 조합은 추가 검증 대상이다.
일부 launch SIGINT 종료 중 rclpy cleanup 예외 로그가 남으며 주행 중 실패와 구분해 원시 로그를 보존했다.
