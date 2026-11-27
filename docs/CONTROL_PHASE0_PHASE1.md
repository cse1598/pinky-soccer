# 이동 제어 개선: Phase 0 및 Phase 1 진단

이 문서는 PART B 구현 **이전** 기준선 기록이다. 이후 [PART B](CONTROL_PART_B.md)에서 실제 제어 동작을
변경했으므로 현재 소스에 대한 이전 진단 전용 AST 동치 검사는 의도적으로 통과하지 않는다.
당시 검사 결과와 구현 스냅샷은 `validation/control_phase0_20261127`에 그대로 보존돼 있다.

2026-11-27 [codex]. 사용자 첨부 명세의 §44에 따라 기존 구현을 보존하고 기본 이동 시험부터 진행한다.
추가 지시로 **MuJoCo를 우선 검증하며 Gazebo 코드 업데이트와 추가 시험은 보류**한다.
현재 단계는 다중 신규 제어기 개발 완료나 실물 버벅임 해결을 의미하지 않는다.

## 목표와 보존 기준

실제 실행 중인 GoToPose 구조를 확인하고, 명령 자체의 불연속과 모터/센서 응답의 불연속을
분리하는 재현·수집·분석 환경을 만든다. 정상인 부분을 다시 구현하지 않는다.
HEAD만으로는 기존 기능을 복원할 수 없다. 작업 시작 때 미커밋 코드가 많았으므로
`validation/control_phase0_20261127/baseline/`에 **실제 dirty 소스/설정 384개**를 저장했다.

- HEAD: `9cb1d0ce4a8b94cb429889d3adc951d8d79acb0b`.
- `source.tar.gz`, 파일별 SHA256 manifest, working/index patch, submodule ID, 의존성 버전,
  요청 원문과 복구 설명을 함께 보존했다. 활성 작업 폴더 위에 덮어쓰지 않는다.
- 원본 캘리브레이션 세션·STL 등 바이너리 자산·native 빌드 산출물은 복제하지 않았다.
  기록된 HEAD/submodule/환경을 별도 checkout에 마련한 후 소스 스냅샷을 올리는 복구 기준이다.
- `capture_control_baseline.py --check`는 아카이브 SHA256 및 384개 내부 파일 검증을 통과했다.
- `preservation.json`은 현재 변경과 원본을 비교한다. MPC에서는 진단문만 제거한 AST가
  원본과 일치하는지 검사한다. 이는 진단의 실행 부하까지 동일하다는 증명은 아니다.

## 현재 코드 분석

| 항목 | 확인한 현재 구현 |
|---|---|
| GoToPose 입출력 | BT가 고유 goal_id의 BehaviorGoal을 발행하고 같은 ID의 MotionResult를 기다림 |
| BT | 기본 15 Hz. 성공/중단 후 STOP 발행. 다음 행동으로 자동 운동 상태 인계 없음 |
| GoToPose backend | 기본 `AcadosTimeOptimalMpc`; 명시 `slsqp` 비교 옵션 유지. 다른 일반 행동은 PoseMpc 경로 |
| 상태/입력 | pose 3 + 실제 바퀴 2 + 모터 ramp 2 + 이전 입력 2 = 9상태, 좌우 wheel 명령 2입력 |
| 동역학 | 공통 ramp + 1차 지연, 수치 replay와 symbolic 모델 일치 검사. 데드존은 모델링되지 않음 |
| 주기 | 10 Hz 최적화 / 50 Hz 명령, ROS clock의 100 ms lookahead activation, 허용 jitter 25 ms |
| horizon | GoToPose는 정수 N=2..80 검색. 일반 `horizon=2, steps=20`과 구분 필요 |
| solver | DISCRETE SQP, HPIPM partial condensing/ROBUST, 최대 SQP/QP 100회, MERIT_BACKTRACKING |
| 예산 | 총 검색 soft budget 75 ms, 후보 timeout `min(22 ms, budget/3.5)` = 약21.43 ms. hard deadline 보장 아님 |
| 비용/검증 | N이 우선, 정규화 effort 0.02/du 0.08 및 terminal 항. 독립 replay로 입력/증분/속도/종단 조건 검증 |
| warm start | 1 worker의 seed 재사용. generation 변경 시 reset; 초기 첫 해 대기에는 동일 goal의 seed 유지 |
| worker | ThreadPoolExecutor 1개, 한 번에 1 pending job. 50 Hz tick에서 완료 결과 pickup |
| activation 누락 | 유효 stage가 없거나 늦으면 해당 slot은 zero. 늦은 generation 폐기, tracking 3회 연속 누락 시 FAILURE |
| 초기 대기 | 첫 명령까지 1 s. 이 구간 누락은 tracking 연속 실패와 별도 |
| 안전 | goal/pose receipt 0.5 s, wheel receipt 0.2 s. watchdog 0.5 s. 무조건 이전 입력 유지 안 함 |
| 도착 | 추정 위치2 cm/방위3°, |v|≤0.01 m/s, |w|≤0.05 rad/s, wheel≤0.2 rad/s, 유효 feedback 0.3 s 지속 |
| 무진전 | `distance + 0.05*heading error`가 4 s간 0.005 이상 개선되지 않으면 FAILURE |
| pose 경로 | 공통 rectangle localizer→map/odom TF→World Model→MPC. GT는 별도 평가 구독만 사용 |
| 명령 경로 | MPC→localization freshness gate→cmd_vel→20 Hz watchdog drive/cmd_vel→모터 |
| 회피 | 기존 선택적 벽/상대/공 회피. 미확정 LiDAR 장애물 별도 입력 유지. 이번 기본 시험은 모두 OFF |

`acados status 2=MAXITER, 3=MINSTEP, 7=TIMEOUT`은 로컬 acados enum 기준이다.
`status != 0`을 곧바로 전체 GoToPose 실패 횟수로 집계하지 않는다.
기존 구현은 조건을 만족하는 nonconverged 후보를 독립 replay 후 제한적으로 사용한다.
반대로 `ok=True`인 결과도 예정된 slot 전에 pickup되지 않으면 사용할 수 없다.

모델 기본값을 loader로 읽은 결과는 `effective_models.json`이다. 실물 빈 profile은
공식 r=0.027 m / b=0.0961 m를 사용한다. 측정된 바퀴 r/b는 profile 지정 없이 자동 선택되지 않는다.
실물 유효 τ는 기존 적용값 90.319/88.945 ms, 시뮬 τ=50 ms는 기존 가정이다.
실측 τ의 held-out 바퀴 속도 RMSE 개선과 실물 폐루프 주행 개선을 동일시하지 않는다.
기존 τ 기록에는 일부 적분 각도 오차가 악화된 구간도 있다. 이번에 보정값을 변경하지 않았다.

## 추가·수정한 기능

- `scripts/capture_control_baseline.py`: dirty 소스/설정/환경 보존 및 무결성 검사.
- `scripts/run_control_baseline.py`: 격리된 시뮬레이터 실행, 고정 입력 또는 기존 BT GoToPose 실행,
  sensor/goal/command/result/solver/TF/CPU 수집. 실물 실행 모드는 없다.
- `scripts/control_baseline_metrics.py`: goal ID별 solver 중복 제거, 초기/중간/종료 zero 구간 분리,
  receipt 주기·source age·종료 오차·source stamp 정렬 pose RMSE 계산.
- `scripts/analyze_control_baseline.py`: 동일 monotonic receipt 축 그래프, CSV/JSON 생성.
- `scripts/verify_control_preservation.py`: 원본 파일 해시와 진단 제거 후 controller AST 비교.
- `pinky_soccer_mpc/timing_trace.py`, `mpc_controller.py`: **진단만 추가**.
  기본 `timing_trace=false`, true일 때 기존 `/mpc/status` JSON에 추가 필드 발행.
  신규 명령 경로·solver/목표/제약/정지 판단은 추가하지 않는다.
- MPC 진단 및 분석 의미 검증 테스트, 본 문서와 README 링크.

## 계측 해석

기존 `activation_missed`와 jitter는 다음 activation까지 반복 발행되는 값이다.
true 메시지 수를 누락 slot 수로 집계하면 안 된다. 기존 `ms`는 취소된 generation을 버린 뒤
남은 결과만 보여 주므로, 문제가 생긴 가장 느린 job이 통계에서 빠질 수 있었다.

새 `timing_trace`는 goal별 slot/init-miss/tracking-miss 누적 계수, 최근 slot의 예정/실제 ROS 시각,
미리 준비된 결과 유무, pending solve ID 및 tick의 monotonic 시각을 기록한다.
완료 job은 generation 취소 여부와 관계없이 worker 시작 대기/실행/pickup 시간을 기록한다.
고정 크기의 최근 slot/job만 보관하므로 제어 루프를 블로킹하는 I/O나 무제한 큐를 추가하지 않는다.

카운터는 최신 수신 시점까지 정확하다. bag이 중간에 끝나면 최종 전체 횟수라고 주장하지 않는다.
job/slot sequence로 반복 발행을 제거하며, 통신 누락이나 종료 직전 pending job 때문에
최근 job 표본이 빠질 가능성은 남는다. canceled job의 wall runtime에는 Python/GIL/OS 대기도 포함된다.
이 기록만으로 순수 native 연산과 GIL 경쟁의 기여도를 완전히 분리할 수는 없다.

모든 시계열 그래프는 recorder의 monotonic receipt 시간이다. source age는 **같은 ROS clock에서**
receipt stamp−header stamp로 계산한다. 벽시계와 시뮬레이션 시계를 빼지 않는다.
pose RMSE는 header stamp 기준 GT 보간이며, 그래프의 receipt 정렬 오차와 구분한다.
world stamp는 World Model 출력 시각이라는 의미까지 포함한다.

watchdog의 drive 출력을 관측하지만 내부 timeout event counter는 없다.
20 Hz 출력이 관측됐다는 이유만으로 timeout이 없었다고 단정하지 않는다.
UI/카메라/인지 노드는 GoToPose 시험에서도 실행하며, raw RGB는 bag에서 제외한다.
카메라 정보·LiDAR·IMU·joints·odom·TF·pose·goal·solver/path/result는 원본 CDR도 저장한다.
TF static/QoS 등 완전 재생용 세션은 아니므로 bag을 실물 주행에 직접 재생하지 않는다.

## 시험 조건과 검증

Python3.12.3 / NumPy1.26.4 / SciPy1.11.4 / CasADi3.8.1 / MuJoCo3.6.0.
`OPENBLAS_NUM_THREADS=1`, localhost 전용 DDS 도메인, headless 하드보드 경기장,
기존 sensor-noise 기본값, robot spawn (−0.4,0), 공은 (0.7,0.45), 회피 OFF.
`preparation=false verify=true`로 기존 localization/BT 검증 경로를 사용한다.
골대 인식 및 센서 기반 초기화가 끝나야 목표를 보낸다. 최종 도구는 MPC parameter 응답과
첫 status까지 대기해 native solver 기동 시간을 이동 성능과 분리한다.
초기 도구로 기록한 시험은 이 추가 barrier가 없으므로 첫 이동 시간에 기동 대기가 포함될 수 있다.

`basic`의 6개 목표는 순차 실행한다. 다음 구간의 실제 시작 Pose를 저장하며,
매번 동일 좌표에서 시작했다고 가정하지 않는다. fresh 단일 case 시험과 구분한다.
초기 spawn yaw가 달라도 초기 골대 조준으로 GoToPose 시작 yaw는 달라질 수 있으므로
`start_estimate`가 실제 제어 시험의 초기 상태 기준이다.

실측 결과 표와 상세 수치는 아래 `validation/control_phase0_20261127/results.json` 및 각 run의
`summary.json`, `episodes.csv`, `motion.png`, `timing.png`를 함께 확인한다.
반복 수가 적고 기록 도구의 개정/진단 출력/기동 barrier가 달라 제어기 우열 또는 개선율은 계산하지 않는다.

대표 그래프: [최종 단독 주행/바퀴 속도](validation/control_phase0_20261127/mujoco_basic_trace_final/motion.png),
[최종 단독 solver/slot/위치/CPU](validation/control_phase0_20261127/mujoco_basic_trace_final/timing.png),
[실패를 포함한 반복의 타이밍](validation/control_phase0_20261127/mujoco_basic_trace/timing.png).


| 기록 | 완료 결과 | 주행 중 zero 구간 수 |
|---|---|---|
| `mujoco_fixed` | 고정 입력 13 s 완료 | 2 (계획된 정지 2회) |
| `mujoco_basic` | 6/6 SUCCESS | 0 |
| `mujoco_long` | 1/1 SUCCESS | 1 |
| `mujoco_yaw07` | 1/1 SUCCESS | 0 |
| `mujoco_long_trace1` | 1/1 SUCCESS | 0 |
| `mujoco_long_trace2` | 1/1 SUCCESS | 0 |
| `mujoco_basic_trace` | 5/6 SUCCESS | 1 |
| `mujoco_basic_repeat_off` | 6/6 SUCCESS | 1 |
| `mujoco_basic_trace_final` | 6/6 SUCCESS | 0 |
| `mujoco_basic_final_off` | 6/6 SUCCESS | 1 |

GoToPose 관측 34건 중33건 성공/1건 실패다. 조건·시작 Pose·기록기 개정이 섞인 관측 합계이며,
성공률 일반화나 개선율/제어기 순위의 근거로 사용하지 않는다. 최종 단독 ON 반복은 slot119개/누락0회다.
최종 OFF 반복에서도 중간 zero 1구간이 있었으므로 ON에만 발생하는 현상은 아니다.
고정 입력의 정지는 시험 명령 자체의 계획된 정지이며 버벅임으로 분류하지 않는다.

### 현재 확인한 문제

1. 초기 MuJoCo 6개 기본 이동은 6/6 성공했지만, 별도 0.9 m 이동에서 약0.1 s의 주행 중 zero와
   `waiting for scheduled solution`/missed flag가 동시에 관측됐다. 단발 성공은 연속성 보장이 아니다.
2. 진단 ON 6개 반복에서는 곡선 이동 중 missed slot 1회 뒤 회복, 다음 reverse 구간에서
   tracking 3회 연속 누락으로 FAILURE가 재현됐다. late 결과 `ok=True`여도 generation 폐기됐다.
   확인된 canceled job은 worker 실행98.26 ms+대기1.27 ms+pickup22.54 ms(곡선),
   실행84.34/92.26 ms(실패 구간)였다. 기존 picked-up 결과 통계만으로는 보이지 않던 정보다.
3. 이 반복의 누락 slot jitter는 부동소수 오차 수준이며 미리 offer된 결과가 없고 worker가 pending이었다.
   따라서 해당 표본은 “준비된 명령이 activation jitter로 버려진 경우”와 구별된다.
   다만 취소된 마지막 pending job의 완료 표본은 수집 종료로 빠질 수 있다.
4. MuJoCo ROS clock은 일부 구간에 wall보다 빠르게 진행했다. source의 기존 bridge는
   누적 deadline에 뒤처지면 catch-up한다. 따라서 100 ms ROS slot이 항상 wall 100 ms 여유를
   주는 것은 아니다. 75 ms soft budget만을 보고 deadline 충족을 단정할 수 없다.
   simulator pacing, worker 연산, pickup 지연의 기여도는 별도로 더 측정해야 한다.
5. 작은 map→odom 보정과 완료 전 감속·0.3 s dwell은 존재한다. GT 허용 위치에 처음 들어온 후
   성공까지의 시간을 전부 불필요한 대기라고 부르지 않는다. 그 구간은 감속/정지 확인도 포함한다.
6. 실물 증상 bag은 없고 실물 재주행은 하지 않았다. MuJoCo에서 발견한 메커니즘을 실물 원인으로
   확정하지 않는다. 데드존/정지마찰/실물 feedback 지연은 미검증이다.

진단을 켜는 것만으로 문제가 해결됐다고 보고하지 않는다. 진단도 CPU/전송 부하를 더하며,
초기 반복에는 별도 분석 그래프 생성 프로세스가 병행된 경우가 있다. 이후 단독 실행 반복을
별도 run으로 보존했다. 고정 입력·반복 주행·진단 OFF/ON 자료를 동일 조건에서 더 비교해야 한다.
실패/timeout/not-ready도 보고서에 남기며 자동 실행기는 이 경우 exit code 2를 반환한다.

최종 관련 테스트는 **91 passed / 3 deselected**(기존 linter 항목 제외)이며,
이전 소스 기준 MPC 테스트는74 passed였다. 전체 실행 로그는 검증 폴더의 `tests.*.txt`에 보존했다.
새 도구/진단 모듈의 flake8 F 검사와 Python compile을 통과했다.
기존 제어기 AST 비교와384개 원본 스냅샷 검사도 통과했다.
실물 주행·전체 경기 전략·새로운 backend 비교가 이 테스트로 검증된 것은 아니다.

## 실행 방법 (MuJoCo)

워크스페이스를 빌드한 환경에서 실행한다. 자동 시험은 기존 노드를 중단하거나 실물 도메인에 연결하지
않고 별도 localhost DDS/GZ partition과 자기 자식 프로세스만 관리한다. output은 새 디렉터리여야 한다.

```bash
cd /home/cse/pinky-soccer
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1

.venv/bin/python scripts/run_control_baseline.py --sim mujoco --mode fixed \
  --bag --domain 225 --output /tmp/mu_fixed_new
.venv/bin/python scripts/run_control_baseline.py --sim mujoco --mode gotopose \
  --case basic --trace --bag --domain 225 --output /tmp/mu_basic_new
.venv/bin/python scripts/run_control_baseline.py --sim mujoco --mode gotopose \
  --case long --trace --bag --domain 225 --output /tmp/mu_long_new

MPLCONFIGDIR=/tmp/pinky-mpl .venv/bin/python scripts/analyze_control_baseline.py /tmp/mu_basic_new
.venv/bin/python scripts/capture_control_baseline.py --check docs/validation/control_phase0_20261127/baseline
.venv/bin/python scripts/verify_control_preservation.py docs/validation/control_phase0_20261127/baseline
```

수동 경기 검증 창: `ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco verify:=true preparation:=false`.
기존 기본 준비 절차는 변경하지 않았다. 일반 경기 launch에서는 준비 신호를 별도로 보내야 한다.
실물/통합 ROS 노드에서 진단만 켜려면 `ros2 param set /mpc_controller timing_trace true`.
이 명령은 이동을 시작하지 않는다. 네임스페이스 로봇은 해당 node 경로를 지정한다.
실물 baseline bag 기록 방법은 기존 [GoToPose 진단 문서](GOTOPOSE_DIAGNOSTICS.md)를 유지한다.
Gazebo 실행/업데이트 및 실물 배포는 이번 단계에서 진행하지 않는다.

## 다음 단계와 진행 조건

| 단계 | 상태 / 진행 조건 |
|---|---|
| Phase0 보존·분석·재현 기반 | 구현·실측 기록 확보. 최신 미커밋 상태까지 포함한 기준선 보존 |
| Phase1 기본 이동 개선 | 진행 중. slot 미준비→zero/실패 재현·취소 계산 계측 확보. 원인별 기여도와 수정 효과 미확정 |
| Phase2 완료/상태/인계 | 미구현. LEGACY만 현재 동작. PRACTICAL/STRICT/PASS_THROUGH/HANDOFF를 구현한 것으로 보고하지 않음 |
| Phase3 회피 개선 | 기존 선택적 회피는 존재, 이번 개선안은 미착수. 기본 이동 안정성부터 검증 |
| Phase4~8 planner/tracking NMPC/LTV-LQR/Block Backstepping | 미착수. 모델·제약·입출력·완료 정책을 공통화한 후 별도 구현/증명/비교 필요 |
| Phase9~11 비교·실물·최종 선정 | 미착수. 실패 포함 반복 시험 및 실물 측정 없이는 추천 순위를 만들지 않음 |

다음 Phase1 작업은 clock 진행률/worker runtime/pickup 각각을 통제한 반복 재현,
동일 조건의 recorder 부하 평가, tracking missed 뒤 cold reset 연쇄의 영향 확인이다.
동일 goal의 seed만 유지하는 방법과 worker 실행 구조 개선을 검토할 수 있지만,
검증 없이 이전 명령을 유지하거나 deadline·가중치·안전 조건을 완화하지 않는다.
고정 입력 및 다양한 실제 시작 자세의 기본 이동이 반복적으로 통과하고 중간 zero/실패가
분류·해소된 뒤 복잡한 신규 backend와 회피 개선 단계로 진행한다.
