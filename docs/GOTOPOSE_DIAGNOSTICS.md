# 실물 GoToPose 전진·정지 반복 진단

후속 변경: 같은 날짜 사용자 요청으로 실물 τ 적용을 추가했다. [모터 응답 보정 기록](GOTOPOSE_MOTOR_RESPONSE.md) 참조.
아래의 제어 파일 불변/τ=0.05 설명은 **처음 진단 도구를 작성한 기준 `9cb1d0c`**에 관한 기록이다.
현 실물 기본은 새 스냅샷 τ이며 `identification:=none`으로 그 기준 응답을 복원한다. 시뮬 기본·안전·가중치·주기는 유지한다.

2026-11-26 기준 코드 `9cb1d0c`와 사용자의 실물 구동 성공/전진·감속 반복 보고를 바탕으로 한 절차다.
문제 주행의 bag은 없다(사용자 확인). **실제 증상의 원인은 아직 미확인이다.**
분석기는 오프라인 전용이다. 기존 제어기·가중치·안전 정지·주기·시뮬 설정을 변경하지 않았다.

## 코드에서 확인된 동작

| 항목 | 확인 내용 | 측정으로 확인할 것 |
|---|---|---|
| 100 ms activation | `begin(now)`가 `now+dt`를 예약한다. `take()` 때 결과가 없으면 zero, 늦은 결과는 다음 slot에 재사용하지 않는다. 고정 절대 시간 격자는 아니다. | missed·active_solve_id·cmd·drive·wheel의 동시 변화 |
| jitter | 기본 허용 지연 25 ms. 예약 경계보다 늦은 tick의 `now-due`다. 허용 범위 안이어도 take 시점에 결과가 없으면 누락이다. | `activation_jitter_sim_s`(실물에서는 ROS 시간), status 수신 간격 |
| solver/worker/pickup | `ms`: solver 전체 경로. `acados_ms`: 시도별 native acados 시간 합. `worker_wait_ms`: 제출→worker 진입. `result_pickup_ms`: worker 반환→ROS tick 수거. | solve_id별 최초 보고, wall input age. predictor/스냅샷 처리 비용은 세 값만으로 전부 분해되지 않는다. |
| 누락→zero | tracking 누락 시 last solution 삭제·zero·generation 증가. 3회 연속 실패는 FAILURE. 첫 해 전 INITIALIZING도 zero다. | 도착/STOP/목표 변경/입력 및 피드백 단절로 인한 zero와 구분 |
| PC 50→watchdog 20 Hz | 최신 유한 Twist를 저장해 50 ms마다 재발행하며 0.5초 수신 단절이면 zero다. **20 Hz라는 이유만으로 중간에 zero를 삽입하지 않는다.** | 양 토픽 주기·값·zero 지속 시간·중복 publisher. 짧은 변화는 샘플링으로 사라질 수 있다. |
| 바퀴 피드백 | 공식 raw는 목표 30 Hz·bulk read 전 ROS stamp다. relay는 이름만 바꾸지만, **그 뒤 joint_state_publisher가 20 Hz로 병합하며 stamp를 현재 시각으로 다시 쓴다.** 오른쪽 속도는 원시 역부호다. | raw→relay→최종 joint_states 세 토픽을 함께 측정. 최종 stamp/수신만 fresh여도 실제 모터 읽기는 오래됐을 수 있다. |
| observer | 기본 off에서는 source age/valid 필드가 실제 유효 stamp를 대표하지 않을 수 있다. source 기록은 observer on일 때 수행한다. | 직접 joint header를 함께 분석. 측정을 위해 observer 설정을 바꾸지 않는다. |
| AMCL/WM | 입력 신선도/유효성은 제어를 막을 수 있다. 도착 범위 진입/이탈은 settle/track을 바꾼다. | AMCL/WM(map), TF map→odom, wheel odom(odom)을 따로 확인 |
| 동역학 | real 기본 τ L/R=0.05초·delay=0은 실측 가정. ramp 74.9 rad/s²는 설정값이다. 제어기는 legacy identification 자동 적용을 거부한다. | 해당 실행 profile/params와 drive→wheel 응답. 기하 보정과 응답 식별을 구분 |
| 저속 | 현재 모델에 실측 stiction/deadband가 없다. 공식 변환은 100 RPM 비례 포화·0.229 RPM 단위의 0방향 절삭이다. | 양/음 방향, 지속 입력, 정지 출발과 이동 중 같은 속도, 전압/바닥/전류. 양자화와 물리 마찰을 구분 |

`activation_missed`와 jitter는 다음 이벤트까지 남는 **sticky 값**이다. status 5개가 true라고 5 slot 누락이 아니다.
연속 누락은 flag edge로 구분되지 않는다. 도구는 true 연속구간 수를 **관측 가능한 하한**으로만 보고하고 정확한 slot 수는 `null`로 남긴다.
취소된 generation의 worker 결과는 진단 갱신에서 제외된다. 완료된 solve만 빠르다고 deadline 문제를 배제하면 안 된다.
기존 진단만으로 제출/완료/예약/적용의 모든 이벤트를 복원할 수 없다. 정확한 분해가 필요하면 후속으로 별도 opt-in 이벤트 진단
(steady/ROS 시각·generation·solve_id·due·offer/take 결과)을 설계하고 기록 부하를 기준 시뮬과 검증해야 한다. 이번에는 제어 경로에 추가하지 않았다.

피드백 재발행 구조는 `robot.launch.py` 및 설치된 `joint_state_publisher.py`의 `source_cb`/`timer_callback`에서 확인했다.
최종 joint_states의 20 Hz 수신은 하드웨어 20 Hz 측정이라는 뜻이 아니다. upstream 중단 시 마지막 값을 새 stamp로 계속 내보낼 수 있다.
따라서 제어기의 `feedback_age<0.2`만으로 원시 모터 측정의 신선도를 보장할 수 없다. 이는 **확인된 관측 경로의 한계이며 현재 증상의 원인으로 확정한 것은 아니다.**

## 기록 준비와 수집

증상이 발생한 **기존 실행 인자를 그대로 유지**한다. 아래 명령은 기록만 하며 로봇을 움직이지 않는다.
원래 launch 명령, PC/로봇 각각의 HEAD·배포 파일, profile 복사본, 바닥·전압·관찰 시각을 함께 보관한다.
시간동기화 상태도 보관한다. 미동기화 상태에서도 receipt 비교는 가능하나 절대 센서 지연을 주장할 수 없다.

기존 ROS domain/DDS 설정을 유지한 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
source /home/cse/pinky-soccer/install/setup.bash
# 이름이 없는 로봇은 ns='', robot_name:=yellow면 ns='/yellow'.
ns=''
run_dir="/home/cse/pinky-soccer/ros_gotopose_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
ros2 node list > "$run_dir/nodes.txt"
ros2 param dump "${ns}/mpc_controller" > "$run_dir/mpc_params.yaml"
ros2 topic info -v "${ns}/cmd_vel" > "$run_dir/cmd_publishers.txt"
ros2 topic info -v "${ns}/drive/cmd_vel" > "$run_dir/drive_publishers.txt"
ros2 bag record -o "$run_dir/bag" \
  "${ns}/cmd_vel" "${ns}/drive/cmd_vel" "${ns}/joint_states" \
  "${ns}/wheel_joint_states_raw" "${ns}/wheel_joint_states" \
  "${ns}/odom" "${ns}/amcl_pose" "${ns}/world_model" \
  "${ns}/behavior_goal" "${ns}/motion/result" "${ns}/mpc/status" \
  "${ns}/tf" "${ns}/tf_static" \
  "${ns}/motor/diagnostics" "${ns}/motor/info" \
  "${ns}/calibration/time_sync_status" /game/playing /game/kickoff
# 평소 조작으로 증상을 기록하고 정지 후 Ctrl-C로 bag을 닫는다.
```

독자적인 `command_topic` remap·관절 이름·namespace는 실제 이름에 맞춘다.
`ros2 topic info -v`로 recorder QoS와 실제 수신도 확인한다. 수신 0을 곧바로 센서 고장으로 해석하지 않는다.
`motor/*`는 이미 진단 드라이버를 쓰는 경우만 나온다. 이번 기준 측정을 위해 드라이버나 feedback rate를 바꾸지 않는다.
한 PC의 bag에 기록된 Twist에는 고유 stamp/연번이 없으므로 동일 명령 재송신의 전달 지연은 유일하게 식별할 수 없다.
필요하면 후속 비교 실험에서 PC/로봇 동시 기록·시계 확인·기존 비구동 Ping-Pong/진단 경로를 사용한다.

## 오프라인 분석

```bash
source /opt/ros/jazzy/setup.bash
source /home/cse/pinky-soccer/install/setup.bash
MPLCONFIGDIR=/tmp/pinky-mpl-diag python3 scripts/analyze_gotopose_bag.py \
  --bag /absolute/path/to/bag \
  --output /absolute/path/to/new_analysis \
  --target real --context '실제 launch 인자; 바닥; 전압; 증상 시각'
# 이름이 있으면: --namespace yellow
# 실행 당시 profile: --profile /absolute/path/to/profile.yaml
# 응답 보정 전/원복 주행: --identification none (현재 실물 기본은 새 τ 스냅샷)
# 원시 피드백으로 별도 응답 비교: --motor-feedback-role raw_joints
# remap: --topic cmd=/actual/cmd_vel --topic joints=/actual/joints
# 시계 정합을 별도로 확인한 경우에만: --clock-synchronized
```

SQLite bag 디렉터리, 단일 `.db3`(read-only 연결, metadata 불필요), rosbag2 MCAP을 지원한다.
실행 검증은 SQLite 디렉터리/단일 파일로 수행했다. MCAP은 rosbag2 저장소 플러그인 경로를 사용하며 이번에는 실제 MCAP 입력으로 검증하지 않았다.
같은 메시지 타입을 source한 ROS 환경과 `numpy`/`matplotlib`가 필요하다. Agg로 그리므로 화면은 필요 없다.
새 출력 폴더만 받으며 bag이나 기존 결과를 덮어쓰거나 재색인하지 않는다.

- `timeline.png`: cmd/drive/odom 속도, 바퀴 실측과 모델 예측, solver/worker/pickup, sticky missed/jitter, stamp/feedback age.
- `timing.png`: 토픽별 수신 간격, 상태 나이, 제어기 state.
- `feedback.png`: raw→relay→최종 joint_states의 원시 부호 속도·수신 간격·stamp age.
- `localization.png`: frame별 x/y, 인접 샘플의 위치·최단 yaw 변화. 거리 step은 샘플 간격에도 영향을 받는다.
- `motor_scatter.png`: drive에서 복원한 wheel 명령과 정규화한 실측 속도. 폐루프 자료이며 마찰 식별 결과가 아니다.
- `samples.csv`: 재표본화 없이 선택 메시지의 bag 시각·source stamp·데이터 보존.
- `summary.json`/`report.md`: 누락 토픽, 주기/age 분포, 완료 solve별 시간, zero 시작과 근접 status, 진단 driver read/write 시간, 위치 step 통계.

source stamp를 임의로 이동하지 않는다. receipt-source는 시계 오프셋·수집/읽기/전송/큐 지연을 포함한다.
`--clock-synchronized`는 확인 기록이며 수치를 보정하지 않는다. 시뮬 bag의 ROS 시간과 recorder 시간이 다른 경우 age를 실제 지연으로 해석하지 않는다.
`analysis_model`은 `--target`/명시 profile/`--identification`에서 읽은 가정으로, bag에서 실제 실행 파라미터를 자동 복원한 값이 아니다.
실물 기본은 현재 MPC와 같은 τ 스냅샷이며 예전 0.05초 기준 bag에는 `--identification none`을 지정한다.
모델 비교는 실측 바퀴에서 매번 초기화한 한 피드백 구간의 ramp+lag 예측이며 구간 내 drive 변화도 적분한다.
내부 profile state는 관측할 수 없어 초기 0.5초를 통계에서 제외하고 입력/피드백이 0.2초 넘게 끊기면 다시 초기화한다.
수신 시각을 사용하므로 예측 오차에는 전송/시계/샘플링 영향이 남는다. 이 결과로 τ나 dead time을 자동 적합하지 않는다.
drive 기록이 없으면 PC cmd로 대체하지 않고 모델 비교를 결측으로 둔다.

## 증거에 따른 개선안

1. **missed·zero·wheel 감속 일치:** 완료 solve의 wall age/worker/pickup과 status heartbeat 지연을 비교한다.
   이벤트 진단으로 native solver와 Python/GIL/ROS tick 지연을 나눈 후 worker 프로세스 분리·스냅샷 처리 경량화를 검토한다.
2. **PC 명령 연속, drive에 zero/공백:** watchdog 실수신·중복 publisher·DDS/통신을 확인한다.
   20 Hz hold 자체를 고장으로 보거나 bag 수신 순서 차이만으로 watchdog 이상을 단정하지 않는다.
3. **drive 연속, wheel만 실속:** 읽기 주기/stamp·전압/전류·반복성을 확인한다.
   양/음 방향과 정지 출발/이동 중 같은 속도를 비교해 양자화·지연·stiction 후보를 구분한다.
4. **AMCL/WM 보정과 state 변화 일치:** TF map→odom/initialpose/goal·피드백·가시성을 확인한다.
   AMCL·WM·wheel odom은 독립 위치 정답이 아니다. 서로 다른 frame의 x/y를 그대로 빼지 않는다.
5. **타이밍/추정은 안정, 응답 오차 재현:** 같은 바닥/전압에서 독립 step/hold/제동 시험을 하고
   학습/검증을 분리해 τ·ramp·지연·저속 구간을 식별한다. 자동 적용하지 않는다.
6. **raw 정체인데 최종 joint_states만 fresh:** 원시 stamp를 보존한 피드백 경로와 source 기반 신선도 검사안을 검토한다.
   실물 경로만 별도 설정으로 바꾸는 후보이며, 시뮬의 기존 구독/동작을 보존해 비교한다. 이번에는 변경하지 않았다.

이번에는 개선안을 적용하지 않는다. 향후 변경은 기준 HEAD/profile/조건을 고정한 같은 경로의 시뮬 A/B
(성공률·시간·overshoot·missed·접촉)와 실물 재측정으로 평가한다.

## 검증 범위

합성 missed→zero 자료와 기존 **실물 바퀴 보정** bag으로 CLI·그래프·결측 표시를 검증한다.
합성 그래프와 과거 보정 그래프는 이번 증상의 측정 결과가 아니다.
기준 HEAD와 주요 제어 파일 SHA256은 각 `summary.json`의 `code_baseline`에 남긴다.

2026-11-26 검증: 도구/기존 schedule·observer·profile·dynamics 회귀 합계 **51 tests PASS**(프로젝트 `.venv`).
시스템 Python에는 CasADi가 없어 기존 symbolic 3건이 실패했으며, 의존성이 있는 `.venv`에서 전체 선택 검사를 통과했다.
기존 실물 보정 bag `raw_bag_ff8b07229e14485dbcc373e2d1de4e78`에서 2,811개를 오류 없이 읽었다.
이 자료는 odom 약29.97 Hz·최종 joints 약20.00 Hz이며 최대 수신 공백은 joints 373.27 ms였다.
MPC status·drive·원시 wheel이 없고 현재 문제 주행도 아니므로 이 공백을 현재 증상의 원인으로 해석하지 않는다.
합성 자료 2,407개로 missed/zero/모델/위치/타이밍 그래프를 확인했다. 제어 파일 6개의 SHA256이 기준 HEAD와 동일함도 확인했다.
