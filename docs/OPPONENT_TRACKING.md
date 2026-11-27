# 경량 상대 인식·추적 (2026-11-27)

공통 LiDAR 경로에 단일 상대 추적과 Alpha-Beta 속도 추정을 추가했다. 상대 UI는 고정 크기 원과
이동 속도 벡터를 사용한다. 공·거리 보정·자기 위치 추정·BT·MPC 비용/제약/반경은 유지했다.
실물 검증은 수행하지 않았다. 아래 수치는 이 시험 조건에서만 유효하다.

## 1. 기존 구현과 문제 범위

기준은 작업 직전 로컬 코드다. [기존 검출 함수](validation/opponents_20261127/baseline_opponents.py)를 보존했다.
`opponents.scan_points → cluster_opponents → world_model_node.tick → WorldModel.opponents` 경로였다.

- wall_margin 0.06m 안쪽의 점만 남겨, 이웃 거리 0.05m·최소3점·최대크기0.30m로 묶고 평균점을 출력했다.
- 무효/벽 포인트를 제거한 뒤 원본 빔 인덱스가 없어졌다. 멀리 떨어진 인덱스의 점도 좌표가 가까우면 연결될 수 있었다.
- 스캔 시작/끝 연결은 실제 360° 여부를 검사하지 않았다. 거리별 빔 간격도 반영하지 않았다.
- 모든 군집을 즉시 표시하고 순서대로 opp_0, opp_1…을 붙였다. 시간적 확정, ID 연관, 속도 추정은 없었다.
- 스캔의 TF 조회 실패 시 최신 TF로 대체했다. 수신 시각을 관측 시각으로 사용했다.
- RViz/OpenCV는 사각형, 검증창은 사각형과 pose.theta 고정 길이 화살표를 사용했다.
- MPC `obstacle_snapshot.take`는 같은 상대 목록의 위치·공분산을 장애물로 사용했다.
  BT `OpponentCloserToBall`도 위치만 사용한다. 상대 theta를 실제 전방 방향으로 사용하는 판단 경로는 확인되지 않았다.

이들은 코드에서 확인한 오연결·ID 불연속 가능 경로다. **사용자가 경험한 실제 오검출의 원인으로
확정하지 않는다.** 해당 실물 스캔 기록이 없고, 이번 정상 경기장 시험의 기존 검출도 빈 경기장에서는
0건이었다. 실물 오검출률 감소를 수치로 주장할 근거는 아직 없다.

## 2. 배경 제거와 O(N) 후보 생성

원본 LaserScan은 수정·재발행하지 않는다. 공통 함수 `indexed_candidates`가 실물/두 시뮬에서 동일하게 실행된다.

1. 유한값·센서 유효 거리 검사, 센서 근접 자기 반사 범위 제외.
2. **측정 header.stamp의 TF**로 map 변환. 최신 TF 대체 없음.
3. 경기장 YAML의 inner_walls/wall_margin과 기존 골대·고정 구조물 footprint로 배경 제외.
   스캔 높이보다 낮은 구조물과 머리 위 goal roof footprint는 수평 스캔 배경으로 사용하지 않는다.
4. 원본 인덱스를 유지하고 이웃 빔만 연결한다. 최대 인덱스 차3(중간 결측2개), 거리 상한은
   `min(0.09, max(0.05, 1.8 * min(range1, range2) * abs(angle_increment) * index_gap))` m.
5. 실제 한 바퀴 스캔에 한해 시작/끝을 같은 조건으로 연결한다.
6. 최소3점, 관측 크기0.015~0.30m. 크기는 평균점에서 최대거리×2인 기존 정의다.
   대표 위치는 **전체 군집의 산술평균**이다. 관측되지 않은 부분/기하학적 중심을 복원하지 않는다.

3점 기준과 기존 wall_margin 0.06m는 유지했다. 가까운 조각은 소수 결측 빔을 가로질러 연결할 수 있으나,
큰 구간을 건너 공간상 가까운 점을 합치지 않는다. Circle/RANSAC/DBSCAN/형상 인식/카메라는 사용하지 않는다.

TF 도착 지연을 위해 최대5스캔·0.3초의 비동기 대기 큐를 사용한다. 30Hz tick에서 재시도하며
콜백을 기다리게 하지 않는다. 만료 스캔은 버리고 새 관측으로 꾸미지 않는다.
진단의 pickup_age_s는 ROS 측정 시각부터 처리 시각까지의 나이이며 센서 전달/큐/TF 대기를 포함한다.
processing_ms는 map TF 확보 후의 처리 구간으로, 첫 TF 조회·큐 대기·발행 tick 대기는 제외한다.
초기 map TF 생성 전의 만료는 정상적으로 집계되며, MPC activation missed와는 다른 지표다.

설정은 `config/world_model.yaml`의 `opponents.*`다. self_radius 0.08m는 **LiDAR 원점 근처
제외 범위의 초기 가정**이다. 로봇 전체 외곽/충돌 반경을 뜻하지 않는다. 실물 장착과 초근접 상대를
확인한 후에만 조정한다. wall_margin을 무조건 늘리는 방식은 적용하지 않았다.

## 3. 단일 상대 추적과 시간

최대8개의 잠정 후보만 유지하고, 서로 다른 최근5개 스캔 중3회 공간적으로 일관되면 하나를 확정한다.
10Hz 스캔에서 연속 관측 시 이론상0.2초다(추가 TF 대기 별도). 확정된 ID는 `opp_0`이며
후보 순서가 바뀌어도 바뀌지 않는다. 재초기화 이후에도 1대1 슬롯 이름은 같다.

확정 후 예측점에 가장 가까운 후보를 선택하되, 마지막 측정점과 예측점 모두에서
`association_slack + max_speed * dt` 이내여야 한다. 초기값0.08m·0.6m/s는 연관 조건이며
자기 로봇의 MPC 속도 한계가 아니다. 연관 실패는 다른 후보로 즉시 전환하지 않는다.

상태는 map의 `(x,y,vx,vy)`이며 측정 stamp 차이로만 dt를 계산한다.

```
predicted = position + velocity * dt
residual = measurement_mean - predicted
position = predicted + alpha * residual
velocity = velocity + beta * residual / dt
```

alpha=0.6, beta=0.06. 속도는 연관 최대속도로 제한하고 미세한 정지 잔차는 감쇠한다.
초기 확정 시 속도는 미확정0으로 시작하며 다음 실제 관측부터 갱신한다. 출력 deadband는0.015m/s다.
동일/과거 stamp는 버린다. World Model 발행·UI 갱신은 읽기 전용 예측이며 재측정으로 입력하지 않는다.

- OBSERVED: 최근 실제 일치 스캔, age≤0.3초. 새로운 스캔에서 놓치면 즉시 ESTIMATED.
- ESTIMATED: 마지막 관측에서 최대 hold_sec=1초 동안 제한 예측·신뢰도 감소.
- 이후: 상대 목록에서 제외(LOST에 해당). UI에 오래된 상대를 남기지 않음.
- 큰 map→odom 보정(위치0.15m/yaw0.35rad), 기존 pose 재초기화, 시뮬 시계 역행 시 추적 초기화.

`pose.theta=0`은 기존 메시지 호환용 값이다. 실제 상대 방위각을 추정하지 않는다.
Beta0.12/0.06/0.03 비교에서0.06은 시험 제자리 회전의 표면 변화에 의한 화살표를 억제하면서
일정 속도 추정 오차를0.03보다 작게 유지했다. **왕복 전환 지연은 남는다.** 소수의 표면 평균만으로
회전에 의한 모든 가짜 병진과 실제 병진을 완벽하게 구분한다고 주장하지 않는다.

## 4. MPC 일반 장애물 보존

상대 확정 대기는 충돌 회피 대기가 아니다. 기존 `scan_points/cluster_opponents` 함수는 그대로 두고,
같은 측정 시각의 점으로 계산한 **미확정 전체 군집**을 별도 `world_model/obstacles`로 전달한다.
기존 `WorldModel` 메시지를 재사용하며 이 토픽은 회피 전용이다. UI/BT는 기존 `world_model`의 단일 상대를 본다.

MPC 변경은 이 후보 토픽 구독과 입력 선택뿐이다. 새 토픽이 최근0.5초 내 수신되면 그 데이터로 기존
`obstacle_snapshot.take`를 호출한다. 토픽 없는 기존 producer/단독 시험에서는 기존 world_model로 돌아간다.
이 fallback의 미수신 상태는 실물 통신 시험에서도 확인해야 한다. 장애물 수 상한·공분산 inflation·
안전 반경·벽 제약·solver·활성화 검사·가중치·주기·GoToPose 옵션은 바꾸지 않았다.
새 배경/최소크기/3-of-5 조건으로 일반 장애물을 지우지 않는다. 회피 입력은 여전히
표면 평균이므로 실제 중심과 차이가 있으며, UI 원을 충돌 반경으로 쓰지 않는다.

기존 ROS 메시지·토픽 계약은 유지한다. 추가 토픽:

| 토픽 | 타입 | 용도 |
|---|---|---|
| world_model/obstacles | 기존 WorldModel | 미확정 전체 군집의 회피 입력 |
| opponents/status | String(JSON) | 측정 stamp, TF 만료, 군집/확정, 포인트 수, 처리 시간 |

진단 JSON은 구독자가 있을 때만 만든다. 전체 scan·영상 디버그를 항상 생성하지 않는다.

## 5. 세 UI의 표시

RViz는 cylinder, OpenCV/Tk는 원을 사용하며 검출 크기로 바뀌지 않는다. 반지름은 기존
`pinky_soccer_mpc.collision_geometry.FOOTPRINT_RECTS`(Pinky chassis/wheel/bumper 외곽)의
가장 먼 모서리 거리에서 계산한 약0.0895m다. 새로운 물리 치수를 추가하지 않았다.
MPC의 기존 상대 장애물 반경0.10m와 별개다. 원 중심은 추적된 관측 표면 평균이며 중심 복원값이 아니다.

세 UI 모두 pose.theta 전방 표시를 하지 않는다. 속도 벡터 화살표는 원의 중심에서 시작하며
map `(vx,vy)` 방향, 길이 `min(scale*speed, max_length)` m다. 화면 Y 반전/미터→픽셀 변환은 기존 경로다.
초기 속도 미확정·정지·LOST에서는 화살표가 없다. ESTIMATED 원은 투명도 또는 점선으로 구분한다.
자기 로봇의 기존 자세 화살표는 유지했다.

세 노드 `world_model_viz`, `world_model_topview`, `behavior_verifier`에서 동일한 설정 이름을 쓴다:

- `opponents.velocity_arrow_scale=0.5` (초)
- `opponents.velocity_arrow_min_speed=0.02` (m/s)
- `opponents.velocity_arrow_max_length=0.35` (m)

속도 숫자 표시는 추가하지 않았다. [실제 TopView 렌더](validation/opponents_20261127/ui.png).

## 6. 검증 결과와 한계

### 합성 센서 비교

같은 경기장 구조물과 사각 상대에 ray casting한 동일 스캔을 이전/현재 검출에 입력했다.
Truth는 스캔 생성/평가만 사용한다. 인식에 로봇 정답 pose/velocity를 넣지 않는다.
각 시나리오100스캔,10Hz,거리 노이즈σ2mm·seed20261127, 360/720빔 시험이다.
중앙/벽/골대 근처, 자기 병진/회전, 상대 회전/전후진, 결측, 잡음, RCJ 빈 경기장을 포함했다.

- 빈 경기장 3종: 이전·현재 모두 오검출0/100. 현재 최대 상대1대. 실제 오검출 재현/감소율은 미검증.
- 상대 시나리오: 첫0.2초 뒤98/100스캔에서 추적 출력. 단기5스캔 가림은 ESTIMATED로 유지/재획득.
- 일정0.06m/s 전진/후진 속도 RMSE 약0.0066/0.0068m/s(초기1.5초 제외).
- 정지·자기 이동/회전 중 정지 상대: 시험 출력 속도0. 제자리 상대 회전의 최대 출력속도0.0177m/s,
  0.02m/s 이상 화살표0회. 다른 형상·실물 노이즈로 일반화하지 않는다.
- 군집 순서·±π 스캔 연결·희소3점·큰 빔 결측·늦은/중복 stamp·불가능 점프·큰 localization 보정은 자동 검사.

처리 시간 평균/P95/최대와 process CPU 시간은 [수치 원본](validation/opponents_20261127/synthetic.json),
[비교 그래프](validation/opponents_20261127/timing_speed.png)에 있다. 일반 시험의 새 검출+추적은
약0.2~0.3ms/scan이며 기존 검출은 약0.5~0.6ms/scan이었다. 이는 알고리즘 단독 비교다.
실제 노드는 회피를 위해 기존 군집 계산도 유지하므로 전체 CPU가 그 비율로 줄었다는 뜻은 아니다.
합성 스캔 생성/plot 시간은 처리 시간에서 제외한다.

### MuJoCo / Gazebo native 센서

별도 localhost DDS domain/Gazebo partition의 headless 검증 모드다. 초기 보정 Trigger만
평가기가 보내고, 상대 정답은 **평가기에서만** 사용했다. 실물 DDS에는 연결하지 않았다.

| 시험 | 결과 |
|---|---|
| MuJoCo 빈 경기장 | 10Hz, 상대 표시0, 미확정 후보0 |
| MuJoCo 정지 상대+자기 GoToPose | 평가 구간 추적100%, 다중표시0, 속도P95=0m/s, 대표 관측6점 |
| MuJoCo 충분한 간격의 이동/회전 | 기존 GoToPose 2/2 SUCCESS(2.79/1.72초), 양 회피 옵션 ON |
| MuJoCo 상대 왕복0.08m/s 설정 | 추적100%, 속도P95=0.0738m/s, 방향 일치91.5%, 속도RMSE=0.0473m/s |
| Gazebo 정지 상대 | 초기화 완료,10Hz, 추적100%, 다중표시0, 속도P95=0m/s, 대표 관측9점 |
| Gazebo 빈 경기장(정렬 시작 배치) | 초기화 완료,10Hz, 상대 표시0, 후보0 |

비율은 기록된 World Model 평가 구간 기준이다. 왕복 지표는 전환/정지 구간을 포함한다.
방향 일치는 truth speed>0.03, estimate speed>0.02m/s 구간에서 dot(v_est,v_truth)>0인 비율이며
정답 속도는±0.25초 창의 최소제곱 기울기로 평가한다. 정답을 필터에 입력하지 않는다.
왕복의 추정 지연은 [시간 그래프](validation/opponents_20261127/native_mujoco_lateral.png)에서 확인한다.
단순 filter의 전환 지연·관측 표면 변화가 남아 있어 동적 속도 정확도를 완성으로 보고하지 않는다.

정지 상대의 축 중심 truth 대비 RMSE는 MuJoCo9.1mm/Gazebo9.2mm, 왕복31.1mm였다.
**표면 평균과 축 중심의 정의 차이까지 포함한 값**이며 중심 복원 정확도가 아니다.
TF 대기 큐 적용 전 MuJoCo 추적 유지율14.6%→적용 후100%를 확인했다.
시작 map TF 생성 전 만료와 실제 처리 stamp는 원본 기록에 있다.

map TF 확보 후 처리(기존 회피 군집+새 추적+map→odom 조회)의 평균/P95/최대는 MuJoCo1.286/2.379/5.507ms,
Gazebo1.614/2.097/2.912ms였다. World Model 전체 CPU는 각각 약21.2/58.9% of one core로
관측했으나 simulation/다른 노드 부하가 통제된 변경 전 CPU 자료가 없어 전후 절감률은 미검증이다.

**미완/실패 기록도 보존했다.** Gazebo 기본 yaw 시작 시험은 기존 초기화가 AIMING_GOAL에서
멈춰 상대 인식 검증에 도달하지 못했다. yaw=-0.12rad의 별도 시작 배치에서 초기화/정지 검증을 했다.
Localization·물리·임계값은 고치지 않았다. Gazebo 주행/상대 이동 속도는 미검증이다.
MuJoCo의 상대 가까운 목표(-0.1,0,1rad) 회전 요청은1회 FAILURE였다. 안전 제약은 완화하지 않았고,
충분한 간격의(-0.25,0,0/1rad) 회귀는2/2 통과했다. 가까운 요청의 상세 실패 원인은 미확인이다.

[Native 요약](validation/opponents_20261127/native/summary.json)과 같은 디렉터리의 gzipped JSON/launch log에
성공·실패·대기 큐 적용 전 시험을 모두 보존했다. 원본은 평가 데이터이며 런타임 입력이 아니다.

### 회귀 검사

- World Model+viz 기능211 PASS, 전역 linter2개 분리 제외. 공 검출/거리/보정·localization/TF·UI 포함.
- 상대·UI·MPC 회피 묶음44 PASS(그중 기존/추가 MPC avoidance9개, acados OFF 경로 동일성 포함).
- 최종 큐/추적/UI 전용25 PASS. 3개 패키지 빌드 PASS. 변경 Python의 F 검사·diff check PASS.
- 최종 UI14 PASS: 원 내부 속도 화살표 대비색, 방향/길이, RViz LOST marker 삭제 포함.
- 전역 스타일 검사를 함께 실행했을 때에는 기능210 PASS, linter2 FAIL이었다. linter가 외부/빌드까지
  검사한 기존 수만 건도 포함되어 전역 lint 통과로 보고하지 않는다.
- [보존 검사27개](validation/opponents_20261127/preservation.json) PASS:
  기존 회피 군집 함수 AST 동일, 이번 범위 밖 기존 tracked diff 동일. 서브모듈/공/KD/보정/물리/BT 변경 없음.

## 7. 파일과 실행

변경: `opponents.py`, `world_model_node.py`, `config/world_model.yaml`, MPC `mpc_controller.py`의
입력 연결, viz `style.py/topview.py/world_model_viz.py/world_model_topview.py/behavior_verifier.py`.
추가: 상대 tracking/node/display 테스트, MPC 입력 회귀, 평가 CLI2개, 이 문서와 검증 자료.
`msgs.py`, 메시지 정의, 공 Kalman Filter, MPC 수식과 물리 정의는 수정하지 않았다.

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --symlink-install \
  --packages-select pinky_soccer_world_model pinky_soccer_viz pinky_soccer_mpc
source install/setup.bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco verify:=true \
  robots:=2 robot_name:=yellow opponent_name:=blue opponent_mode:=lateral
# 시작 위치 보정은 기존 창 R 또는 서비스로 요청한다.
ros2 service call /yellow/localization/calibrate std_srvs/srv/Trigger '{}'
ros2 topic echo /yellow/opponents/status
# Gazebo: mode:=gazebo. 실물: 기존 로봇 bringup + PC mode:=real 및 기존 K/D/profile.
```

이번 변경은 **PC만** 빌드/재시작한다. 핑키 내부 코드·LED·센서 드라이버 교체는 필요하지 않다.
실물은 정지 상대/상대 없음→벽·골대 근처→자기 이동/회전→상대 왕복·가림 순으로 시험한다.
원본 scan, tf/tf_static, localization/pose/status, world_model/obstacles, world_model,
opponents/status와 MPC status를 같은 bag에 기록한다. 실물 scan Hz/stamp/TF 대기·만료와
정지 속도 분포를 확인한 뒤 wall_margin·self_radius·association·filter/UI 문턱만 근거를 갖고 조정한다.
0.02m/s 화살표 기준의 실물 타당성, 초기 탐지 지연, 장기 가림·근접 상대·재초기화는 후속 검증 대상이다.

재현 가능한 격리 평가(새 출력 디렉터리 사용):

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/evaluate_opponents.py --output /tmp/opp-synthetic
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/evaluate_opponents_native.py \
  --sim mujoco --opponent static --drive --duration 30 --domain 219 --output /tmp/opp-mujoco
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/evaluate_opponents_native.py \
  --sim gazebo --opponent static --initial-yaw -0.12 --domain 220 --output /tmp/opp-gazebo
```

평가 스크립트는 실물 모드를 제공하지 않는다. 실제 노드의 input은 scan/TF와 기존 경기장 정의뿐이다.
