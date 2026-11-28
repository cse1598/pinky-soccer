# MuJoCo 두 로봇 3점 선승 반복 개선 — 2026-11-28

> 이 문서는 과거 6판의 기록이다. 2026-11-28 사용자 요청으로 이후 행동 개선을 롤백했다. 현재 10cm 후진 복구 구성의 경기 검증 결과가 아니다. [현재 구성](../../ATTACK_STRATEGIES.md).

현재 main `4597ae0` 위의 미커밋 코드에서 6판을 진행했다. 앞의 3판은 실패했고,
수정 후 최근 3판은 **노랑:파랑 3:0, 3:1, 3:0**으로 완주했다.
최종 판의 세 번째 골은 시뮬레이션 시각 **87.92초**다. 영상/GUI를 직접 열어 판정하지 않고
센서 기반 World Model, 동작/준비 상태, 명령 및 상부 RGB 영상 심판의 토픽을 기록했다.

이는 동일 조건의 충분한 반복 통계나 실전 합격을 뜻하지 않는다. 모든 판은
`field:=hardboard`, 두 공격 로봇, `strategy:=legacy`, `ball_source:=hybrid`,
`localization_heading_source:=odom`, headless, ROS domain193이다. 실물/SSH/RealSense/Gazebo는 사용하지 않았다.

## 판별 결과

| 판 | 변경/실행 조건 | 팀 점수 노랑:파랑 | 결과 | 효율 보조 점수 합(v2) |
|---|---|---|---|---:|
| 01 | CarryBall 포함 기준 코드 | 1:1 | 240 wall초 제한, 실패 | -280.22 |
| 02 | A* 측면 우회·가시 선행점·벽 접근 수정 1차 | 2:0 | 한 랠리 75 sim초 초과, 실패 | -281.48 |
| 03 | 외벽/경계선 분리·한쪽 경합 후퇴 | 0:1 | 추가 득점 없이 130 sim초 이상/이탈 반복, 수동 실패 종료 | -326.95 |
| 04 | 공 옆 이탈 판정·공 중심 조향 | **3:0** | **완주**, 143.13 sim초 마지막 이벤트 | -194.03 |
| 05 | 반대 팀(B) 기준 실행·측면 후퇴 부호 수정 | **3:1** | **완주**, 190.42 sim초 | -243.31 |
| 06 | 같은 B 기준, 먼 공 접근의 연속 곡선 추종 | **3:0** | **완주**, 87.92 sim초·아웃0 | -75.33 |

`result.json.scores`는 기존 계약의 **[노란 골대 진입수, 파란 골대 진입수]**라 팀 점수와
순서가 반대다. `summary.json`은 `team_score_yellow_blue`로 명시한다.
03은 수동 실패 종료이며 reward는 종료 직전 마지막 토픽 표본이다(파일에 명시).
01의 wall 제한과 나머지 판의 조건/길이가 다르므로 점수 합만으로 향상률을 주장하지 않는다.
평가식은 기존 [v2 정의](../../ATTACK_MATCH_ITERATION.md)를 유지했다. 실제 강화학습은 하지 않았다.

## 원인과 반영

1. **벽 근처 목표/완료 판정 불일치**: 공 뒤 목표 Y를 공 반대편으로 반사하면서
   뒤에 섰는지는 원래 골대 축으로 판정했다. 도착해도 밀기로 전환하지 않는 원인이었다.
   반사를 없애고 접근과 밀기가 같은, 실제 몸체가 들어갈 수 있는 임시 방향을 사용한다.
   경기장 YAML의 외벽(하드보드 ±0.91/±0.61m)과 경기선(±0.83/±0.53m)을 구분한다.
   몸체는 기존 `FOOTPRINT_RECTS`에서 읽는다. 물리/규격을 바꾸지 않았다.
2. **각진 측면 우회**: `attack_only.xml`도 `planner_approach=true`로 A*가 공 주변 우회를
   담당하게 했다. 별도의 직각 측면 목표를 먼저 쫓지 않는다. 0.14m 선행점을 사용하되
   로봇에서 그 점까지 보이는 충돌 없는 구간만 허용한다.
3. **원형 벽 여유의 과대 추정**: GoBehindBall 벽 계획만 실제 반너비+기존 여유를 사용할 수 있다.
   상대/일반 장애물의 기존 원형 팽창은 유지하고, 매 명령의 정확한 회전 몸체·제동 replay가
   최종 허용 여부를 결정한다. 벽 충돌 검사를 제거한 것이 아니다.
4. **정면 경합 재현**: 둘 다 같은 시점에 후퇴하면 동일한 경합이 반복된다.
   공격 끝에서 더 먼 쪽이 먼저 직선10cm 후진→중앙 쪽 측면 재접근을 한다.
   6초 재진입 제한을 둔다. 중앙 동률은 -X 공격 쪽이 먼저 물러난다.
   파란 팀의 측면 부호도 검사해 위쪽 벽 근처에서 벽 쪽으로 재접근하지 않게 했다.
5. **공을 비켜 가는 밀기**: 관측 공이 옆으로6.5cm 이상 또는 앞쪽2cm 미만이면,
   기존 하단 접촉 관측이 없는 경우 재접근한다. 가까운 PushBall은 공에서 슈팅 축으로
   10cm 앞인 가시 점을 향해 조향한다. 골대 직선만 쫓으며 공 옆을 지나가는 것을 줄인다.
   공 검출/거리 보정/필터는 변경하지 않았다.
6. **전진의 불연속 차단**: 먼 GoBehindBall 접근만 기존 cos³ 감속과 회전을 동시에 허용한다.
   공 거리30cm에서 활성화/22cm 안에서 해제해 문턱 노이즈에 의한 반복 전환을 막는다.
   고정 GoToPose, 근접 구간, 후진 탈출, MPC의 기존 피벗 정책은 그대로다.
7. **진단/시험 기록**: `command_scale`과 제동으로 줄인 명령 이유를 기록한다.
   심판 점수 갱신이 마지막 GOAL 콜백보다 먼저 도착해 평가가 누락되는 경쟁 조건도 수정했다.
   최종 드라이버는 점수/이벤트의 일치를 확인한 후 성공 판정하고 STOP 이후 결과를 저장한다.

04의 원래 결과는 세 번째 득점 이벤트 이전에 저장됐다. RefereeState의3:0 및 STOP 중 받은
세 번째 GOAL은 원본 토픽에 있다. `result.before_event_fix.json`을 보존하고 이벤트와 득점 보상만
그 원본에서 복구했음을 최종 `result.json.note`에 명시했다. 05/06에는 네 번째/세 번째 마지막
이벤트와 보상이 정상 저장됐다. 성공 판의 이벤트 수와 누적 득점 합도 대조했다.

## 마지막 판 지표와 한계

- 최종 판의 유효 플레이 시간(양팀 합): 126.49 robot·s.
- 추정 pose 기준 1초를 넘긴 정체 감점 시간: 양팀 합0.46초(약0.36%).
  바퀴 피드백 기반 제어기 탈출은 양팀 합2회. 이것이 모든 순간 정지의 부재를 의미하지 않는다.
- 먼 공(>30cm) GoBehindBall에서 제자리 회전 명령 비율:
  05에서3.18/23.72초=13.4%, 06에서0.12/4.29초=2.8%.
  표본 길이/경기 흐름이 달라 제어 변경만의 일반적인 개선율로 해석하지 않는다.
- 전체 플레이의 제자리 회전 명령은 05 약24.9%, 06 약23.8%다.
  가까운 공 정렬·후진 전환에서는 여전히 회전한다. 각진 동작을 완전히 없앴다고 보고하지 않는다.
- 최종 제어 callback 시간(ms), 노랑: 평균8.39/P95 15.27/최대39.88,
  파랑: 평균8.81/P95 15.63/최대102.19. 엄격한 실시간성/CPU 사용량 벤치마크는 아니다.
- 센서 잡음/실행 스케줄이 달라지는 여러 경기장·초기 자세 성공률은 미검증이다.
  중앙 동률 우선 규칙에 팀 비대칭이 있으며, 팀 간 승률 균형도 검증하지 않았다.
- 실물 주행·벽 접촉 횟수 직접 계측·GUI 클릭/LED 육안·Gazebo는 이번에 검증하지 않았다.
  Ground Truth는 기록/궤적 비교에만 사용했고 제어/판단 입력으로 넣지 않았다.
- MPC 비용/제약/주기, 모터 보정값/물리 한계, 경기장/로봇 물리, 공식 서브모듈은 변경하지 않았다.
  Path MPC 실제 주행 비교는 이번 경기 범위가 아니다.

## 파일과 재실행

핵심 변경: `pinky_soccer_bt/{src/bt_nodes.cpp,src/soccer_logic.cpp,src/bt_node_main.cpp,include/*,trees/attack_only.xml}`,
`pinky_soccer_viz/{robot_stack.py,attack_controller.py,attack_motion.py}` 및 관련 회귀,
`scripts/verify_attack_match.py`. 이전 CarryBall/준비 수정과 같은 공통 코드라 실물 PC도 사용한다.
로봇 하드웨어 코드를 새로 배포할 필요는 없다. PC launch 재시작이 필요하다.

빌드: BT/Viz2패키지 성공. Python51개와 CTest3묶음(geometry/logic15, tree21, catalog3) 통과.
세부 결과는 `python_tests.txt`, `bt_tests.txt`, `build.txt`다.

일반 UI에서 실행(준비 P, 준비 완료 후 시작 G, 정지 S, 재준비 R):

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
source .venv/bin/activate
ros2 launch pinky_soccer_viz attack.launch.py \
  mode:=mujoco field:=hardboard robots:=2 \
  strategy:=legacy opponent_strategy:=legacy ball_source:=hybrid
```

시험을 똑같이 반복하려면 **두 터미널 모두** 위의 source 이후 다음 환경을 설정한다.
실물 domain에서 자동 경기 드라이버를 사용하지 않는다.

```bash
export ROS_DOMAIN_ID=193
export CYCLONEDDS_URI="$(python -c 'from scripts.benchmark_localization import LOCAL_DDS; print(LOCAL_DDS)')"
```

첫 터미널에서 위 launch에 `headless:=true team:=B`를 추가한다. 두 번째 터미널:

```bash
python scripts/verify_attack_match.py \
  --output /tmp/pinky-next-match --limit 600
```

시험 드라이버만 양팀 준비→중앙 공/재개 가능 확인→1초 후 START→3점 STOP을 수행한다.
일반 UI는 수동 START를 유지하며 생산 경기 상태 머신에 자동 3점 종료를 추가한 것은 아니다.
공 중앙 재배치 외에 득점이나 로봇 위치를 인위적으로 만들지 않았다.
시험 launch/자식은 모두 종료했고 STOP을 보냈다.

## 증거

`summary.json`, `comparison.json` 및 각 판 `command.json`, `source.sha256`, `result.json`,
`driver.log`, `metrics.json`(있는 판), `far_approach_metrics.json`에 숫자를 보존했다.
원본 `topics.jsonl`/ROS 로그는 용량 때문에 이 폴더의 `.gitignore`로 git에서 제외하되 로컬에 보존한다.
05/06의 설정은 같은 B 기준이지만 통계적 대조 실험은 아니다.

- [마지막 판 궤적·속도·각속도](trial_06/trajectories.png): 추정 pose와 평가용 참 위치,
  영상 심판 공 경로. 이벤트 사이 경로를 끊어 중앙 재배치를 주행처럼 연결하지 않았다.
- [첫 완주 판의 같은 그래프](trial_04/trajectories.png)
