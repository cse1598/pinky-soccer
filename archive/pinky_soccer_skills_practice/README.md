# pinky_soccer_skills

축구 기본 스킬(이동 기본 동작·공 스킬), 스킬을 조합하는 플레이북, ROS 실행 노드 `skill_player`,
ROS 없이 MuJoCo로 스킬을 반복 검증하는 시험대 `skill_bench`.

- 사람이 읽는 스킬 설명서: [SKILLS.md](SKILLS.md) (스킬별 동작 그림·입력·성공/실패 조건·검증 결과)
- 판단 계층 구조: 저장소 `docs/ARCHITECTURE.md`

## 실행

```bash
# 스킬 시험대 (ROS 노드 없음, MuJoCo 물리 직접 구동). 전체 시나리오 + 궤적 그림
ros2 run pinky_soccer_skills skill_bench --plot /tmp/skill_plots
ros2 run pinky_soccer_skills skill_bench dribble_straight --seeds 3
ros2 run pinky_soccer_skills skill_bench dribble_straight --set dribble.mode=tap,push,hybrid

# 경기 (기본 decision:=skills). 시작/정지는 기존과 같이 /game/playing
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco decision:=bt   # 이전 BT
```

## 구조

| 모듈 | 내용 |
|---|---|
| `geometry.py`, `context.py`, `params.py` | 좌표 도구, 스킬 입출력(`SkillContext`/`SkillOutput`), 파라미터 기본값과 근거 |
| `prediction.py` | L0 질의: 공 위치 예측(일정 구름 감속), 요격점, 선 통과점 |
| `motion.py` | L1: 명령 제한기, 후진 포함 자세 제어, 운동학 롤아웃, 공 회피 경로 후보 계획 |
| `skills.py` | L2: SearchBall, ApproachBall, OrbitBall, Dribble, ChargeShot, RecoverBall, BlockShot, ShadowBall, GoToPose, TurnTo |
| `planning.py`, `playbook.py` | L3: 상대 우회 경로(CurveDribble), Striker 플레이북(공격·수비·뺏기·탐색) |
| `skill_player.py` | ROS 노드: `/world_model`·`/odom`·`/us_sensor/range` → `/cmd_vel`, `/skills/status` |
| `sim/` | 시험대: MuJoCo 월드(공식 모델+경기장+공+재질+상대 블록), 센서 모델, 시나리오 실행/채점/그림 |
| `scenarios/*.yaml` | 스킬별·경기 시나리오와 합격 기준 |

## 설계 제약

- 스킬은 `SkillContext`만 읽습니다. 시험대와 ROS 노드가 같은 형식을 채우므로 스킬 코드는 그대로 실물에서 동작합니다.
  정답값(`/ground_truth`, 시뮬레이터 상태)은 시험대 채점·그림에만 씁니다.
- 시험대는 공식 `pinky_mujoco` `Physics`와 `pinky_soccer_sim`의 경기장·공·재질 래퍼를 그대로 씁니다(공식 코드 수정 없음).
  카메라는 렌더링 대신 기하 가시성 모델(원 둘레 50% 이상 화면 안, 가림 없음)이며 잡음·지연·위치 오차는 가정값입니다.
- 상대 로봇은 키네마틱 블록(무한한 힘)이라 밀어내기 싸움·뺏기는 검증되지 않습니다.
- 모든 수치의 근거는 `params.py` 주석에 있습니다. 실측이 필요한 값은 `config/skills.yaml`에 표시합니다.
