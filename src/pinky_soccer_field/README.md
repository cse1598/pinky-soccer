# pinky_soccer_field

경기장 치수와 기하를 `map` 좌표로 제공하고, AMCL용 2D 점유 지도를 만드는 패키지입니다.
월드 모델, 행동 트리 입력, UI가 모두 이 패키지로 경기장을 읽습니다. 시뮬레이터(gz, MuJoCo)에 의존하지 않으므로 로봇과 PC 어디서나 쓸 수 있습니다.

## 좌표계

- `map` = 경기장 좌표계. 원점은 경기장 중앙, z=0은 평평한 바닥 상단, **+X 파랑 골대 / −X 노랑 골대**, 단위 m.
- 시뮬레이터 world 원점도 경기장 중앙이므로 시뮬레이터 실제 위치와 `map` 좌표가 같습니다.
- 우리 골대 색 `our_goal_color`(`yellow`|`blue`, 기본 `yellow`)로 `own_goal`/`opp_goal`과 `attack_sign`(공격 방향 부호)을 계산합니다. 코드에서 "+X = 상대 골대"로 가정하지 마세요.

## 치수 원본

치수 원본은 **하나**입니다: `pinky_soccer_sim/config/rcj_field.yaml` (Codex 작업 영역, 이 패키지로 옮기지 않음).

- 노드는 `field_spec` 파라미터(YAML 경로)로 읽습니다. 비어 있으면 `PINKY_SOCCER_FIELD_SPEC` 환경 변수, 그다음 설치된 `pinky_soccer_sim` share의 `config/rcj_field.yaml`을 씁니다.
- 로봇처럼 `pinky_soccer_sim`이 설치되지 않은 곳에서는 YAML 사본 경로를 `field_spec`으로 넘기세요.
- `load_spec()`은 sim 생성기의 `load_spec()`과 같은 검증 규칙을 따릅니다. sim 패키지가 있으면 테스트가 두 로더의 판정이 같은지 확인합니다.
- 장기적으로 YAML과 `load_spec()`을 이 패키지로 옮기고 sim 생성기가 여기서 import하는 편이 깔끔합니다. 사용자와 Codex의 동의 후 별도 작업입니다 (`.agents/STATUS.md` 메시지 참고).

## API (`pinky_soccer_field.field_geometry`)

```python
from pinky_soccer_field.field_geometry import Field
field = Field.from_file(path_or_None, our_goal_color='yellow')
field.own_goal, field.opp_goal, field.attack_sign      # (x, y), (x, y), ±1
field.goal_mouth('blue'), field.goal_back('blue')      # 선분
field.inner_walls, field.playing_area                  # Rect (contains, clamp)
field.penalty_area('yellow'), field.center_circle, field.neutral_spots
field.ball_in_goal(x, y, 'blue')                       # 공 중심이 골라인을 넘고 골대 폭 안
field.obstacles_seen_at(lidar_height)                  # 라이다 높이 위로 솟은 고체
```

주요 값 (2026-06-03 규격): 골대 입구 x=±1.095, 골대 안쪽 뒤 x=±1.169, 폭 y=±0.30, 외벽 안쪽 2.43 × 1.82 m, 페널티 구역 0.25 × 0.80 m, 중앙 원 반지름 0.30 m.

## 지도 생성

```bash
ros2 run pinky_soccer_field generate_field_map --output maps/rcj_field
# 옵션: --spec <yaml> --resolution 0.01 --lidar-height 0.125 | --urdf <file>
```

- 라이다 스캔 높이보다 위로 솟은 고체만 점유로 그립니다. Pinky Pro 라이다는 지면 0.125 m(URDF `base_footprint → rplidar_link`)이고 골대 구조는 지붕 포함 0.11 m이므로 **외벽만** 그려집니다. 골대·흰 선은 카메라로 봅니다.
- 라이다 높이는 기본으로 `pinky_soccer_description`의 `soccer_robot.urdf.xacro`를 펼쳐 계산합니다.
- 해상도 기본 0.01 m는 가정입니다. AMCL 성능을 보며 조정하세요.
- 지도 `origin`은 경기장 중앙이 `map` 원점이 되도록 (−폭/2, −높이/2)로 잡습니다. 벽 안쪽은 자유(254), 벽은 점유(0), 벽 바깥은 미지(205)입니다.
- 생성 결과는 재현 가능한 산출물이므로 git에 넣지 않았습니다. 필요할 때 위 명령으로 다시 만드세요.

## 대칭 문제 (AMCL)

외벽 사각형만 있는 지도는 **좌우·상하 대칭**입니다. 라이다만으로는 실제 자세와 180° 회전한 자세를 구별할 수 없어 AMCL이 뒤집힌 해에 빠질 수 있습니다.

1. 킥오프 자세를 초기 위치로 반드시 지정합니다 (`initial_pose` 또는 `/initialpose`). 우리 편 진영에서 상대 골대를 보고 시작합니다.
2. 이후에는 카메라로 본 골대 색(노랑/파랑)으로 방향을 검증하고, 뒤집혔으면 자세를 180° 돌려 재초기화하는 보정이 필요합니다 (TODO).
3. 흰 선 검출로 AMCL을 보정하는 것은 확장 과제입니다.

## 테스트

```bash
colcon test --packages-select pinky_soccer_field && colcon test-result --verbose
```

골대 입구 x = ±`playing_length`/2, 외벽 안쪽 2.43 × 1.82 m, 지도 원점 왕복, `our_goal_color` 반전 시 부호 반전, sim 로더와의 검증 동치, 라이다 높이 계산을 확인합니다.
