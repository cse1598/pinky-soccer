# pinky_soccer_bt

`/world_model`을 읽어 `/behavior_goal`을 내는 행동 트리입니다 (BehaviorTree.CPP v4.11 + BehaviorTree.ROS2 `0.3.0`).
트리는 센서를 직접 구독하지 않고, 목표만 정합니다. `/cmd_vel`은 `pinky_soccer`의 `reactive_controller`가 냅니다.

## 실행

```bash
ros2 launch pinky_soccer_bt bt.launch.py            # config/bt_node.yaml
ros2 topic pub --once /game/playing std_msgs/msg/Bool "{data: true}"
```

Groot2(`~/Groot2/bin/groot2`)로 포트 1667에 연결하면 실시간으로 볼 수 있습니다. Groot2 Free의 노드 수 제한에 걸리면 `cout_logger:=true` 또는 `file_logger_path:=/tmp/soccer.btlog`(FileLogger2)를 씁니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `tree_file` | share `trees/soccer_1v1.xml` | |
| `tick_rate_hz` | 15 | 10~20 Hz |
| `groot2_enabled`, `groot2_port` | `true`, 1667 | |
| `cout_logger` | `false` | StdCoutLogger |
| `file_logger_path` | 빈 문자열 | 비어 있지 않으면 FileLogger2 |
| `world_model_topic`, `behavior_goal_topic` | `/world_model`, `/behavior_goal` | XML의 `topic_name`이 우선 |

## 구조

- `trees/soccer_1v1.xml`: 지시문 부록 C 그대로 + 발행 전용 `PublishGoal` 트리.
- `ReadWorldModel`(`RosTopicSubNode`): 최신 `/world_model`을 블랙보드 포트로 복사. 메시지가 없거나 `stale_sec`(0.5 s, steady clock)보다 오래되면 FAILURE + STOP 목표.
- 액션 노드는 목표를 루트 블랙보드 `goal`에 씁니다. `bt_node`는 매 주기 `Main`을 tick한 뒤 `PublishGoal` 트리(`PublishBehaviorGoal`, `RosTopicPubNode`)를 tick해서, 액션이 RUNNING이어도 목표가 나갑니다.
- 기하 판정은 `soccer_logic.{hpp,cpp}`에 ROS 없이 모았고 gtest로 검사합니다.

| 노드 | 동작 |
|---|---|
| `IsPlaying` | `playing` |
| `BallKnown` | 공이 LOST가 아니고 위치 공분산의 최대 표준편차 ≤ `max_sigma` (시간이 아닌 공분산 기준) |
| `BallInOwnHalf` | 공 x가 우리 골대 쪽 절반 + `margin` 안 |
| `OpponentCloserToBall` | LOST가 아닌 상대 중 나보다 공에 가까운 것이 있음 |
| `IsBehindBall` | (공→상대 골대) 슛 선 뒤쪽(공 뒤 `max_behind` 0.30 m 이내), 선에서 옆으로 `tol_dist` 이내, 방향 오차 `tol_angle` 이내. 밀면서 공에 붙어도 참이 유지되도록 "뒤 offset 지점과의 거리" 대신 선 기준으로 판정 |
| `NotFacingOwnGoal` | 진행 방향과 우리 골대 방향의 각도 ≥ `min_angle` |
| `GoBehindBall` | 슛 선 위 공 뒤 `offset` 지점, 공(골대)을 향하는 방향. 로봇이 공 앞·옆에 있으면 먼저 공 옆 `clearance` 지점을 거쳐 돌아감 (단순 우회, 충돌 검사 없음 — TODO) |
| `PushBall` | 상대 골대 중심을 슛 방향으로 `beyond`(0.10 m) 지난 지점, 제한 속도 `speed` |
| `GoToBlockPoint` | 우리 골대→공 선분의 `ratio` 지점, 공을 향함. 도달 판정에 히스테리시스(0.05/0.10 m) |
| `FaceBall` | 제자리에서 공을 향함 |
| `TurnToLastSeen` | 마지막 공 추정 방향으로 회전, 그 방향을 보면 FAILURE(다음 탐색 단계로) |
| `SpinInPlace` | 목표 방향을 현재보다 1 rad 앞에 계속 둠, `max_speed` = 회전 속도 상한 |
| `GoToPose` | 지정 자세(기본 경기장 중앙, theta 0)로 이동. 위치 5 cm·방향 0.15 rad 안이면 SUCCESS |
| `StopRobot` | mode STOP |

mode: GoToBlockPoint·GoToPose = NAVIGATE(Nav2 연동 전까지 제어기가 같은 비례 제어), 나머지 이동 = REACTIVE.

## 검증 모드 (`verify_mode:=true`)

`bt_node`가 `Main` 대신 `/behavior_test/request`(JSON)로 요청받은 **액션 노드 하나 또는 서브트리 하나**를
같은 factory·같은 XML로 만들어 `ReactiveSequence(ReadWorldModel, 대상)`으로 실행합니다
(`src/verify_mode.cpp`). 요청 노드·포트는 C++ 노드 모델로 검사하고 숫자만 받으며, 월드 입력 포트는
자동 연결합니다. 상태는 `/behavior_test/status`(노드별 상태, 조건 판정, 목표)로 매 tick 냅니다.
제한 시간(노드 시계)·월드 모델 끊김·검증 창 무응답(1.5 s)이면 STOP. 화면과 사용법은
`pinky_soccer_viz/VERIFY.md`, 목록·설명은 `config/verify_catalog.yaml`.
`bt_node --print-models`는 노드 포트/기본값과 실행 가능한 대상을 출력합니다.

`GoBehindBall`은 이제 `planned_path`에 우회점 → 공 뒤 자리를 채웁니다(표시용, 제어기는 target만 사용).

## 알려진 한계 / TODO

- XML의 수치(`max_sigma`, `margin`, `tol_*`, `offset`, `speed`, `ratio`)는 모두 시작값 가정입니다.
- `NotFacingOwnGoal min_angle=1.57`은 상대 골대 쪽 구석(예: 공 (1.0, 0.7))에서 정상 슛도 막습니다. 로봇이 공 뒤에서 계속 자리만 잡습니다. 테스트 `NotFacingOwnGoalBlocksCornerShotsKnownLimit`로 고정해 두었으며, 시뮬레이션에서 `min_angle`을 1.0 근처로 줄이는 것을 검토합니다. 자책골 방지 자체는 "슛 방향은 항상 상대 골대 쪽 x 성분을 가진다"는 성질로 보장됩니다(테스트 `ShotDirectionAlwaysTowardsOpponentSide`).
- 공을 건드리지 않고 돌아가는 경로 계획은 단순 우회점뿐입니다.
- `planned_path`는 GoBehindBall만 채웁니다 (나머지는 Nav2 연동 시).

## 테스트

```bash
colcon test --packages-select pinky_soccer_bt && colcon test-result --verbose
```

- `test_soccer_logic`: 기하 판정, 자책골 방지 성질, 우회점, 목표점 계산.
- `test_verify_catalog`: 검증 목록이 실행 가능한 대상과 같고, 포트·기본값이 트리 XML·C++ 모델과 같은지.
- `test_soccer_tree`: 실제 XML을 로드하고 `/world_model`을 발행해 STOP/PushBall/GoBehindBall/GoToBlockPoint/TurnToLastSeen 선택과 `/behavior_goal` 발행을 확인 (ROS_DOMAIN_ID 93으로 격리).
