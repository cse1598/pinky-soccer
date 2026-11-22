# pinky_soccer_msgs

판단 계층이 주고받는 메시지입니다. 모든 층(월드 모델, 행동 트리, 제어, UI)에 영향을 주므로 **필드를 바꿀 때는 사람에게 확인받습니다** (`docs/ARCHITECTURE.md`).

| 메시지 | 토픽 | 설명 |
|---|---|---|
| `TrackedObject` | (내장) | 객체 하나: `id`, `pose`(Pose2D, map), `velocity`(x, y만), `covariance`(x, y, θ 3×3 행 우선), `state`(OBSERVED=0 / ESTIMATED=1 / LOST=2), `last_seen`(마지막 실제 관측 시각), `confidence` |
| `WorldModel` | `/world_model` | `header`(frame `map`), `self`, `ball`, `opponents[]`, `own_goal`·`opp_goal`(골대 입구 중심), `our_goal_color`, `playing`, `ball_search_grid`(공 존재 확률 0~100) |
| `BehaviorGoal` | `/behavior_goal` | `branch`(ATTACK/DEFEND/SEARCH/STOP), `action`(실행 중 leaf), `mode`(NAVIGATE=0 / REACTIVE=1 / STOP=2), `target`(Pose2D, map), `max_speed`(0이면 제어기 기본값), `planned_path` |

경기 시작·정지는 당분간 `std_msgs/Bool`의 `/game/playing`으로 받습니다 (심판 장치 없음):

```bash
ros2 topic pub --once /game/playing std_msgs/msg/Bool "{data: true}"
```

## 설계 메모

- 카메라 단발 요청 서비스 `/camera/capture_frame`: `pinky_soccer_msgs/srv/CaptureFrame`.
  `format`은 raw/jpeg, `timeout_sec`은0초 초과~10초 이하(기본3초). 응답의 success/message와
  선택된 image 또는 compressed_image 및 같은 촬영의 camera_info를 반환합니다.
  기존 판단 메시지 필드는 변경하지 않습니다. 사용법은 bringup의 `CAMERA_CAPTURE.md` 참고.
- 공분산은 위치 불확실성을 직접 담아, 행동 트리의 `BallKnown`이 시간 대신 "최대 표준편차 ≤ `max_sigma`"로 판단합니다.
- `velocity`는 `Vector3`이지만 z는 쓰지 않습니다.
- `self`는 Python·C++ 모두에서 필드 이름으로 문제없이 생성됨을 확인했습니다 (`msg.self`).
