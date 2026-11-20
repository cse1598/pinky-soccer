# pinky_soccer

축구 제어 패키지입니다. 판단 계층 구조는 [docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md)를 참고합니다.

## reactive_controller

`/behavior_goal`의 목표를 비례 제어로 따라가 `/cmd_vel`을 냅니다. 로봇 자세는 `/world_model`의 `self`(map)에서 읽어 목표와 같은 좌표계를 씁니다.

```bash
ros2 run pinky_soccer reactive_controller --ros-args --params-file \
  $(ros2 pkg prefix pinky_soccer)/share/pinky_soccer/config/reactive_controller.yaml
```

| mode | 동작 |
|---|---|
| REACTIVE | 목표 위치로 이동(방향 오차 > `turn_in_place`면 제자리 회전, 속도는 cos² 감속), 도착 후 목표 방향으로 회전 |
| NAVIGATE | Nav2 연동 전까지 REACTIVE와 같은 비례 제어 (TODO) |
| STOP | 0 |

- `max_speed > 0`이면 선속도 상한을 낮춥니다. `SpinInPlace`에서는 회전 속도 상한으로 씁니다.
- 안전: `/behavior_goal` 또는 `/world_model`이 `goal_timeout`/`pose_timeout`(0.5 s, steady clock)보다 오래되면 0을 냅니다. 실물·시뮬레이터의 `command_watchdog`(0.5 s)와 별개입니다.
- 이득·상한은 `config/reactive_controller.yaml`, 모두 시작값 가정입니다.

`hello_node.py`(`hello_pinky`)는 예전 예제로 그대로 둡니다.
