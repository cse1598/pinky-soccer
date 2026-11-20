# pinky_soccer_world_model

센서 → `/world_model` 계층입니다. 행동 트리는 이 토픽만 봅니다.

| 실행 파일 | 입력 | 출력 |
|---|---|---|
| `ball_detector` | `/camera/image_raw`(BEST_EFFORT), `/camera/camera_info`, TF | `/ball_detection` (`PointStamped`, `base_link`, 영상 시각) |
| `world_model_node` | `/ball_detection`, `/odom` 또는 TF `map→base_footprint`, `/scan`, `/camera/camera_info`, `/game/playing` | `/world_model` (~30 Hz), odom_only면 TF `map→odom`(static) |
| `fake_world_model` | `/cmd_vel`, `/game/playing` | `/world_model`, `/sim/ball_ground_truth`(가짜 실제 공) |

```bash
ros2 launch pinky_soccer_world_model world_model.launch.py use_sim_time:=true localization:=odom_only
ros2 run pinky_soccer_world_model fake_world_model --ros-args --params-file $(ros2 pkg prefix pinky_soccer_world_model)/share/pinky_soccer_world_model/config/fake_world_model.yaml
```

설정: `config/ball_detector.yaml`, `config/ball_filter.yaml`, `config/world_model.yaml`(`our_goal_color`, 임계값), `config/fake_world_model.yaml`. 경기장 치수는 `field_spec` 파라미터로만 받습니다 (비면 sim share의 `rcj_field.yaml`, `pinky_soccer_field` README 참고).

## ball_detector

- HSV 색 검출 → 가장 큰 원형 덩어리의 중심 픽셀 → 카메라 모델로 광선 → `base_footprint`의 평면 z = 공 반지름과 교차 → `base_link`로 변환.
- HSV 범위·최소 면적·원형도는 **가정**입니다(시뮬레이터 공 rgba `1 0.45 0` 기준). 실제 주황 골프공과 경기장 조명에서 다시 정해야 합니다.
- 내부 파라미터: `camera_yaml`(ROS camera YAML 또는 보정 UI의 `profile.yaml`)이 있으면 그것, 없으면 `camera_info`. `camera_info`의 K가 0(미보정)이면 검출하지 않고 경고합니다. 보정 프로필을 launch의 `calibration_profile`로 적용했다면 `camera_info`에 이미 들어 있습니다.
- 실물 영상이 광학 프레임 기준 180° 돌아 있으면 `image_rotated_180:=true` (`MEMORY.md` 열린 질문: 실물 방향 미확인).
- 공 중심 높이에서 교차하므로 공이 바닥에 있다는 가정입니다. YOLO로 바꿀 때는 같은 `/ball_detection` 토픽을 내면 됩니다.

## world_model_node

- **자기 위치**: `localization:=tf`면 TF `map→base_footprint`(AMCL 사용 시). `odom_only`면 `/odom`을 킥오프 자세 `initial_pose`(map) 뒤에 붙이고 `map→odom` static TF를 냅니다 (AMCL 연동 전 단계).
- **공**: 2D 등속 + 마찰 감쇠 칼만 필터(`ball_filter.py`). 관측이 없으면 예측만 하고 공분산이 커집니다. 측정 표준편차는 거리 제곱에 비례해 커집니다. 마할라노비스 게이트로 튀는 값을 버리되 3번 연속이면 공이 옮겨진 것으로 보고 다시 잡습니다. 경기장 밖으로 예측되면 외벽 안쪽(공 반지름만큼)으로 제한하고 그 축 속도를 0으로 둡니다(비탄성 가정).
- **상태**: OBSERVED = 마지막 관측 후 `observed_timeout`(0.3 s) 이내, LOST = `lost_timeout`(5 s) 초과 또는 최대 표준편차 > `lost_sigma`(0.5 m), 그 사이 ESTIMATED.
- **ball_search_grid**: 셀마다 공 존재 믿음(0~1, 메시지는 0~100). 카메라가 살아 있고(최근 0.5 s 내 `camera_info`) 공이 OBSERVED가 아니면 시야 부채꼴 안 셀을 낮추고, 모든 셀은 시간이 지나면 사전값 0.5로 돌아갑니다. 검출되면 그 주변을 높입니다. 시야각은 `camera_info`의 K에서 계산, 거리 범위(0.15~1.5 m)는 가정. 상대에 의한 가림은 TODO.
- **상대**: `/scan` 점 중 외벽 안쪽 `wall_margin`보다 안의 점을 이웃 거리로 묶고, 크기 ≤ 0.30 m인 군집을 상대 후보로 냅니다. 정교한 추적(ID 유지, 속도)은 TODO. 라이다(0.125 m)는 공(지름 0.043 m)을 보지 못합니다.
- **경기 시작**: `/game/playing`(std_msgs/Bool). 기본 `playing_default: false`.

## fake_world_model

로봇·시뮬레이터 없이 행동 트리와 UI를 시험합니다. 공은 경기장 안 타원을 천천히 구르고, 12초 주기로 OBSERVED → ESTIMATED(공분산 증가, 마지막 관측에서 외삽) → LOST를 반복합니다. 상대 1대는 상대 진영을 돌며 따로 상태가 바뀝니다. 나는 `/cmd_vel`이 오면 그대로 적분해서 움직이고(제어기 연결 확인용), 없으면 천천히 원을 그립니다.

## 테스트

`colcon test --packages-select pinky_soccer_world_model`: 필터 상태 전이·마찰 예측·벽 제한·이상치 처리, 검색 그리드, 광선-바닥 교차, HSV 검출, 라이다 군집 (12개, flake8 포함).

## 가정과 남은 일

- 필터·그리드·검출 수치는 모두 시작값 가정입니다. 카펫 위 골프공 구름(마찰 시간 상수)은 실측이 필요합니다.
- AMCL(`localization:=tf`) 연동, 상대 추적, 가림 처리, 카메라로 본 골대 색으로 자기 방향 검증은 후속 작업입니다.
