# pinky_soccer_world_model

센서 → `/world_model` 계층입니다. 행동 트리는 이 토픽만 봅니다.

경기 launch 기본 위치 추정은 **LiDAR AMCL + 골대 색/지자기 대칭 후보 확인**입니다.
기록·실물 NDOF 드라이버·세 환경 실행/검증은 [LOCALIZATION.md](LOCALIZATION.md)를 참고하세요.

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

- 경기와 보정 UI가 같은 `robust_ball.BallImageDetector`를 사용합니다. 원본/제한적 화이트밸런스의 주황색 단서 → 밝기 CLAHE와 정규화 R−G 경계 → Hough 원 후보 → 보이는 원호의 방사 방향 경계 검사와 이상점 제거 원 맞춤 → 점수·시간 연속성으로 **최대 하나**를 선택합니다. 색 영역이 책상과 이어져도 영상 경계로 공을 분리하며, 원본 픽셀은 변경하지 않습니다.
- 기본 색상 씨앗 H3~19는 유지하고 그림자에 대해 S/V 하한을 최대95/25로 완화합니다. 약한 주변색(H±4)·흰 반사광은 내부 지지로만 사용하며 독립적인 공 씨앗이 아닙니다. 엄격한 채움/축비/원형도 검사는 완전한 색 윤곽 후보 경로이고, 가림은 별도 원호 경로로 복원합니다. 기본 원호 지지율≥42%, 주황 단서≥10%, 내부 설명 비율≥42%, 점수≥0.48은 실험적 초기값입니다.
- 원 안/밖의 정규화 R−G 대비와 남은 색의 채도 일관성도 확인해 책상 무늬·피부의 원호를 줄입니다. 점수는 원호·색·내부 지지·경계 대비·색 강도의 가중합이며 확률이 아닙니다. 최근0.7초의 위치/크기는 후보 연결에만 사용하고 현재 영상 근거가 없으면 **미검출**입니다. 전체 가림/완전한 백색 포화에서는 복원할 수 없고 같은 색의 둥근 물체는 오인식할 수 있습니다. 부분 가림과 화면 잘림은 충분한 원호가 있을 때 허용하므로 중심/반지름 오차는 커질 수 있습니다.
- `publish_debug_image:=true`는 `/ball_detection/debug_image`(일반 초록 원, 가림/반사광 복원 황색 점선·점수)와 `/ball_detection/debug_mask`(mono8, 흰색=색상 씨앗)를 같은 원본 시각으로 발행합니다. 투영은 원 중심 광선과 `base_footprint`의 z=공 반지름 평면 교차 후 `base_link` 변환으로 유지합니다. 보정 UI도 같은 설정값/구현을 쓰며 이전 사용자 YAML이 새 기본값을 덮어쓰는지 확인하세요.
- HSV 범위·최소 면적·원형도는 **가정**입니다(시뮬레이터 공 rgba `1 0.30 0` 기준). 공 지름은 사용자 실측42.7mm이며 형광 주황 외관은 사진 기반 근사입니다. 실제 공과 경기장 조명에서 HSV 조건은 별도로 검증해야 합니다.
- 2026-11-22 검증: 실제 노트북 웹캠의 공/책상 장면84프레임 검출·마지막 처리16ms, 저장 영상의 조명색/감광/가림/백색 반사광 변형에서 연속 대상 유지, 알려진 중심/반경 합성 시험·손실/재획득 시험 통과. 이는 영상 검출 검증이며 실물 조명 전체나 거리 정확도 보장은 아닙니다. 처음부터 심하게 가린 경우보다 온전한 공으로 추적을 시작한 경우가 안정적입니다.
- 내부 파라미터: `camera_yaml`(ROS camera YAML 또는 보정 UI의 `profile.yaml`)이 있으면 그것, 없으면 `camera_info`. `camera_info`의 K가 0(미보정)이면 미터 단위 `/ball_detection`을 발행하지 않고 경고합니다. 픽셀 미리보기는 보정 없이도 가능합니다. 보정 프로필을 launch의 `calibration_profile`로 적용했다면 `camera_info`에 이미 들어 있습니다.
- 실물 영상이 광학 프레임 기준 180° 돌아 있으면 `image_rotated_180:=true` (`MEMORY.md` 열린 질문: 실물 방향 미확인).
- 공 중심 높이에서 교차하므로 공이 바닥에 있다는 가정입니다. YOLO로 바꿀 때는 같은 `/ball_detection` 토픽을 내면 됩니다.

실물 시험 경기는 `pinky_soccer_viz/REAL_GAME.md`를 참고합니다.
`allow_uncalibrated:=true`일 때만 시야각 가정(`approximate_hfov_deg`, 기본 66°)과
영상 중앙 주점·왜곡 0으로 임시 미터 좌표를 계산합니다. 실제 보정값이 아니며
`APPROXIMATE` 표시·경고를 냅니다. 기본값은 false, 유효한 CameraInfo가 우선합니다.

### 실물 카메라 미보정 상태에서 공 인식 확인

로봇의 `robot.launch.py`로 영상이 발행되는 상태에서 PC에서 실행합니다.
각 터미널은 워크스페이스를 source하고 로봇과 같은 ROS domain/DDS 설정을 사용합니다
(2026-11-21 실물 연결 확인 시 domain 37).
확인용 노드는 다른 이름을 사용하고 미터 출력을 별도 토픽으로 분리하므로,
기존 월드 모델의 공 관측에 영향을 주지 않습니다.

```bash
source ~/pinky-soccer/install/setup.bash
export ROS_DOMAIN_ID=37
ros2 run pinky_soccer_world_model ball_detector --ros-args \
  -r __node:=ball_detector_preview -r ball_detection:=ball_preview/metric \
  -p publish_debug_image:=true
```

다른 PC 터미널에서 표시합니다. rqt의 Image Topic에서
`/ball_detection/debug_image`를 선택합니다.

```bash
ros2 run rqt_image_view rqt_image_view /ball_detection/debug_image
```

공을 좌우·앞뒤로 옮겨 녹색 원과 십자가가 따라가는지 확인합니다.
상단 `BALL pixel=(u, v)`는 원본 영상의 좌상단 기준 픽셀 좌표이며,
없으면 `NO BALL`, 미보정이면 `UNCALIBRATED`로 표시합니다.
영상 자체를 회전시키지 않으므로 `image_rotated_180` 설정과 관계없이 표시가 공에 겹칩니다.
이 확인에는 TF·경기장 위치·카메라 보정값이 필요하지 않습니다.
주황색 다른 물체도 후보가 될 수 있으므로 실제 공과 조명에서 HSV 범위를 확인해야 합니다.
보정 전에는 거리·경기장 좌표를 만들지 않으며, 이 검출기만 실행해도 주행 명령은 발행되지 않습니다.
정확한 거리 추정·공 추적·경기 제어는 내부 보정 및 카메라 장착 자세 검증 후 진행합니다.

## world_model_node

- **자기 위치**: `localization:=tf`면 TF `map→base_footprint`(AMCL 사용 시). `odom_only`면 `/odom`을 킥오프 자세 `initial_pose`(map) 뒤에 붙이고 `map→odom` static TF를 냅니다 (AMCL 연동 전 단계).
- 경기 launch 기본 `localization:=lidar`는 AMCL의 TF를 사용하며 AMCL·odom 단절 시 playing=false를 내보냅니다. `localization:=odom_only` 선택 시 실물은 `align_first_odom:=true`로 첫 odom을 `initial_pose`에 맞춥니다.
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
- AMCL·골대 색/지자기 방향 확인은 연결됐으며 실물 간섭·HSV·지도 정합 검증이 필요합니다. wheel+gyro EKF, 상대 추적·가림 처리는 후속입니다.

## 카메라 보정 UI의 공 검토

`pinky_soccer_calibration`의 **공 인식 검토** 탭은 경기용 `BallImageDetector`,
공통 `ball_vision.project_ball`/`annotate_ball`, `BallFilter`를 재사용합니다.
`geometry.detect_color_blob`도 같은 단일 프레임 검출을 호출합니다. 토픽/미터 투영 계약은 유지하며 원호 복원·시간 연속 검출도 공유합니다.
검토 탭은 JPEG를 사용하고 별도 검출/주행 토픽을 발행하지 않습니다.
설정·보정값 선택 및 비교 한계는 [ROS_UI.md](../pinky_soccer_calibration/ROS_UI.md)를 참고하세요.
