# pinky_soccer_world_model

실물·MuJoCo·Gazebo의 기본 위치 추정은 같은 [경량 직사각형 Localization](STARTUP_LOCALIZATION.md)입니다.
외벽 정합→최소각 골대 조준→정지 후 HLS 3프레임→단일색으로 IMU 양축 확정→팀 LED 순서입니다.
런치 후에는 센서 수신만 하고, `localization/calibrate` Trigger 또는 창의 R 신호로 보정을 시작합니다.

센서 → `/world_model` 계층입니다. 행동 트리는 이 토픽만 봅니다.

초기화 뒤에는 골대 영상처리 없이 LiDAR·IMU·odom으로 추적합니다.
기존 AMCL은 `localization:=lidar`로 보존하며 [LOCALIZATION.md](LOCALIZATION.md)에 설명합니다.
구현·성능·미검증 사항은 [통합 보고서](../../docs/LIGHTWEIGHT_LOCALIZATION.md)에 있습니다.

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

- 실물·MuJoCo·Gazebo·보정 UI가 같은 `robust_ball.BallImageDetector`를 사용합니다.
  하단350px(640×480에서 y≥130)에서 HLS 한 번 변환 → 색 윤곽 → 원호 합의/기존 최소제곱 원 피팅으로 최대 하나를 선택합니다.
  Hough·Sobel·CLAHE·화이트밸런스·딥러닝을 사용하지 않습니다. 원본 영상과 K/D는 바꾸지 않습니다.
- 원래 영상 픽셀의 `(u,v,r,area)`를 유지하고 ROI Y오프셋을 복원합니다. 충분한 원호이면 잘림/가림의 전체 원을 추정합니다.
  근거가 부족하면 원/미터 좌표를 강제하지 않고 `ball_detection/status`에 방향 픽셀과 거리 유효 상태를 분리해 표시합니다.
  방향 전용 위치는 보이는 원호의 대략적 중심이며 정밀한 전체 공 중심 관측이 아닙니다.
- 기존 `project_ball`은 그대로입니다. 영상 r로 거리를 계산하지 않고 중심 광선을 왜곡 보정한 뒤
  공 반경 **0.02135m** 높이의 평면과 교차합니다. 지름42.7mm·K/D·센서 TF·좌표 변환을 보존합니다.
- `ball_offset_source:=auto`는 선택한 보정 프로필 또는 camera_yaml의 형제 profile.yaml에 저장된
  human_verified 오프셋을 읽습니다. `none`은 보정 없는 기존 투영, 경로를 지정하면 그 프로필입니다.
  base_link X/Y[m] 보정은 투영/좌표 변환 후 한 번 적용하며 Z는 유지합니다.
  보정 UI는 기존 raw 관측+범퍼 기준 비교에서 한 번만 적용하므로 중복 보정하지 않습니다.
- `publish_debug_image:=true`는 기존 원본/표시/mono8 마스크 토픽을 유지합니다.
  현재 ROS 미터 출력은 기존 PointStamped이며, 신뢰할 수 없는 원은 발행하지 않습니다.
- HLS/원호 임계값은 `config/ball_detector.yaml`에 있습니다. 기존 HSV·compact-shape 키 일부는
  과거 설정 로드를 위해 남겼으며 새 HLS 마스크/Hough 없는 경로에서는 사용하지 않습니다.
  최근0.7초의 기억은 연결만 돕고 새 관측/반지름을 만들지 않습니다.
- 코드 경로·192개 동일 입력 비교·캘리브레이션 보존·반지름 편차·실행 방법은
  [HLS 교체 및 거리 보존 보고서](../../docs/BALL_HLS_DISTANCE.md)를 참고하세요.
- 내부 파라미터: `camera_yaml`(ROS camera YAML 또는 보정 UI의 `profile.yaml`)이 있으면 그것, 없으면 `camera_info`. `camera_info`의 K가 0(미보정)이면 미터 단위 `/ball_detection`을 발행하지 않고 경고합니다. 픽셀 미리보기는 보정 없이도 가능합니다. 보정 프로필을 launch의 `calibration_profile`로 적용했다면 `camera_info`에 이미 들어 있습니다.
- 실물 영상이 광학 프레임 기준 180° 돌아 있으면 `image_rotated_180:=true` (`MEMORY.md` 열린 질문: 실물 방향 미확인).
- 공 중심 높이에서 교차하므로 공이 바닥에 있다는 가정입니다. YOLO로 바꿀 때는 같은 `/ball_detection` 토픽을 내면 됩니다.

실물 시험 경기는 `pinky_soccer_viz/REAL_GAME.md`를 참고합니다.
`allow_uncalibrated:=true`일 때만 시야각 가정(`approximate_hfov_deg`, 기본 66°)과
영상 중앙 주점·왜곡 0으로 임시 미터 좌표를 계산합니다. 실제 보정값이 아니며
`APPROXIMATE` 표시·경고를 냅니다. 기본값은 false, 유효한 CameraInfo가 우선합니다.

공 검출은 기본적으로 **영상 상단 1/3을 제외한 아래 2/3**만 검색합니다. 흰색 폼보드 벽 위의
경기장 밖 배경을 제외하려는 현재 카메라 배치 기준이며, 흰 벽 경계를 자동 추정하는 기능은 아닙니다.
`config/ball_detector.yaml`의 `detection_top_fraction`(0 이상 1 미만, 0은 전체 화면)으로 조정합니다.
경기 디버그 영상과 보정 UI의 공 검토 화면은 제외 영역을 어둡게 하고 경계선을 표시합니다.
ROI 내부에서 검출하더라도 출력 픽셀 좌표와 거리 투영은 원래 전체 영상 기준을 유지합니다.

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
경기장 영상에서 실제 공과 조명에 맞는 HLS 범위를 확인합니다.
보정 전에는 거리·경기장 좌표를 만들지 않으며, 이 검출기만 실행해도 주행 명령은 발행되지 않습니다.
정확한 거리 추정·공 추적·경기 제어는 내부 보정 및 카메라 장착 자세 검증 후 진행합니다.

## world_model_node

- **자기 위치**: `localization:=tf`면 TF `map→base_footprint`(AMCL 사용 시). `odom_only`면 `/odom`을 킥오프 자세 `initial_pose`(map) 뒤에 붙이고 `map→odom` static TF를 냅니다 (AMCL 연동 전 단계).
- 경기 launch 기본 `localization:=rectangle`는 단독 map→odom과 기존 `localization/pose`를 제공합니다. 명시적 `localization:=lidar`는 기존 AMCL 경로입니다. `localization:=odom_only` 선택 시 실물은 `align_first_odom:=true`로 첫 odom을 `initial_pose`에 맞춥니다.
- **공**: 2D 등속 + 마찰 감쇠 칼만 필터(`ball_filter.py`). 관측이 없으면 예측만 하고 공분산이 커집니다. 측정 표준편차는 거리 제곱에 비례해 커집니다. 마할라노비스 게이트로 튀는 값을 버리되 3번 연속이면 공이 옮겨진 것으로 보고 다시 잡습니다. 경기장 밖으로 예측되면 외벽 안쪽(공 반지름만큼)으로 제한하고 그 축 속도를 0으로 둡니다(비탄성 가정).
- **상태**: OBSERVED = 마지막 관측 후 `observed_timeout`(0.3 s) 이내, LOST = `lost_timeout`(5 s) 초과 또는 최대 표준편차 > `lost_sigma`(0.5 m), 그 사이 ESTIMATED.
- **ball_search_grid**: 셀마다 공 존재 믿음(0~1, 메시지는 0~100). 카메라가 살아 있고(최근 0.5 s 내 `camera_info`) 공이 OBSERVED가 아니면 시야 부채꼴 안 셀을 낮추고, 모든 셀은 시간이 지나면 사전값 0.5로 돌아갑니다. 검출되면 그 주변을 높입니다. 시야각은 `camera_info`의 K에서 계산, 거리 범위(0.15~1.5 m)는 가정. 상대에 의한 가림은 TODO.
- **상대**: 측정시각 TF·외벽/골대 제거·인덱스 연속 O(N) 군집 평균점 → 3/5 스캔 확정·단일 Alpha-Beta 추적. 미확정 전체 군집은 `world_model/obstacles`로 기존 MPC 회피에 별도 전달합니다. [설계·검증·실물 적용](../../docs/OPPONENT_TRACKING.md).
- **경기 시작**: `/game/playing`(std_msgs/Bool). 기본 `playing_default: false`.

## fake_world_model

로봇·시뮬레이터 없이 행동 트리와 UI를 시험합니다. 공은 경기장 안 타원을 천천히 구르고, 12초 주기로 OBSERVED → ESTIMATED(공분산 증가, 마지막 관측에서 외삽) → LOST를 반복합니다. 상대 1대는 상대 진영을 돌며 따로 상태가 바뀝니다. 나는 `/cmd_vel`이 오면 그대로 적분해서 움직이고(제어기 연결 확인용), 없으면 천천히 원을 그립니다.

## 테스트

기능 검사는 필터 상태·마찰·벽 제한·이상치·검색 그리드·기존 광선 투영·HLS/원호·거리 보존·보정 적용·라이다·Localization을 포함합니다. 실행 결과와 lint 범위는 각 검증 보고서에 기록합니다.

## 가정과 남은 일

- 필터·그리드·검출 수치는 모두 시작값 가정입니다. 카펫 위 골프공 구름(마찰 시간 상수)은 실측이 필요합니다.
- AMCL·골대 색/지자기 방향 확인은 연결됐으며 실물 간섭·HSV·지도 정합 검증이 필요합니다. wheel+gyro EKF와 공 시야 가림 처리는 후속입니다. 상대 추적은 공통 Alpha-Beta 경로를 사용하며 실물 정확도는 미검증입니다.

## 카메라 보정 UI의 공 검토

`pinky_soccer_calibration`의 **공 인식 검토** 탭은 경기용 `BallImageDetector`,
공통 `ball_vision.project_ball`/`annotate_ball`, `BallFilter`를 재사용합니다.
`geometry.detect_color_blob`도 같은 단일 프레임 검출을 호출합니다. 토픽/미터 투영 계약은 유지하며 원호 복원·시간 연속 검출도 공유합니다.
검토 탭은 JPEG를 사용하고 별도 검출/주행 토픽을 발행하지 않습니다.
설정·보정값 선택 및 비교 한계는 [ROS_UI.md](../pinky_soccer_calibration/ROS_UI.md)를 참고하세요.
