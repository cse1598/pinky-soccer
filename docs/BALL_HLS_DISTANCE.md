# HLS 공 검출 교체와 기존 거리·보정 보존 (2026-11-27)

검출만 HLS/원호 피팅으로 교체했다. **기존 `project_ball` 거리 함수는 소스 그대로 보존했고,
영상 반지름으로 새 거리를 계산하지 않는다.** 기존 K/D·장착값·공 크기·보정 원본은 수정하지 않았다.
선택한 검증 프로필의 공 XY 오프셋을 경기 실행 경로에 한 번 적용하도록 연결했다.
실물 로봇 주행/실제 거리 정확도는 미검증이며 재캘리브레이션을 수행하지 않았다.

## 1. 기존 코드에서 확인한 계약

| 항목 | 기존 동작 및 이번 유지 사항 |
|---|---|
| 검출 출력 | `BallObservation.blob=(u,v,r,area)` 또는 None. u/v는 원본 영상 픽셀, r은 **픽셀 반지름**, area는 px² |
| 영상 검출 | 이전 HSV/제한 WB/CLAHE/Sobel/Hough/원호 refit → 이번 HLS/색 윤곽/제한된 원 후보/기존 LS refit |
| 거리 입력 | `project_ball(blob, shape, K, D, camera_rt, output_rt, radius, max_range, rotated)` |
| 거리 출력 | 출력 프레임의 XYZ numpy 배열[m] 또는 None |
| 실제 사용 픽셀 | **blob의 u/v만 사용**. 영상 r/지름으로 focal×R/r 계산을 하지 않음 |
| 실제 공 크기 | 지름42.7mm, 물리 반경0.02135m. 바닥 좌표의 공 중심 높이 평면 z=0.02135m |
| 좌표계 | floor_frame 기본 base_footprint → output_frame 기본 base_link. PointStamped 시각은 원본 영상 시각 |
| 내부 보정 | camera_yaml(직접 camera YAML 또는 profile→launch.camera_info_url), 없으면 유효 CameraInfo K/D |
| 외부 보정 | 기존 센서 TF. 보정된 장착 높이/roll/pitch는 기존 bringup configure_urdf/TF 경로. 검토 UI는 기존 apply_mount 경로 |
| 미보정 | 기존 allow_uncalibrated 명시 옵션만 HFOV/zero-D 가정 허용. 기본은 K가 없으면 미터 출력 없음 |
| 실물/시뮬 | 같은 검출·투영 코드. 실제 K/D/TF 또는 렌더 CameraInfo/TF와 해당 환경 프로필만 구분 |

광선/투영 순서는 그대로다:

1. ROI 검출 u/v를 **전체 영상 좌표로 복원**한다(640×480이면 v에130 더함).
2. 기존 image_rotated_180 옵션이면 전체 영상 폭/높이를 기준으로 u/v를 반전한다.
3. `pixel_ray`가 비영 D일 때 `cv2.undistortPoints`로 **그 중심점**을 보정하고 단위 광선을 만든다.
4. 카메라 TF로 광선을 floor_frame에 표현하고 `intersect_plane_z(..., radius)`를 호출한다.
5. 기존 최대 거리/교점 유효 검사를 거친 뒤 output_rt로 변환한다.
6. 이번에 연결한 선택 오프셋을 출력 점에 한 번 더한다.

원본 이미지 전체를 미리 undistort하지 않는다. 따라서 영상 중심점을 중복 왜곡 보정하지 않는다.
픽셀 r은 신뢰도·표시에 남기되 물리 반경을 대신하지 않는다.
공이 바닥 위에 있고 floor/camera TF가 맞다는 기존 가정도 유지한다.

## 2. 경량 HLS·원호 피팅

공통 [설정](../src/pinky_soccer_world_model/config/ball_detector.yaml):
HLS 강한 색 범위(3,15,100)–(19,220,255), 낮은 채도 보조 범위(3,65,60)–(19,160,99).
H는 OpenCV0–180, L/S는0–255이며 설정 가능한 초기 범위다. 평균 RGB 한 점으로 검증된 색 분포라는 뜻은 아니다.
실물·MuJoCo·Gazebo 모두 같은 기본 범위를 쓴다. 기존 HSV/compact-shape 키 일부는
이전 YAML 호환을 위해 유지하며 새 마스크는 HLS 설정을 사용한다.

- 기본 y≥130 ROI에서 **한 번만** BGR→HLS 변환한다. 다른 해상도에서는 같은 높이 비율을 사용한다.
- 색 마스크 외곽에서 이미지/ROI 잘림의 인공 직선 경계를 제외한다.
- 기존 `circle_fit` 최소제곱 코드를 재사용하고, 필요할 때만 최대48개의3점 원 제안을 검사한다.
- 접선/반지름 방향·잔차·측정된 각도 구간을 확인해 상대 로봇의 가림 절단선을 원호로 쓰지 않는다.
- 기본 전체 원호 지지42%, 연속 원호25%, 상대 반지름 표준오차8% 등 조건을 만족할 때만 blob을 낸다.
  작은 래스터 원의 각도 구간 한 칸 결손은 연속성 검사에서만 잇고, 실제 관측 지지율은 늘리지 않는다.
- 충분히 설명되는 원은 무작위 복구를 실행하지 않는다. 현재 관측 근거 없는 예측 원을 발행하지 않는다.
- Hough·Sobel·CLAHE·영상 WB·딥러닝은 사용하지 않는다. 사용자 지시에 따라 손/얼굴 전용 분류 로직은 넣지 않았다.

잘림/가림이어도 충분한 원호면 전체 u/v/r을 추정한다. 복원된 중심은 화면 또는 ROI 밖일 수 있다.
짧은 원호는 `blob=None`로 두고 대략적인 **관측 원호 중심 픽셀**만 방향 정보로 제공한다.
그 픽셀은 정확한 전체 원 중심이나 미터 위치가 아니므로 거리에 넣지 않는다.

기존 `/ball_detection` PointStamped와 디버그 영상/마스크 토픽은 유지한다.
추가 상태 `ball_detection/status`(JSON String)는 원본 시각·bearing_valid·bearing_uv·
bearing_horizontal_px·circle_valid·range_valid·원호/반지름 불확실성·보정 출처를 제공한다.
circle_valid는 영상 원 피팅 조건, range_valid는 K/D·TF·기존 교점까지 통과한 최종 거리 조건이다.
방향 픽셀의 우측+ 부호는 기존180° 설정을 고려하며, 거리 불가 시 PointStamped에 가짜 r/NaN 거리를 넣지 않는다.
기존 World Model/BT/Skill은 방향 전용 후보를 새 미터 관측으로 사용하지 않는다.

## 3. 기존 실측 오프셋 분석·연결

원본 저장 위치는 프로젝트 meta의 `ball_review.offset_xy_m`이며, `session.yaml` 및
내보낸 human_verified `profile.yaml`에도 보존된다.
현재 확인한 실물 프로젝트는 `ros_20261123_195718_b5b380`이다.

| 원본 | 저장 값[m] | 이번 동작 |
|---|---|---|
| session/project meta | +0.013, 0 | 원본 유지, 실행 중 프로젝트를 자동 탐색하지 않음 |
| exports/6fa01d7d19d64855b878e2613e1d5824/profile.yaml | +0.013, 0, human_verified | 선택 시 읽기 전용 적용 |
| exports/b687f3020741455c9ee5afabd8cd6bbb/profile.yaml | 0, 0, human_verified | 그 파일을 선택하면0 그대로, 최신13으로 덮지 않음 |
| exports/37d9258a350543a88b263166faf6ad29/profile.yaml | 공 오프셋 없음 |0, 출처 상태에 no_ball_offset 표시 |

근거는 사용자 제공 범퍼 기준 중심 거리쌍100→87mm, 200→187mm다.
따라서 `offset = actual − observed = +13mm = +0.013m`이다.
원래 구현은 `compare_bumper_ball`에서 검토값만 보정했고 경기 BallDetector에는 읽기/적용 코드가 없었다.
프로필의 `purpose: review_only_not_applied_to_gameplay`는 그 이전 동작의 기록이다.
이번 사용자 요청으로 **선택한 export의 값만** 실행에 연결하며 그 기록/값은 고치지 않았다.

범퍼 앞면 중심을 f, base_link 원시 공 좌표를 p, 보정 벡터를 δ라 하면:

```
기존 검토: corrected_bumper_xy = (p_xy − f_xy) + δ_xy
이번 경기: corrected_base_xy   = p_xy + δ_xy
따라서 corrected_base_xy − f_xy == corrected_bumper_xy
```

δ는 base_link 축, X 전방+/Y 왼쪽+, 단위 m이다. 픽셀/반경/카메라 광축 오프셋이 아니다.
경기 출력에서 범퍼 위치를 또 빼지 않으며 Z는 그대로다.
다른 output_frame이면 base_link→출력의 **회전만** 보정 벡터에 적용한다(평행이동 중복 없음).
보정 UI의 `point`는 계속 raw이고 기존 비교 함수에서 δ를 한 번 더한다.
따라서 노드 보정값을 UI raw에 다시 가져와 두 번 더하는 경로를 만들지 않았다.

선택 우선순위:

- `ball_offset_source:=auto`: 기존 선택 calibration_profile, 없으면 camera_yaml/그 옆 profile.yaml.
- `ball_offset_source:=none`: 기존 raw 투영 그대로, 저장 값을 지우지 않는다.
- `ball_offset_source:=<profile.yaml>`: 그 검증 export만 읽는다.

offset 필드는 schema1/human_verified/real 또는 rehearsal 환경 및 유한 XY를 검사한다.
실물 보정을 시뮬에 자동 복사하지 않는다. 출처가 없으면0과 이유를 명시한다.
선택을 바꾸거나 보정 UI에서 새 값을 저장하는 것만으로 이미 켜진 경기 노드를 몰래 갱신하지 않는다.
같은 K/D·장착 TF에서 쓰는 보정값이어야 하며, 기존 장착값을 노드에 중복 적용하지 않는다.

## 4. 보존/비교 검증

- 동결한 [이전 거리 함수](validation/ball_hls_20261127/baseline_ball_vision.py)와 현재 함수의 소스 동일.
- 서로 다른u/v/r·D0/비영왜곡·180° true/false를 포함한 **192개 동일 입력**에서
  결과 bitwise 동일(무효 교점의 None도 동일). 같은 u/v에서 영상 r=5/35/150 변경도 거리 불변.
- 픽셀→광선/평면/좌표 변환인 geometry.py 전체, 기존 장착 helper, K/D·공 크기와 보정 원본 보존.
- 콜백 spy 검사: 실제 출력 직전에 오프셋 호출1회, X+13mm 및 다른 프레임 회전 보정 확인.
- UI의 범퍼 비교/저장/export 원본 보존 검사 유지. 원본 보정 파일8개 SHA256도 유지했다.
- 격리 ROS 실물 설정 노드에 기존6fa 프로필을 실제 로드해 K/D와 [0.013,0]m 선택을 확인했다.
  실제 로봇 센서 수신/구동 검사라는 뜻은 아니다.

동일 영상 비교의 합성 원은 bitmap 중심/반지름을 정답으로 사용한다.
투영 좌표는 그 카메라 모델을 거친 비교값이며 **실물 거리 정답이 아니다**.
기존 실물 프로필의 K/D/장착을 양쪽에 똑같이 적용했고, 시뮬 저장 영상은 기존 렌더 K/TF를 사용했다.

| 영상 | 이전 평균 ms | HLS 평균 ms | 새 중심 오차 px | 새 r 편차 px |
|---|---|---|---|---|
| 일반 r35 | 7.50 | 1.38 | 0 | −0.44 |
| 좌측 잘림 | 12.22 | 1.46 | 0.185 | −0.57 |
| 중심이 화면 밖 | 8.21, 원 미검출 | 1.79 | 0.197 | −0.26 |
| 아래 잘림 | 11.43 | 1.44 | 0.185 | −0.57 |
| ROI 경계 잘림 | 7.55, 원 미검출 | 1.37 | 0.139 | −0.54 |
| 짧은 원호 | 7.34, 원 미검출 | 18.70, 방향만 | 거리용 중심/r 없음 | 없음 |
| 부분 가림 | 12.65 | 19.00 | 0.184 | −0.57 |
| MuJoCo 저장 카메라 | 8.03 | 1.41 | 실제 정답 미제공 | 미제공 |
| Gazebo 저장 카메라 | 7.89 | 1.50 | 실제 정답 미제공 | 미제공 |

단일 PC·동일 입력8회 중 이후7회 처리 평균이다. 최초 획득 시간/P95/최대는 JSON에 별도 기록했다.
일반/잘림은 빨라졌지만 어려운 원호 복구가 모든 경우 더 빠른 것은 아니다.
아홉 개 거리 피팅 가능 합성 영상은 이전7/9, 새9/9이며 짧은 원호는 별도로 거리 거부했다.
새 bitmap 경계 r은 약0.26–0.57px 작게 맞춰졌으나 임의 +0.5px 같은 추가 보정은 하지 않았다.
**영상 r은 기존 거리식에 안 들어가므로 이를 거리 비례 오차로 해석하지 않는다.**
양쪽 모두 검출한 동일 영상에서 중심 변화에 따른 투영 XY 차이는 최대 약0.545mm였다.
이는 이 시험 입력들의 결과이며 실제 경기장 전체 정확도를 증명하지 않는다.

[수치 원본](validation/ball_hls_20261127/comparison.json) ·
[같은 영상의 이전/현재 원 표시](validation/ball_hls_20261127/comparison.png) ·
[처리 시간·반지름 편차 그래프](validation/ball_hls_20261127/timing_and_radius.png) ·
[비교 CLI](../scripts/compare_ball_detection.py).

## 5. 실제 실행 경로 검증

무제어/초기 보정 신호 전의 격리 시뮬에서 같은 경기 BallDetector를 실행했다.
MuJoCo와 Gazebo 모두640×480 BGR8/optical frame·CameraInfo 및 **PointStamped15개씩** 수신했다.
시뮬 보정은 각각의 기존 rehearsal 프로필에 공 오프셋이 없어0이며 원본 K/D를 사용했다.
[MuJoCo 원본 수신 결과](validation/ball_hls_20261127/live/mujoco.json),
[Gazebo 원본 수신 결과](validation/ball_hls_20261127/live/gazebo.json).

이 검사는 인터페이스/실행 연결 검사다. 실제 순간의 공·카메라 3D 정답과 대조하지 않았으므로
수신 성공을 거리 정확도 성공으로 보고하지 않는다. Gazebo의 이전 구동/자세 정합 문제도 이 작업에서 고치지 않았다.

World Model·Localization·공 검출·거리/보정 UI·launch 기능 회귀 **161 tests PASS**.
세 PC 패키지 symlink 빌드, 새/변경 코드 F 검사 통과. 전체 스타일 lint 통과를 주장하지 않는다.
MPC·BT·Skill·World Model 추적/거리 모델·물리·센서·Localizer 알고리즘은 이번에 변경하지 않았다.

## 6. 파일과 실행

- 신규 [hls_ball.py](../src/pinky_soccer_world_model/pinky_soccer_world_model/hls_ball.py),
  [ball_offset.py](../src/pinky_soccer_world_model/pinky_soccer_world_model/ball_offset.py).
- 기존 robust_ball.py: 공통 API/LS/시간 연결 재사용, 검출 경로 교체와 ROI/유효성 추가.
- ball_detector.py: HLS 설정·선택 보정·상태 연결. ball_vision.py: 방향 전용 표시만 추가(project_ball은 동일).
- config/ball_detector.yaml, 보정 ball_review.py, viz robot_stack.py 및 game/real_game/sim_field launch:
  공통 검출/선택 source 전달. 기존 UI 비교/저장 계산 자체는 보존.
- 신규 test_hls_ball_projection.py·비교 CLI·이 보고서/검증 자료, 관련 README/ROS_UI/ARCHITECTURE 갱신.

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --symlink-install \
  --packages-select pinky_soccer_world_model pinky_soccer_calibration pinky_soccer_viz
source install/setup.bash

# 하나씩 실행. 센서/공 검출은 시작 신호 전에도 확인 가능.
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco verify:=true
# Gazebo: mode:=gazebo
ros2 topic echo /ball_detection/status
```

실물은 기존 센서/장착 TF를 유지하고 PC만 재시작한다(이번 작업으로 핑키 내부 코드 교체 없음).
예를 들어 현재 확인한 실물 export를 명시적으로 선택하려면:

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=real verify:=true \
  camera_yaml:=$PWD/ros_20261123_195718_b5b380/exports/6fa01d7d19d64855b878e2613e1d5824/camera.yaml \
  ball_offset_source:=$PWD/ros_20261123_195718_b5b380/exports/6fa01d7d19d64855b878e2613e1d5824/profile.yaml
```

이 설정은 공 검출 K/D와 XY 보정만 선택한다. 휠/MPC 프로필을 몰래 바꾸지 않는다.
기존 실행 TF의 장착 높이59.608mm·pitch8.955° 등도 같은 보정 상태인지 실제 환경에서 확인해야 한다.
`ball_offset_source:=none`으로 같은 K/D/TF의 raw 결과와 비교할 수 있고 데이터 파일은 그대로다.
namespace를 쓰면 토픽에도 `/<robot_name>/` 접두사를 붙인다.

동일 입력 비교를 재실행하려면:

```bash
.venv/bin/python scripts/compare_ball_detection.py --output /tmp/pinky-ball-comparison \
  --real-profile ros_20261123_195718_b5b380/exports/6fa01d7d19d64855b878e2613e1d5824/profile.yaml
```

**남은 검증:** 실물 공의 여러 거리/좌우 위치·가림/잘림에서 기존 보정값으로 정확도를 확인한다.
100→87/200→187mm는 저장된 사용자 측정쌍이며 그때의 원시 공 영상/측정 행은 없어
새 검출기를 그 실측 영상에 재적용했다고 주장하지 않는다. 먼저 동일 K/D/장착/13mm 보정으로
확인하고, 그것으로 필요한 정확도가 나오지 않을 때만 기존 보정 UI로 재측정한다.
완전 가림은 복원하지 않고, 반사/렌즈 왜곡/동색 물체 전 상황의 성능은 미검증이다.
