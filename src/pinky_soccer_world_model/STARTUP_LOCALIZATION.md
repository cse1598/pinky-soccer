# 공통 경량 Localization: 시작·운영 절차

## 현재 기본 경기 운영

기본 통합 launch는 [경기 준비·LED 관리자](../../docs/MATCH_PREPARATION.md)를 사용한다.
런치 후 정지/센서 수신 → **경기 준비 버튼 또는 preparation/prepare** → 기존 보정 →
자기 팀 시작 위치 GoToPose → 실제 정지 확인 → **READY에서 팀 LED** → 별도 경기 시작이다.
보정 직후에는 팀 LED로 바뀌지 않고 시작 위치 이동 중에도 초록 점멸한다.
경기 준비 창의 시작 버튼은 READY 전에는 비활성이다. 기존 보정 창 R/S도 준비/정지로 연결된다.

```bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco team:=Y
ros2 service call /preparation/prepare std_srvs/srv/Trigger '{}'
ros2 topic echo /preparation/status
ros2 service call /preparation/start std_srvs/srv/Trigger '{}'
ros2 service call /preparation/stop std_srvs/srv/Trigger '{}'
```

실물의 한 명령 센서·LED 실행은 위 보고서의 robot_host와 로봇 bringup 전달/빌드 절차를 따른다.
**아래 직접 localizer 운영 설명은 `preparation:=false`의 단독 분석/검증 경로다.**
알고리즘 자체는 기본 준비 경로에서도 동일하며 LED/최종 주행 허용은 준비 관리자가 소유한다.


2026-11-27: 실물·MuJoCo·Gazebo에서 같은 `rectangle_localization`을 사용한다.
기존 PC 시작보정의 외벽 정합·TF·명령 게이트·LED를 재사용했다.
구현 원리·변경 전후 평가·남은 문제는 [통합 보고서](../../docs/LIGHTWEIGHT_LOCALIZATION.md)에 있다.
**Gazebo는 기존 구동 정체 때문에 회전이 필요한 초기화/주행 검증이 미완료다. 실물은 미검증이다.**

## 시작과 제어 소유권

런치/로봇 연결만으로 제어하지 않는다. `WAITING_FOR_CALIBRATION`에서 센서 수신만 한다.
`localization/calibrate` Trigger 또는 창의 R 뒤에만 보정 회전·초록 LED 점멸을 시작한다.
신호 전 cmd_vel·LED 요청·map→odom 발행은 없다. 자동 경기 시작도 없다.

1. LiDAR·IMU·odom·카메라 TF/신선한 영상을 기다린다.
2. 선택 경기장 규격에 맞는 네 외벽을 찾고 두 대칭 후보를 유지한다.
   내부 장애물은 위치 추정 사본에서 제외한다. 원본 `scan`은 변경하지 않는다.
3. 두 짧은 변 중점을 보는 상대 회전각을 비교해 절댓값이 작은 쪽을 선택한다.
   거리가 가까운 골대 선택이 아니다. 기본 0.3rad/s, 설정 및 기존 max_ang cap 안에서 회전한다.
4. 각도 오차가 기본 0.035rad 이내이고 gyro가 0.05rad/s 이하이면 정지한다.
   0.5초 안정화 뒤 촬영된 영상에만 HLS를 적용한다.
5. 하단350px(640×480의 y≥130)만 한 번 BGR→HLS 변환하고 두 마스크 픽셀을 센다.
   기본 500px 이상·면적비3 이상·동일 결과3프레임이면 확정한다.
   실패하면 반대편 중점을 한 번 시도한다. 모두 실패하면 `GOAL_COLOR_FAILED`에서 정지한다.
6. 한 색으로 두 대칭 후보 중 하나를 고르고 기하학적으로 보정한 IMU 양축을 계산한다.
   노랑 −X/파랑 +X를 유지한다. 팀 색은 관측 골대 색과 별개다.
7. 팀 LED를 켜고 `READY`로 전환한다. 이후 골대 영상 디코딩·HLS 검출을 중단한다.

`MPC → localization/controller_cmd_vel → rectangle_localization → cmd_vel → 기존 watchdog`.
준비 후에는 MPC 명령을 즉시 그대로 전달한다. MPC solver·가중치·주기·안전 조건은 그대로다.

## 경기 중 추적과 실패 상태

외벽 정합 기본 10Hz. IMU로 전달한 이전 각도 주변에서 작은 직선 최소제곱 보정을 먼저 시도하며,
지지점·잔차 조건을 만족하지 않으면 기존 각도 탐색을 사용한다. 네 벽을 관측하지 못하면
정합을 만들어 내지 않는다. IMU 예상 yaw로 대칭 후보를 고른 뒤 odom 예측과의 위치/각도
차이를 검사한다. 수용한 외벽 정합으로 IMU offset을 천천히 보정한다.

LiDAR 사이에는 wheel odom 이동과 IMU 각도 증분으로 예측한다. 기본 최대0.8초만 유효하며,
신뢰도는 시간 경과에 따라 감소한다. 그 뒤 `LOCALIZATION_LOST`로 명령을 막고,
3초가 지나면 `REINITIALIZATION_REQUIRED`를 유지한다. R/Trigger 없이 다시 회전하지 않는다.
IMU 단발 이상치는 버리고, 자이로 예측과 맞지 않는 연속3회 불연속/기준 단절은 중단한다.
신뢰도는 경험적 품질 지표이며 보정된 확률이 아니다.

시작 입력 준비 뒤 60초 또는 누적2.5π 회전 한도를 넘으면 정지한다.
S/`localization/abort`는 명시적 중단이며 자동 해제하지 않는다.
LED는 async 호출이므로 서비스 부재/지연이 위치 추정을 막지 않는다.

## 설정

공통 설정: [rectangle_localization.yaml](config/rectangle_localization.yaml).
요청 HLS 기본값은 노랑 (24,85,180)–(35,180,255), 파랑 (96,30,135)–(115,130,255)다.

`localization_config:=auto`: 실물은 공통값, 시뮬은
[실측 렌더 HLS override](config/rectangle_sim_hls.yaml)를 추가한다.
기존 조명의 MuJoCo 영상은 S가 낮아 요청 기본값으로 검출되지 않는다.
시뮬 하한만 노랑 (24,60,100), 파랑 (96,20,80)으로 보정했으며 H/상한/면적/비율/연속수/ROI는 유지했다.
근거 원본은 [색상 측정](../../docs/SIMULATION_COLORS.md)에 있다. 알고리즘은 세 환경에서 동일하다.

`localization_config:=none`이면 시뮬도 요청 기본 HLS만 사용한다.
사용자 YAML 경로를 넘기면 공통 설정 위에 덮는다. 키는 `/**/rectangle_localization: ros__parameters:` 형식이다.
실물 HLS·노이즈 임계값 변경은 실제 측정 뒤 결정한다. K/D는 이 픽셀 수 방식에는 불필요하며,
기존 공 검출기의 내부 보정 요구는 유지한다. 카메라 광축·위치 TF는 필수다.

## PC 빌드·시뮬 실행

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --symlink-install \
  --packages-select pinky_soccer_world_model pinky_soccer_viz
source install/setup.bash
ros2 launch pinky_soccer_viz game.launch.py mode:=mujoco field:=hardboard verify:=true \
  localization_debug:=true
# 위 실행을 종료한 뒤 Gazebo: mode:=gazebo
# 명시적 이전 AMCL 경로: localization:=lidar (동시에 실행하지 않음)
```

별도 PC 터미널(같은 ROS/domain 환경) 또는 창의 R로 시작한다:

```bash
source install/setup.bash
ros2 service call /localization/calibrate std_srvs/srv/Trigger '{}'
ros2 service call /localization/abort std_srvs/srv/Trigger '{}'
ros2 topic echo /localization/status
```

`robot_name:=yellow`이면 토픽/서비스도 `/yellow/...`다.
S=중단, R=시작/재보정, ESC=모니터 닫기다. `headless:=true`면 같은 서비스/상태만 사용한다.

## 실제 핑키와 PC 실행 (실물 미검증)

이번 변경으로 **로봇 내부 소스 교체는 필요 없다**. 기존 설치와 ROS/domain/DDS 설정을 유지한다.

```bash
# 핑키 터미널: 기존 bringup과 중복 실행하지 않는다.
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
# 축구 wrapper를 설치한 기존 워크스페이스 setup도 source
ros2 launch pinky_soccer_bringup robot.launch.py imu_enabled:=true
# 기존 LED 서비스가 없을 때만, 핑키에서 실행
ros2 run pinky_led led_server
```

PC에서:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pinky_soccer_viz game.launch.py mode:=real field:=hardboard team:=Y verify:=true \
  camera_yaml:=/absolute/path/camera.yaml localization_debug:=true
# 이후 같은 calibrate Trigger/R. 기존 calibration_profile/robot_name 설정을 그대로 추가한다.
```

`pinkylib` LED 드라이버는 핑키 GPIO용이다. PC에 동명 pip 패키지를 설치하지 않는다.
기존 `pinky_interfaces/SetLed`의 fill(RGB 밝기96), 없으면 SetLamp를 async로 사용한다.
실물 LED 발광·회전·센서 축·시간 동기화·외벽 가림·HLS는 현장 검증이 필요하다.

## 출력·디버그·저장

외부 형식은 기존 `localization/pose`(PoseWithCovarianceStamped, map),
`localization/ready`(Bool), `localization/status`(JSON String), map→odom TF다.
World Model은 기존 TF 모드와 같은 데이터 형식을 사용한다. 이 경로에서 AMCL/goal_heading을 중복 실행하지 않는다.
Ground Truth는 이 노드의 구독/계산에 없다. 평가 도구·기존 UI만 정답을 사용한다.

`localization_debug:=false`가 기본이다. true일 때만 boundary_scan, map_image,
goal_image(HLS 마스크), observed_map을 만든다. map_image에는 원본 관측의 외벽/제외점,
직사각형, 양 후보, 골대 중점, 확정 pose가 표시된다. 제외점은 원본 scan에도 그대로 남는다.
OccupancyGrid는 기존 디버그 출력만 보존하며 추정에는 사용하지 않는다.

status에는 양 후보·확정 pose·선택 중점·목표 각도·HLS 픽셀 수/연속수·IMU offset·
신뢰도·유효 상태·초기화 완료·정합 시간 평균/P95/최대·거부 횟수를 담는다.
시뮬 LED 하드웨어 대신 같은 상태 전환을 여기서 확인한다.

세션별 고유 폴더에 initial_scan.csv/yaml과 heading_reference.yaml을 저장한다.
기본 경로는 `~/.ros/pinky_soccer/localization/`, heading_record 지정 시 그 부모 아래다.
양축·관측/유도 여부·raw IMU·offset·초기 pose·팀·TF 출처·시각을 기록한다.
외곽 PGM/final_scan/observed_map은 debug 활성화 때만 생성하며 추정 입력으로 읽지 않는다.
