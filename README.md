# pinky-soccer
ROS 2 robot soccer project using Pinky Pro

## 패키지

| 패키지 | 역할 | 로봇에 필요 |
|---|---|---|
| `src/pinky_pro` | 공식 Pinky Pro (Git Submodule, **수정 금지**) | 일부 |
| `src/pinky_soccer` | 축구 제어 | O |
| `src/pinky_soccer_description` | 공식 모델 + 범퍼 (Wrapper Xacro) | O |
| `src/pinky_soccer_bringup` | 실물 로봇 실행 | O |
| `src/pinky_soccer_sim` | Gazebo / MuJoCo 실행 | X |
| `src/pinky_soccer_calibration` | 실물 보정 실험 기록·검증 (현재 카메라 내부 보정) | 실험 시 |
| `src/pinky_soccer_field` | 경기장 치수·기하(`map` 좌표), AMCL용 지도 생성 | O |
| `src/BehaviorTree.ROS2` | BehaviorTree.ROS2 (Git Submodule, 태그 `0.3.0`, **수정 금지**) | 판단 실행 시 |

판단 계층(월드 모델 → 행동 트리 → 제어 → UI)의 구조와 규칙은 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)를 따릅니다.

각 패키지 README에 실행 방법이 있습니다.

카메라 내부 보정은 [pinky_soccer_calibration/README.md](src/pinky_soccer_calibration/README.md)를 따릅니다.

RCJ 규격 축구 경기장은 두 시뮬레이터에서 `world_name:=rcj_soccer.world`로 실행합니다. [경기장 안내](src/pinky_soccer_sim/FIELD.md)를 참고하세요.
ROS 토픽 기반 기록·검증 도구와 초기 [Jupyter 보정 프로토타입](src/pinky_soccer_calibration/WORKBENCH.md)을 포함합니다.
자체 `pinky_soccer_bringup`에 로봇에서 직접 실행하는 카메라 발행 노드와 선택형 IMU·초음파·IR 구성을 추가했습니다. 실행 방법은 [bringup README](src/pinky_soccer_bringup/README.md)를 참고합니다.
PC 보정·YOLO와의 연결은 후속 작업이며 Jupyter 방식의 추가 개발은 중지했습니다. 신규 센서 실행 구성의 실물 검증은 아직 필요합니다.

Gazebo와 MuJoCo도 실물 기준 카메라·IMU·초음파·배터리 인터페이스를 제공합니다. IR은 거리 검출을 제공하며 ADC 출력에는 실측 보정표가 필요합니다. 공통 실행 옵션과 환경 간 점검 명령은 [시뮬레이션 센서 README](src/pinky_soccer_sim/README.md#실물과-공통인-센서-인터페이스)를 참고합니다.
체커보드 기준은 9×6 내부 코너, 사용자 확인 한 칸 30 mm입니다. 실제 보드 촬영·보정 정확도 검증은 아직 진행하지 않았습니다.

## 빌드

```bash
git clone --recursive https://github.com/cse1598/pinky-soccer.git
cd pinky-soccer
git config core.hooksPath .githooks   # 클론 후 한 번: 커밋 보호 훅 사용
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

로봇(라즈베리파이)에서는 `src/pinky_soccer_bringup/README.md`의 설치 방법을 따릅니다.

`src/BehaviorTree.ROS2`는 submodule(태그 `0.3.0` 고정, **수정 금지**)입니다. apt 패키지(`ros-jazzy-behaviortree-ros2`)가 Jazzy에 없어 submodule로 넣었습니다. 예제 패키지 빌드를 건너뛰려면 `colcon build --symlink-install --packages-skip btcpp_ros2_samples`를 씁니다.

### 커밋 보호

`.githooks/pre-commit`이 다음 커밋을 거부합니다. `--no-verify`로 우회하지 않습니다.

- `src/pinky_pro`, `src/BehaviorTree.ROS2` submodule 포인터 변경 또는 그 안의 수정. 사람이 지시한 포인터 변경(추가·업그레이드)만 `PINKY_ALLOW_SUBMODULE=src/BehaviorTree.ROS2 git commit ...`처럼 경로를 명시해 허용합니다.
- 로컬 전용 경로 `refs/`, `.agents/`, `AGENTS.md`, `CLAUDE.local.md`
- 10 MB를 넘는 파일 (원본 실험 데이터 방지용 기준값)

`refs/`는 참고용 외부 코드(`robocup_demo`, `BHumanCodeRelease`)를 로컬에 클론하는 곳입니다. git·colcon에서 제외되고(`.gitignore`, `refs/COLCON_IGNORE`) `.ignore`로 검색만 허용합니다. `.claude/settings.json`은 Claude Code가 `refs/`, `src/pinky_pro/`, `src/BehaviorTree.ROS2/`를 편집하지 못하게 막습니다.

## 남은 작업

- [ ] **바퀴 반지름 보정**: 실물 드라이버 파라미터는 0.027 m, URDF와 Gazebo DiffDrive는 0.028 m입니다. 실물에서 일정 거리를 주행해 odom과 실제 이동 거리를 비교해 값을 정하고, 실물(`wheel_radius` 인자)과 시뮬레이션에 같은 값을 반영합니다. 공식 URDF는 수정하지 않습니다.
- [ ] **카메라 기울기 보정**: `cam_tilt_deg`는 수평에서 아래로 기운 각도입니다. 실물이 약 8° 내려가 있어 모든 launch 기본값을 8°로 통일했습니다(공식 실물 구성은 0°). 정밀한 값은 카메라 캘리브레이션으로 확인합니다.
- [ ] **범퍼 디자인**: 공식 STL을 기반으로 범퍼를 모델링하고 출력합니다. 너비, 높이, 초음파 간섭, 공 크기를 함께 정합니다(`src/pinky_soccer_description/README.md`의 설계 제약 참고).
- [ ] **실물 bringup 검증**: 로봇에서 `pinky_soccer_bringup robot.launch.py`를 실행해 확인합니다(`src/pinky_soccer_bringup/README.md`의 확인 목록 참고).
- [x] **MuJoCo 통합**
- [ ] **Nav2 footprint**: 범퍼 치수가 확정되면 반영합니다.
- [ ] **1대1 시뮬레이션**: 로봇 2대를 띄울 수 있도록 namespace를 지원합니다.
