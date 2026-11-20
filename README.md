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

각 패키지 README에 실행 방법이 있습니다.

카메라 내부 보정은 [pinky_soccer_calibration/README.md](src/pinky_soccer_calibration/README.md)를 따릅니다.
ROS 토픽 기반 기록·검증 도구와 초기 [Jupyter 보정 프로토타입](src/pinky_soccer_calibration/WORKBENCH.md)을 포함합니다.
현재 방향은 자체 `pinky_soccer_bringup`에 카메라 발행 노드를 추가하고 PC에서 직접 구독하는 구성입니다. 이 연결은 아직 미구현이며 Jupyter 방식의 추가 개발은 중지했습니다.
체커보드 기준은 9×6 내부 코너, 사용자 확인 한 칸 30 mm입니다. 실제 보드 촬영·보정 정확도 검증은 아직 진행하지 않았습니다.

## 빌드

```bash
git clone --recursive https://github.com/cse1598/pinky-soccer.git
cd pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

로봇(라즈베리파이)에서는 `src/pinky_soccer_bringup/README.md`의 설치 방법을 따릅니다.

## 남은 작업

- [ ] **바퀴 반지름 보정**: 실물 드라이버 파라미터는 0.027 m, URDF와 Gazebo DiffDrive는 0.028 m입니다. 실물에서 일정 거리를 주행해 odom과 실제 이동 거리를 비교해 값을 정하고, 실물(`wheel_radius` 인자)과 시뮬레이션에 같은 값을 반영합니다. 공식 URDF는 수정하지 않습니다.
- [ ] **카메라 기울기 보정**: `cam_tilt_deg`는 수평에서 아래로 기운 각도입니다. 실물이 약 8° 내려가 있어 모든 launch 기본값을 8°로 통일했습니다(공식 실물 구성은 0°). 정밀한 값은 카메라 캘리브레이션으로 확인합니다.
- [ ] **범퍼 디자인**: 공식 STL을 기반으로 범퍼를 모델링하고 출력합니다. 너비, 높이, 초음파 간섭, 공 크기를 함께 정합니다(`src/pinky_soccer_description/README.md`의 설계 제약 참고).
- [ ] **실물 bringup 검증**: 로봇에서 `pinky_soccer_bringup robot.launch.py`를 실행해 확인합니다(`src/pinky_soccer_bringup/README.md`의 확인 목록 참고).
- [x] **MuJoCo 통합**
- [ ] **Nav2 footprint**: 범퍼 치수가 확정되면 반영합니다.
- [ ] **1대1 시뮬레이션**: 로봇 2대를 띄울 수 있도록 namespace를 지원합니다.
