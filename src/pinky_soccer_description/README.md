# pinky_soccer_description

Pinky Pro 로봇 모델에 축구용 전방 범퍼를 추가하는 Description 패키지입니다.

공식 `pinky_description`(`src/pinky_pro`, Git Submodule)은 **수정하지 않고** Wrapper Xacro로 확장합니다.

## 공식 Description과의 관계

```text
urdf/soccer_robot.urdf.xacro
 ├─ include  pinky_description/urdf/pinky.urdf.xacro   (공식, 수정 없음)
 ├─ include  urdf/bumper.urdf.xacro
 ├─ insert_robot(namespace, is_sim, cam_tilt_deg, screen_tilt_deg)
 │    └─ is_sim:=true 이면 공식 Gazebo 플러그인(DiffDrive, LiDAR, 카메라, IMU, 램프)도 포함
 └─ bumper_enabled:=true 이면 insert_bumper
      └─ bumper_link ── bumper_joint(fixed) ── base_link
```

- 인자 `namespace`, `is_sim`, `cam_tilt_deg`(기본 8), `screen_tilt_deg`(기본 -25)는 공식 `robot.urdf.xacro`와 같습니다.
- `bumper_enabled:=false`이면 생성되는 URDF가 공식 `robot.urdf.xacro`의 결과와 **같습니다**(테스트로 확인).
- 이름 규칙도 공식 코드를 따릅니다. 링크 이름에는 접두사가 없고(`bumper_link`), 조인트 이름에만 `namespace`가 붙습니다(`<ns>bumper_joint`).

## 폴더 구조

```text
pinky_soccer_description/
├── CMakeLists.txt
├── package.xml
├── config/bumper_params.yaml     # 범퍼 치수 (유일한 원본)
├── urdf/
│   ├── soccer_robot.urdf.xacro   # 공식 모델 + 범퍼
│   └── bumper.urdf.xacro         # insert_bumper 매크로
├── meshes/visual/                # 범퍼 visual 메시 (예정)
├── meshes/collision/             # 범퍼 collision 메시 (예정)
├── launch/view_robot.launch.py
└── test/test_xacro.py
```

RViz 설정은 공식 `pinky_description/rviz/view_robot.rviz`를 그대로 사용합니다.

## 범퍼 파라미터

값은 `config/bumper_params.yaml`에서 읽습니다. **현재 값은 시뮬레이션 검증용 임시값이며, 실제 범퍼 사양이 아닙니다.**

| 키 | xacro 덮어쓰기 인자 | 의미 | 임시값 |
|---|---|---|---|
| `width` | `bumper_width` | 너비 (y) | 0.12 m |
| `depth` | `bumper_depth` | 앞뒤 두께 (x) | 0.01 m |
| `height` | `bumper_height` | 높이 (z) | 0.02 m |
| `offset_x/y/z` | `bumper_offset_x/y/z` | base_link 기준 `bumper_link` 원점(박스 중심) | 0.046, 0, -0.010 |
| `mass` | `bumper_mass` | 질량 | 0.04 kg |
| `visual_mesh`, `collision_mesh` | — | 메시 URI. 빈 문자열이면 박스를 사용 | "" |
| `mesh_scale` | — | 메시 배율 (STL이 mm 단위면 0.001) | 1 1 1 |
| `color` | — | RGBA | 파랑 |

활성화는 xacro 인자 `bumper_enabled`(기본 `true`)로 제어합니다. 다른 yaml을 쓰려면 `bumper_params_file:=<경로>`를 넘깁니다.

### 설계 제약 (base_link 기준, base_link는 지면 위 0.028 m)

- **질량**: 전방에 지지점이 없습니다(바퀴 x=0, 캐스터는 후방). 바퀴 축 기준 기존 모멘트는 약 -0.031 kg·m라서, x≈0.07 m에서 약 0.4 kg을 넘으면 정지 상태에서도 앞으로 넘어집니다. 가감속 중에는 더 가벼워도 기울 수 있으니 실제 출력물 무게(수십 g)로 유지합니다.
- **지면 간격**: 범퍼 하단이 z = -0.028보다 위에 있어야 합니다. 목표는 5 mm 이상입니다(테스트로 확인).
- **센서**: 초음파 (0.0267, 0, 0.009)의 전방 시야와 IR 3개 (0.0295, 0 / ±0.02, -0.015)를 가리지 않아야 합니다.
  - 초음파는 원뿔 모양으로 퍼집니다(HC-SR04 계열은 반각 약 15°). 그래서 정면을 가리지 않아도 범퍼 윗면에 빔 아래쪽이 닿을 수 있습니다. 현재 임시 박스(윗면이 지면 2.8 cm, 센서에서 1.4 cm 앞)는 빔 하단과 겹칩니다.
  - 디자인 단계에서 정할 것: 축구에서 초음파를 쓸지(공이 3.7 cm보다 낮으면 초음파로는 공을 감지할 수 없음), 쓴다면 센서 앞부분을 낮추거나 파내거나 모따기할지. 시뮬레이션에는 초음파 센서가 없으므로 실물에서 확인합니다.
- **너비**: 현재 임시값 0.12 m는 바퀴 바깥면(±0.055 m)보다 양쪽으로 약 5 mm 넓습니다. 최종 너비는 디자인 단계에서 정합니다.
- **공 높이**: 범퍼가 공 중심 높이를 덮어야 공이 범퍼를 타고 넘지 않습니다. 현재 범퍼는 지면 0.008~0.028 m 구간을 덮습니다.
- **기존 형상**: `base_link` 충돌 메시의 앞면은 x = 0.0387, 바퀴 바깥면은 |y| ≈ 0.055입니다.
- **관성**: inertial의 `rpy`는 0이어야 합니다(`pinky_mujoco` 제약).

## 빌드와 테스트

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pinky_soccer_description
source install/setup.bash

colcon test --packages-select pinky_soccer_description
colcon test-result --verbose --test-result-base build/pinky_soccer_description
```

`test/test_xacro.py`에서 검사하는 항목:

- 범퍼 on/off와 `is_sim` true/false 조합이 모두 xacro 확장에 성공하는지
- 링크·조인트 이름이 중복되지 않는지, 트리의 루트가 `base_footprint` 하나인지
- 범퍼를 끄면 공식 모델과 같은지, 켜면 `bumper_link`와 `bumper_joint`만 추가되는지
- namespace 규칙, yaml 값과 xacro 인자 덮어쓰기
- 관성이 양수이고 삼각 부등식을 만족하는지, rpy가 0인지
- 지면 간격, 잘못된 값(0 이하) 거부, `check_urdf` 통과

## RViz2

```bash
ros2 launch pinky_soccer_description view_robot.launch.py
ros2 launch pinky_soccer_description view_robot.launch.py bumper_enabled:=false
ros2 launch pinky_soccer_description view_robot.launch.py gui:=false     # GUI 슬라이더 없이
ros2 launch pinky_soccer_description view_robot.launch.py rviz:=false    # 창 없이 TF만
```

치수를 바꾼 뒤 다시 launch하면 xacro가 새로 확장되어 반영됩니다. `--symlink-install`로 빌드했다면 yaml을 수정해도 다시 빌드할 필요가 없습니다.

## CAD 메시로 교체하기

1. 범퍼를 `bumper_link` 좌표계 기준으로 모델링합니다. 원점은 `offset_x/y/z`로 정한 장착점이고, x는 전방, z는 위쪽입니다.
2. `meshes/visual/bumper.stl`(또는 `.dae`)과 `meshes/collision/bumper.stl`을 넣습니다. collision 메시는 단순화한 버전이 좋습니다.
3. `bumper_params.yaml`에 경로를 적습니다.
   ```yaml
   visual_mesh: package://pinky_soccer_description/meshes/visual/bumper.stl
   collision_mesh: package://pinky_soccer_description/meshes/collision/bumper.stl
   mesh_scale: [0.001, 0.001, 0.001]   # STL이 mm 단위인 경우
   ```
4. 관성은 `width/depth/height/mass`로 계산한 박스 근사를 계속 사용합니다. 이 값을 메시 외곽 크기와 실제 출력물 무게로 갱신합니다.
5. 다시 빌드한 뒤 테스트하고 RViz로 확인합니다. 링크·조인트 구조는 바뀌지 않습니다.

Gazebo에서 `package://pinky_soccer_description` 메시를 찾으려면 `GZ_SIM_RESOURCE_PATH`에 이 패키지의 share 상위 폴더가 있어야 합니다. `pinky_soccer_sim/launch/gazebo_soccer.launch.py`가 이미 추가합니다.

## Nav2 footprint (범퍼 치수 확정 후)

공식 `pinky_navigation/params/nav2_params.yaml`의 footprint는 `base_footprint` 기준 ±0.06 m 사각형이고 padding은 0.03입니다. 임시 범퍼의 앞면은 x = 0.051, 너비는 ±0.06이라 현재는 그 안에 들어갑니다.

범퍼가 확정되면 공식 파일을 수정하지 말고, 자체 nav2 yaml을 만들어 launch의 `params_file:=` 인자로 넘깁니다. footprint는 `bumper_params.yaml`에서 계산해 범퍼 치수와 일치시킵니다.

## 검증 현황

| 항목 | 결과 |
|---|---|
| xacro 확장 / 중복 / 트리 / on-off / 관성 / 지면 간격 (pytest 16개) | 통과 |
| flake8, pep257, xmllint, lint_cmake | 통과 |
| `view_robot.launch.py` 헤드리스 실행: `/robot_description`, `/tf`, `/tf_static` 발행, `base_footprint → bumper_link` = (0.046, 0, 0.018) | 통과 |
| RViz2 화면 표시(형상·색·위치 확인) | 사람이 직접 확인해야 함 |
| RViz2 화면 표시 (사용자 확인) | 통과 |
| Gazebo | `pinky_soccer_sim` README 참고 |
| 실물 | `pinky_soccer_bringup` README 참고 |
| MuJoCo | `pinky_soccer_sim` README 참고 |

## 알려진 제한

- 현재 범퍼는 임시 박스이며, 치수는 실제 설계값이 아닙니다.
- 생성된 URDF 맨 위에 xacro 파일의 설명 주석이 그대로 복사됩니다. 동작에는 영향이 없습니다.
- 워크스페이스 `install/`이 이미 source된 셸에서 빌드하면 colcon이 `pinky_description` underlay 경고를 냅니다. 동작에는 영향이 없습니다.
