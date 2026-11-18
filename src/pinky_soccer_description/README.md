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
├── cad/bumper/                     # 범퍼 CAD 생성기 (cad/bumper/README.md)
│   ├── bumper_v1.yaml              #   설계 파라미터 (mm)
│   ├── generate_bumper.py          #   생성 + 간섭/삽입 검사
│   ├── blender_csg.py              #   Blender 형상 연산
│   └── print/                      #   출력용 STL과 검사 보고서
├── config/bumper_v1_params.yaml    # URDF 파라미터 (생성 파일, 직접 수정 금지)
├── meshes/visual/bumper_v1.stl     # URDF visual (생성 파일)
├── meshes/collision/bumper_v1_collision.stl
├── urdf/
│   ├── soccer_robot.urdf.xacro     # 공식 모델 + 범퍼
│   └── bumper.urdf.xacro           # insert_bumper 매크로
├── launch/view_robot.launch.py
└── test/test_xacro.py
```

RViz 설정은 공식 `pinky_description/rviz/view_robot.rviz`를 그대로 사용합니다.

## 범퍼

기본 범퍼는 **v1**입니다. 섀시 앞을 감싸는 ㄷ자 범퍼이고, 양쪽 갈고리를 앞쪽 기둥에 끼워 고정합니다(나사 없음). 형상, 장착 방법, 출력 방법은 [cad/bumper/README.md](cad/bumper/README.md)에 있습니다.

`config/bumper_v1_params.yaml`은 `cad/bumper/generate_bumper.py`가 만듭니다. 형상을 바꾸려면 `cad/bumper/bumper_v1.yaml`을 고친 뒤 다시 생성합니다.

### URDF 파라미터 (`bumper_params_file`)

| 키 | xacro 덮어쓰기 인자 | 의미 |
|---|---|---|
| `offset_x/y/z` | `bumper_offset_x/y/z` | base_link 기준 `bumper_link` 원점. CAD 범퍼는 무게중심 |
| `mass` | `bumper_mass` | 질량 (CAD 부피 × PLA 밀도 × 채움률) |
| `inertia` | — | 무게중심 기준 관성 (CAD에서 계산). 없으면 `width/depth/height` 박스로 근사 |
| `width`, `depth`, `height` | `bumper_width/depth/height` | 외곽 크기. 메시가 없을 때는 박스 형상으로도 쓰임 |
| `visual_mesh`, `collision_mesh` | — | 메시 URI. 빈 문자열이면 박스 형상을 사용 |
| `mesh_scale`, `color` | — | 메시 배율, RGBA |

활성화는 xacro 인자 `bumper_enabled`(기본 `true`)로 제어합니다. 다른 yaml을 쓰려면 `bumper_params_file:=<경로>`를 넘깁니다.

collision은 앞판 상자만 사용합니다. 옆벽과 갈고리는 로봇 외곽 안쪽에 있어서 공과 닿지 않고, MuJoCo는 메시를 볼록 껍질로 다루므로 상자 하나가 가장 정확합니다.

### 설계 제약 (base_link 기준, base_link는 지면 위 0.028 m)

- **질량**: 전방에 지지점이 없습니다(바퀴 x=0, 캐스터는 후방). x≈0.07 m에서 약 0.4 kg을 넘으면 정지 상태에서도 앞으로 넘어지고, 가감속 중에는 더 가벼워도 기울 수 있습니다. v1은 약 16 g입니다.
- **지면 간격**: 범퍼 하단은 지면에서 9 mm입니다(테스트로 5 mm 이상 확인).
- **센서**: 초음파 센서는 원뿔 구멍(반각 25°, 빔은 약 15°)으로 시야를 비웁니다. IR 센서(기판 아래)는 가리지 않습니다.
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
- namespace 규칙, yaml 값과 xacro 인자 덮어쓰기, 메시가 없을 때의 박스 형상과 관성
- v1이 기본값이고 설치된 메시와 CAD 관성을 쓰는지, 메시가 지면에서 5 mm 이상 떠 있고 바퀴보다 넓지 않은지
- 잘못된 값(0 이하) 거부, `check_urdf` 통과

## RViz2

```bash
ros2 launch pinky_soccer_description view_robot.launch.py
ros2 launch pinky_soccer_description view_robot.launch.py bumper_enabled:=false
ros2 launch pinky_soccer_description view_robot.launch.py gui:=false     # GUI 슬라이더 없이
ros2 launch pinky_soccer_description view_robot.launch.py rviz:=false    # 창 없이 TF만
```

## Nav2 footprint

공식 `pinky_navigation/params/nav2_params.yaml`의 footprint는 `base_footprint` 기준 ±0.06 m 사각형이고 padding은 0.03입니다. v1 범퍼는 x ≤ 0.0415, |y| ≤ 0.0385라 그 안에 들어갑니다. 범퍼가 이보다 커지면 공식 파일을 수정하지 말고, 자체 nav2 yaml을 만들어 launch의 `params_file:=` 인자로 넘깁니다.

## 검증 현황

| 항목 | 결과 |
|---|---|
| xacro 테스트, flake8, pep257, xmllint, lint_cmake | 통과 |
| RViz2 표시 (사용자 확인) | 통과 |
| 범퍼 CAD 간섭·삽입 검사 | `cad/bumper/README.md` 참고 |
| Gazebo / MuJoCo | `pinky_soccer_sim` README 참고 |
| 실물 | `pinky_soccer_bringup` README 참고 |

## 알려진 제한

- 생성된 URDF 맨 위에 xacro 파일의 설명 주석이 그대로 복사됩니다. 동작에는 영향이 없습니다.
- 워크스페이스 `install/`이 이미 source된 셸에서 빌드하면 colcon이 `pinky_description` underlay 경고를 냅니다. 동작에는 영향이 없습니다.
