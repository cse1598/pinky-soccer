# RoboCupJunior Soccer 경기장

별도 하드보드 경기장(내부 1820×1220mm, 외벽 200mm)은 **[HARDBOARD_FIELD.md](HARDBOARD_FIELD.md)**를 참고하세요. 기존 RCJ는 `field:=rcj`, 제2 경기장은 `field:=hardboard`로 선택합니다. 이 문서의 기존 규격·YAML·생성물은 그대로 유지합니다.

Blender/MCP 없이 `config/rcj_field.yaml`에서 치수를 읽어 Gazebo SDF와 MuJoCo MJCF를 함께 생성합니다. 공식 Submodule은 수정하지 않습니다. 규격 출처는 [Technical Specification For Soccer Fields (2026-06-03)](https://robocup-junior.github.io/soccer-rules/master/field_specification.html)이며, 사용자가 제시한 2023 도면의 주요 치수와 일치합니다. 공식 CAD 형상을 복사한 모델이 아니라 수치 규격 기반 자체 생성 모델입니다.

## 실행

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pinky_soccer_sim
source install/setup.bash

# 둘 중 하나만 실행
ros2 launch pinky_soccer_sim gazebo_soccer.launch.py world_name:=rcj_soccer.world imu_enabled:=true adc_enabled:=true
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py world_name:=rcj_soccer.world imu_enabled:=true adc_enabled:=true
```

기존 `empty.world` 기본값과 다른 공식 world는 유지합니다. `world_name`은 자체 패키지 `worlds/`를 우선 검색하고, 없으면 공식 world 경로를 사용합니다. `world:=/절대/경로/rcj_soccer.world`도 가능합니다. MuJoCo에서는 같은 폴더의 `rcj_soccer.mjcf`와 `rcj_meshes/`도 필요하므로 폴더 전체를 복사하세요.

기존 카메라·LiDAR·IMU 등 토픽과 보정 UI를 그대로 사용합니다. `/map`을 발행하는 Nav2 점유 지도나 자동 심판/득점 판정은 포함하지 않습니다. 기존 `maps/` 사용자 지도는 변경하지 않습니다. 기본 로봇은 원점, 공은 +X 0.3m에 생성됩니다. Gazebo GUI에서 경기장이 화면 밖이면 모델 목록에서 `rcj_field`를 선택해 카메라 초점을 맞추세요.

## 치수·좌표

단위는 m입니다. 경기장 중심이 (0,0,0), 평평한 바닥 상단이 z=0, **+X 파랑 골대 / -X 노랑 골대**입니다.

| 항목 | 값 |
|---|---|
| 경기 구역 (흰 경계선 바깥 가장자리 기준) | 길이 2.19 × 폭 1.58 |
| 외곽 여유 공간 | 각 방향 0.12 |
| 벽 안쪽의 전체 바닥 | 길이 2.43 × 폭 1.82 |
| 검정 외벽 | 높이 0.22 |
| 골대 내부 | 폭 0.60 × 높이 0.10 × 깊이 0.074 |
| 외곽 경사면 | 밑변 0.10, 높이 0.02 (허용범위의 명목값) |
| 흰 선 | 폭 0.02, 경기 구역에 포함 |
| 페널티 구역 | 깊이 0.25 × 폭 0.80, 바깥 모서리 반경 0.15 |
| 중앙 검정 원 | 지름 0.60 |
| 중립점 | 5개, 지름 0.01 |

외곽 경사면은 코너에서 45° 이음으로 만나며, 골대 내부와 뒤쪽 통로에는 없습니다. 골대 뒤의 외벽도 유지합니다. 벽·골대·경사면·바닥에는 충돌이 있고, 흰 선·중앙 원·중립점·골대 바닥 색칠은 충돌이 없는 시각 요소입니다. 선을 넘을 때 턱이 생기지 않습니다.

Pinky의 LiDAR 높이는 골대 높이보다 높아 골대 구조 위를 지나 외벽을 볼 수 있습니다. 공과 골대 인식에는 카메라를 함께 사용하세요.

## 파일과 재생성

- `config/rcj_field.yaml`: 공식 치수와 별도로 표시한 시뮬레이션 가정.
- `pinky_soccer_sim/rcj_field.py`: 공통 형상 생성기와 자체 MuJoCo 로딩 래퍼.
- `worlds/rcj_soccer.world`: Gazebo world(필수 physics/sensors/IMU 플러그인 포함).
- `worlds/rcj_soccer.mjcf`: 동일 형상의 MuJoCo 모델. 단독 컴파일/렌더링도 가능.
- `worlds/rcj_meshes/`: 두 엔진에서 공유하는 경사면 convex binary STL 6개.
- `worlds/rcj_field_manifest.yaml`: 생성에 사용한 규격·단위·가정.

```bash
ros2 run pinky_soccer_sim generate_rcj_field \
  --spec src/pinky_soccer_sim/config/rcj_field.yaml \
  --output src/pinky_soccer_sim/worlds
```

MuJoCo의 공식 SDF 변환기는 수정하지 않고, 이 world에 한해서 동일 생성 데이터의 MJCF를 로봇 scene에 병합합니다. 원래 평면 바닥은 제거하고 녹색 바닥 collision으로 교체합니다. 로봇 freejoint 순서·동적 골프공·센서 래퍼는 유지합니다. 다른 world의 기존 변환 동작은 그대로입니다.

## 가정과 한계

벽/판 두께(0.01/0.02m), 얇은 검정 선 폭(0.002m), 색상·조명·마찰은 구현 가정입니다. 카펫 섬유·탄성·실제 골프공의 구름 저항·반발계수는 실측하지 않았습니다. 기본 마찰 0.8은 시뮬레이션용 값이며 실물 보정값이 아닙니다. 엔진별 접촉 모델도 다르므로 동일한 치수라고 동일한 실물 물리를 보장하지 않습니다. 골대 구조 전체에 해당 골대 색을 사용하며, 실물 외장 도색까지 정밀 재현한 것은 아닙니다.

호의 선은 작은 사각형 구간으로 분할합니다(원 기준 128개). 코너 경사면은 실제 충돌 메시입니다. 표시선은 수치적으로 바닥 위 약 0.3mm에 놓지만 충돌하지 않습니다. 자체 침투/클리핑 등의 문제가 있으면 world의 치수와 로봇 spawn 위치를 먼저 확인하세요. 실제 대회장 소재·조명 편차는 별도 실측/캘리브레이션 대상입니다.

## 검사

- 생성 파일 재현성, SDF/MJCF 객체 수·충돌 유무 및 규격 치수 검사.
- MuJoCo: 공의 골대 진입/뒷벽 충돌, 경사면을 따라 안쪽으로 구르기, LiDAR 외벽 거리, 표시선의 비충돌 검사.
- 실제 로봇 모델과 world 병합 시 robot freejoint 순서 유지 검사.
- 별도 Gazebo/MuJoCo launch에서 카메라·센서 수신 및 공통 `check_interface` PASS 확인.
- Gazebo에서는 로봇/공이 안정화된 뒤 공을 경사면 위(z=0.04m)에 배치하고, sim time으로 6초 이상 진행한 위치 토픽에서 안쪽으로 내려온 것을 확인했습니다. 경사면 내부에 겹쳐 생성하면 초기 접촉이 불안정하므로 검사 시 충분한 높이에 배치해야 합니다.
- MuJoCo 오프스크린 렌더링으로 선·골대·경사면 배치 확인. 실물 규격 인증 또는 실제 카펫 성능 검증은 아닙니다.
