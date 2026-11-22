# 제2 경기장: 하드보드 4장

`config/hardboard_field.yaml`이 이 경기장의 치수 원본입니다. 기존 `rcj_field.yaml`과 RCJ 생성물은 유지합니다. 610×910mm 판을 2×2로 놓아 **외벽 안쪽 전체 바닥 1820×1220mm**를 구성합니다. 높이는 사용자 정정에 따라 **외벽 200mm(20cm)**입니다. 원점·축은 기존과 같습니다: 바닥 중앙 `(0,0,0)`, +X 파란 골대, −X 노란 골대.

## 제작 치수

아래는 공식 대회 규격이 아니라 이 판 크기에 맞춘 연습용 설계값입니다. 라인·골대는 기존 RCJ 기준의 약 2/3 크기를 제작하기 쉬운 mm 단위로 반올림했습니다. 판의 종횡비가 달라 전체를 한 비율로 단순 축소하지는 않았습니다. 원과 모서리 호는 원형을 유지합니다.

| 항목 | 치수(mm) |
|---|---|
| 벽 안쪽 전체 바닥 | **1820×1220** |
| 외벽 높이 | **200** |
| 흰 경계선 바깥 가장자리 기준 경기 구역 | **1660×1060** |
| 경계선 밖 여백 | 각 방향 **80** |
| 흰 선 폭 | **15**, 경기 구역 안에 포함 |
| 골대 내부 폭×높이×깊이 | **400×70×50** |
| 페널티 구역 폭×깊이 | **540×170** |
| 페널티 모서리 반경 | **100** |
| 중앙 원 지름 | **400** |
| 중립점 지름 | **7** |
| 중립점: 골라인에서 안쪽 거리 | **300** |
| 외곽 경사면 | 없음, 평평한 바닥 |

자로 배치할 때 중앙을 기준으로 외벽 안쪽은 X=±910, Y=±610입니다. 흰 경계선의 바깥 가장자리는 X=±830, Y=±530이고, 골대 입구는 X=±830, Y=−200~+200입니다. 골대 뒷면의 안쪽은 X=±880입니다. 페널티 구역은 골라인에서 안쪽으로 170mm까지, 중립점은 중앙과 `(X,Y)=(±530,±270)`에 놓습니다. 골대 뒤 공간도 판 위 전체 크기에 포함되며 판 밖으로 골대를 돌출시키지 않습니다.

사용자 실측 지름 42.7mm를 적용하므로 골대 입구 높이 70mm·깊이 50mm에 들어갑니다. 골대 판 두께는 사용자 확인값 **5mm**(`goal_thickness`)이며, 뒷판을 옆판 사이에 넣고 지붕을 위에 얹는 맞댐 구조입니다. 외벽 두께 10mm와 바닥 모델 두께 20mm는 아직 **구조 가정**이며 하드보드 실측 두께가 아닙니다. 외벽/바닥 실제 두께를 잰 뒤 `wall_thickness`/`floor_thickness`를 수정하고 재생성하세요. 바닥 윗면은 항상 Z=0입니다.

판 이음매 2개는 시뮬레이션에 요철로 만들지 않았습니다. 실제 판은 이음매·테이프가 공의 진행을 바꾸지 않게 평평하게 연결하세요. 마찰 0.8·표면 색·조명도 기존 시뮬레이션 가정이며 실제 하드보드 물성을 측정했다는 뜻이 아닙니다.

## 경기장 선택

```bash
source /opt/ros/jazzy/setup.bash
cd ~/pinky-soccer
colcon build --symlink-install --packages-select pinky_soccer_sim pinky_soccer_field pinky_soccer_viz
source install/setup.bash

# Gazebo 또는 MuJoCo 중 하나만 실행
ros2 launch pinky_soccer_sim gazebo_soccer.launch.py field:=hardboard imu_enabled:=true adc_enabled:=true
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py field:=hardboard imu_enabled:=true adc_enabled:=true

# 기존 RCJ 경기장
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py field:=rcj

# 판단 노드·표시까지 함께 실행 (모든 계층이 같은 YAML 사용)
ros2 launch pinky_soccer_viz sim_field.launch.py field:=hardboard
ros2 launch pinky_soccer_viz sim_field.launch.py field:=hardboard simulator:=gazebo
# 센서/물리 없는 가짜 월드 모델 표시 연습
ros2 launch pinky_soccer_viz sim_viz.launch.py field:=hardboard
```

기존 `world_name:=rcj_soccer.world`와 새 `world_name:=hardboard_soccer.world`도 가능합니다. 단, 판단/표시를 별도로 실행하면 그 노드의 `field_spec`에도 같은 YAML 경로를 지정해야 합니다. `sim_field.launch.py field:=hardboard`는 이를 자동으로 맞춥니다. 충돌하는 `field`와 `world/world_name`을 동시에 주면 오류로 알립니다. 시뮬레이터 단독 launch의 기본값은 여전히 `empty.world`이고, 판단 계층 launch 기본값은 여전히 RCJ입니다.

실물 `robot.launch.py`는 센서·모터 실행용이라 경기장 옵션이 필요하지 않습니다. 실물 판단·표시 노드의 `field_spec`을 이 YAML 사본으로 지정하세요. 로봇에 시뮬레이터 전체를 설치할 필요는 없습니다.

## 생성·지도 내보내기

```bash
ros2 run pinky_soccer_sim generate_rcj_field \
  --spec src/pinky_soccer_sim/config/hardboard_field.yaml \
  --output src/pinky_soccer_sim/worlds

# 필요할 때 LiDAR 높이를 고려한 AMCL용 점유 지도 생성
ros2 run pinky_soccer_field generate_field_map \
  --spec src/pinky_soccer_sim/config/hardboard_field.yaml \
  --output /tmp/hardboard_map
```

생성물은 `worlds/hardboard_soccer.world`(Gazebo), `hardboard_soccer.mjcf`(MuJoCo), `hardboard_field_manifest.yaml`입니다. 두 엔진은 같은 형상 목록에서 생성합니다. 외벽 200mm는 기본 LiDAR 높이 125mm보다 높고, 골대 70mm는 LiDAR 아래에 있으므로 기본 점유 지도에는 외벽이 반영됩니다. 시뮬레이터의 시각적 경기장과 Nav2 점유 지도는 별도이며, 자동 AMCL 시작이나 득점 심판을 추가한 것은 아닙니다.

## 검증 (2026-11-21)

기존 RCJ 생성물의 바이트 재현성, 새 경기장 내부 치수·골대 여유·경사면 없음, MuJoCo 공의 축소 골대 진입·외벽 LiDAR 거리·경기장 전환 시 기존 형상 제거를 확인했습니다. Gazebo에서 `field:=hardboard`의 공통 카메라/IMU/ADC 인터페이스 검사가 통과했고, MuJoCo 통합 launch에서는 카메라·LiDAR·월드 모델·탑뷰 수신과 골대 X=±0.83m, 월드 모델/두 UI의 동일 YAML 사용을 확인했습니다. 점유 지도는 네 외벽을 포함한 194×134셀(해상도 1cm)로 생성됐으며, 오프스크린 렌더링도 검토했습니다. 실제 하드보드 조립 치수와 마찰·반발·공 구름은 아직 실측 검증하지 않았습니다.

## 제작 도면과 A4 출력 본

제작 PDF는 저장소 루트의 `output/pdf/hardboard_fabrication_pack.pdf`입니다. 전체 13쪽이며 PDF 책갈피로 세 부분을 선택할 수 있습니다.

- 1~4쪽: 전체 배치(1:10), 줄자용 좌표표, 5mm 판 골대 조립/재단표, 1:1 페널티 코너 본.
- 5~7쪽: 골대 부품의 1:1 재단 본. 5~6쪽은 20mm 겹쳐 십자를 맞추고, 7쪽은 옆판 4개입니다.
- 8~13쪽: 중앙 원 1:1 본(3행×2열). 15mm 겹침과 빨간 십자로 정렬합니다. 검정 선만 옮기고 빨간 기준표시는 칠하지 않습니다.

**A4 가로, 실제 크기 100%, 페이지에 맞추기 OFF**로 출력합니다. 각 본의 100mm(옆판 본은 80mm) 기준선을 자로 확인한 뒤 사용하세요. 중앙 원 Ø400은 선중심 기준이며 검정 선 폭 2mm이므로 바깥 지름은 402mm입니다. 페널티 곡선은 바깥 R100/안쪽 R85로, 흰 띠 폭 15mm를 유지합니다.

골대 2개는 뒷판 400×70mm 2장, 옆판 55×70mm 4장, 지붕판 410×55mm 2장입니다. 모두 5mm 판이며 뒷판은 두 옆판 사이, 지붕은 그 윗면에 얹습니다. 바닥은 경기장 판을 쓰므로 별도 바닥판은 없습니다. 접착 후 내부 폭400/높이70/깊이50을 확인하세요. 접착제 두께·칼날 손실을 임의로 더하지 않았습니다. 완성 부품 치수선의 바깥(버릴 쪽)으로 절단합니다.

생성기는 `scripts/generate_hardboard_drawings.py`입니다. reportlab·pypdf·PyYAML이 설치된 별도 Python 환경에서 실행하고, 한글 TrueType 글꼴(예: NanumGothic-Regular.ttf)을 지정합니다. 실행 중 로봇의 Python 환경은 변경하지 않습니다.

```bash
python scripts/generate_hardboard_drawings.py \
  --spec src/pinky_soccer_sim/config/hardboard_field.yaml \
  --font /경로/NanumGothic-Regular.ttf --output output/pdf
```

현재 1820×1220mm·5mm 판용 도면 레이아웃입니다. YAML 치수가 달라지면 조용히 잘못된 본을 만들지 않고 오류를 내므로 레이아웃도 갱신해야 합니다. `hardboard_drawing_manifest.json`에 원본 YAML 해시와 재단 목록을 보존하며 `output/pdf/parts/`에는 분할 PDF가 있습니다. PDF 전 페이지 렌더 검토·한글 추출·A4 페이지 크기·실제 1:1 부품/원 치수를 검사했습니다.
