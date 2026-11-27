# 시뮬레이션 색상 검증 (2026-11-27)

사용자가 제공한 Pinky Pro 카메라 평균 RGB를 양 엔진의 **초기 재질 색상**으로 적용했다.
픽셀 색상까지 실물과 일치한 상태는 아니다. 조명·카메라·제어 설정은 변경하지 않았다.

## 적용값과 수정 파일

| 대상 | 기존 재질 RGBA (0–1) | 변경 RGB (0–255) | 변경 재질 RGBA (0–1, 반올림) |
|---|---|---|---|
| 공 | 1, 0.30, 0, 1 | 230, 57, 4 | 0.9019607843, 0.2235294118, 0.01568627451, 1 |
| 노랑 골대 | 1, 0.85, 0.04, 1 | 237, 229, 4 | 0.9294117647, 0.8980392157, 0.01568627451, 1 |
| 파랑 골대 | 0.08, 0.3, 0.95, 1 | 5, 75, 134 | 0.01960784314, 0.2941176471, 0.5254901961, 1 |

- [경기장 생성기](../src/pinky_soccer_sim/pinky_soccer_sim/rcj_field.py): 양 골대 상수, RGB/255.
- 생성 결과: [RCJ SDF](../src/pinky_soccer_sim/worlds/rcj_soccer.world),
  [RCJ MJCF](../src/pinky_soccer_sim/worlds/rcj_soccer.mjcf),
  [하드보드 SDF](../src/pinky_soccer_sim/worlds/hardboard_soccer.world),
  [하드보드 MJCF](../src/pinky_soccer_sim/worlds/hardboard_soccer.mjcf).
  양 골대의 뒤판·옆판·지붕·바닥 전체에 적용.
- [Gazebo 런타임 공](../src/pinky_soccer_sim/launch/gazebo_soccer.launch.py): 기존 SDF ambient/diffuse.
- [MuJoCo 런타임 공](../src/pinky_soccer_sim/pinky_soccer_sim/mujoco_soccer_bridge.py): 기존 MJCF material rgba.
- [기존 공 회귀 검사](../src/pinky_soccer_sim/test/test_mujoco_ball.py): 기대 색상 갱신.
- [패키지 README](../src/pinky_soccer_sim/README.md), 이 보고서와 아래 검증 자료.

MuJoCo·Gazebo 모두 적용 완료. 생성 월드에는 공이 없고 launch 때 공을 추가하므로
월드 네 파일과 런타임 두 경로를 함께 변경했다. 별도 모델·패키지는 추가하지 않았다.

## 실제 렌더 측정

하드보드 경기장, 로봇 시작 XY=(0,0), yaw=0(파랑)/π(노랑), 공 XY=(0.3,0),
기존 범퍼·카메라 기울기 8°, 640×480, 수평 FOV 1.1519 rad를 사용했다.
노랑 확인은 기존 launch의 yaw 인자에 해당하는 시험 자세만 사용하며 저장된 모델 위치는 바꾸지 않았다.
MuJoCo 3.6.0/NVIDIA OpenGL에서 공식 builder+경기장 wrapper로 생성한 모델의
`front_camera`를 `mj_forward` 후 정지 렌더했다(simple_render=false).
Gazebo는 실제 `gazebo_soccer.launch.py gui:=false`의 `/camera/image_raw`를
독립 localhost DDS domain188·GZ partition에서 12프레임 수신한 뒤 마지막 프레임을 저장했다.
기존 전역 조명과 재질 광택은 유지했다. MuJoCo 결과는 ROS 토픽 수신 검증이 아니다.

양 엔진 모두 같은 **고정 내부 ROI**를 썼다: 골대 뒤판 x=[215,425), y=[175,200),
공 중심(320,249)·반경30px 원. 테두리·배경을 피하고 ROI 내부 음영·하이라이트는 포함한다.
목표 색으로 픽셀을 선별하지 않았다. 생산 Perception과 분리된 오프라인 측정이다.
OpenCV 4.6 `COLOR_RGB2HLS`의 8비트 값(H=0–180, L/S=0–255)을 픽셀마다 계산한 후 평균했다.
따라서 평균 RGB를 HLS로 한 번 변환한 값과 다를 수 있다.

| 엔진 | 대상 | 출력 평균 RGB | 출력 평균 HLS | ΔRGB (출력−실측) | ΔHLS (출력−제공값) |
|---|---|---|---|---|---|
| MuJoCo | 공 | 154.89, 41.75, 7.19 | 7.00, 81.17, 236.67 | −75.11, −15.25, +3.19 | 0.00, −35.83, −9.33 |
| MuJoCo | 노랑 | 161.65, 157.68, 46.07 | 29.00, 103.86, 142.18 | −75.35, −71.32, +42.07 | 0.00, −17.14, −104.82 |
| MuJoCo | 파랑 | 46.56, 81.29, 110.56 | 104.00, 78.56, 104.29 | +41.56, +6.29, −23.44 | 0.00, +8.56, −132.71 |
| Gazebo | 공 | 111.67, 56.40, 9.66 | 13.95, 60.39, 217.53 | −118.33, −0.60, +5.66 | +6.95, −56.61, −28.47 |
| Gazebo | 노랑 | 147.89, 145.24, 15.00 | 29.35, 81.89, 208.00 | −89.11, −83.76, +11.00 | +0.35, −39.11, −39.00 |
| Gazebo | 파랑 | 8.00, 54.00, 73.00 | 99.00, 40.00, 205.00 | +3.00, −21.00, −61.00 | −5.00, −30.00, −32.00 |

비교 기준 HLS는 사용자 제공 공(7,117,246), 노랑(29,121,247), 파랑(104,70,237)이다.
명도·채도 차이가 크며 엔진 간에도 차이가 남는다. 이 한 자세의 평균만으로 여러 거리·면의
색상 일치가 성립하지 않으므로 이번에는 재질값 추가 보정 없이 지정 초기값을 유지했다.
재질 이외 조명·카메라·영상 후처리를 바꾸지 않았다. 최종 색상 일치와 HLS 인식 성능은 미검증이다.
RCJ 월드는 생성 일치·물리 회귀를 검증했으며 RCJ 카메라 색상은 별도 미측정이다.
실물 원본 ROI/조명/노출 조건을 다시 측정한 결과가 아니라 제공된 평균과의 비교다.

원본 PNG: [MuJoCo 파랑/공](validation/sim_colors_20261127/mujoco_blue.png),
[MuJoCo 노랑](validation/sim_colors_20261127/mujoco_yellow.png),
[Gazebo 파랑/공](validation/sim_colors_20261127/gazebo_blue.png),
[Gazebo 노랑](validation/sim_colors_20261127/gazebo_yellow.png).
[수치 원본/ROI](validation/sim_colors_20261127/measurements.json),
[저장 PNG 재측정 코드](validation/sim_colors_20261127/measure.py).

## 변경 범위 검증

- 변경 전 사본과 네 월드 XML을 재귀 비교: 골대 색상 값 60곳만 변경(엔진별 10개 면 × 경기장2종,
  SDF는 ambient/diffuse 각1개). 나머지 태그·속성·값 동일.
- 재생성된 모든 메시 STL·manifest 바이트 동일.
- 런타임 공 SDF/MJCF도 색상·XML 공백을 제외하면 동일. 공 specular/shininess 유지.
- `test_rcj_field.py`, `test_mujoco_ball.py`, `test_materials.py`: **26 passed**.
- `pinky_soccer_sim` symlink 빌드·Python 컴파일·`git diff --check` 통과.
- 형상·치수·위치·질량·관성·마찰·반발·충돌·경기장·카메라·조명·MPC·BT·Skill·
  World Model·ROS 인터페이스·Perception 변경 없음. 공식 로봇 코드도 수정 없음.
  현재 브랜치에 이미 있던 다른 작업의 미커밋 변경은 보존했다.
- HLS 필터·원형 피팅·골대 형상 인식·초음파 융합·드리블은 구현하지 않았다.
  이번 색상에서 기존 검출/경기 전체 동작의 정확도는 미검증이다.

## 실행 및 확인

PC에서만 빌드한다. 이번 변경으로 핑키 내부 코드를 교체할 필요는 없다.
이미 켜진 시뮬레이션은 종료 후 재실행해야 재질이 반영된다.

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_sim
source install/setup.bash

# 하나씩 실행. field:=rcj로 기존 RCJ도 확인 가능.
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py field:=hardboard
ros2 launch pinky_soccer_sim gazebo_soccer.launch.py field:=hardboard
```

노랑 골대는 위 명령에 `yaw:=3.141592653589793`을 추가해 확인한다.
두 실행을 동시에 띄울 경우 기존 프로젝트의 DDS 분리 규칙을 따른다.
다른 터미널에서 동일 환경을 source한 뒤 원본 카메라 PNG를 저장할 수 있다:

```bash
ros2 run pinky_soccer_bringup capture_image --format raw --output /tmp/sim_color.png
# 이 보고서의 저장 PNG 통계 재계산 (numpy/OpenCV 필요)
.venv/bin/python docs/validation/sim_colors_20261127/measure.py
```

픽셀 비교는 RViz/화면 캡처나 JPEG 대신 원본 PNG로 한다. 다른 GPU·렌더러·자세·카메라
보정 프로필에서는 출력과 ROI가 달라질 수 있다. 저장 ROI를 다른 자세에 그대로 쓰지 않는다.
