# 카메라 한 프레임 요청

실물·Gazebo·MuJoCo가 동일한 서비스 `/camera/capture_frame`
(`pinky_soccer_msgs/srv/CaptureFrame`)를 제공합니다. `camera_enabled:=true`일 때만 생성됩니다.
기존 원본/JPEG 영상 토픽과 함께 사용할 수 있으며 원본 영상 구독 없이 호출할 수 있습니다.

## 실행과 저장

PC에서 빌드하고 새 터미널에 적용합니다:

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pinky_soccer_msgs pinky_soccer_bringup pinky_soccer_sim
source install/setup.bash
```

실물 로봇에는 `pinky_soccer_msgs` 소스와 갱신된 `pinky_soccer_bringup` 소스를
별도 워크스페이스에 전달하고 로봇 ARM64 환경에서 아래 두 패키지를 빌드해야 합니다.
PC의 build/install을 복사하지 않습니다. 기존 description/공식 드라이버 설치는 유지합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/local_setup.bash
cd ~/pinky_soccer_ws
colcon build --symlink-install --packages-select pinky_soccer_msgs pinky_soccer_bringup
source install/local_setup.bash
```

갱신된 코드로 카메라 launch를 다시 실행해야 서비스가 생성됩니다.
실물은 평소 `robot.launch.py`, 시뮬레이션은 평소 Gazebo/MuJoCo launch를 사용합니다.
이미 실행 중인 카메라와 중복으로 실행하지 않습니다. 통신 환경(domain/DDS)은 서버와
클라이언트가 일치해야 합니다. 실물 domain37, 시뮬레이션 domain118은 프로젝트의 예시입니다.

원본 한 장을 무손실 PNG로 저장:

```bash
ros2 run pinky_soccer_bringup capture_image --format raw --output /tmp/pinky_frame.png
```

JPEG 한 장은 받은 압축 바이트 그대로 저장:

```bash
ros2 run pinky_soccer_bringup capture_image --format jpeg --output /tmp/pinky_frame.jpg
```

출력 사진 옆에 `pinky_frame.png.camera_info.yaml` 또는
`pinky_frame.jpg.camera_info.yaml`을 함께 저장합니다. 촬영 시각·프레임·해상도·당시 K/D/R/P를
보존하며 기존 파일은 덮어쓰지 않습니다. 이 YAML은 촬영 당시의 CameraInfo이며 새로 계산한
보정 결과가 아닙니다. 미보정 카메라의 K=0 등은 그대로 보존합니다.

서비스만 확인하려면:

```bash
ros2 service type /camera/capture_frame
ros2 interface show pinky_soccer_msgs/srv/CaptureFrame
```

직접 호출도 가능하지만 응답의 이미지 배열이 터미널에 길게 출력되므로 저장 CLI가 편합니다:

```bash
ros2 service call /camera/capture_frame pinky_soccer_msgs/srv/CaptureFrame \
  '{format: jpeg, timeout_sec: 3.0}'
```

## 요청·응답 규칙

| 필드 | 의미 |
|---|---|
| 요청 `format` | `raw`(기본) 또는 `jpeg` |
| 요청 `timeout_sec` | 새 프레임 대기 제한, 0초 초과~10초 이하·기본3초, 실제 경과 시간 기준 |
| 응답 `success`, `message` | 성공 여부 또는 오류/시간 초과 사유 |
| 응답 `image` | raw 요청 시에만 채워지는 bgr8 `sensor_msgs/Image` |
| 응답 `compressed_image` | jpeg 요청 시에만 채워지는 `sensor_msgs/CompressedImage` |
| 응답 `camera_info` | 같은 획득 프레임의 시각·광학 좌표계·해상도·내부 파라미터 |

요청을 서버에서 처리한 ROS 시각보다 타임스탬프가 **엄격히 새로운** 프레임만 반환합니다.
캐시된 이전 사진을 대신 반환하지 않습니다. 실물 타임스탬프는 기존 카메라 노드와 같이
SDK 프레임 읽기 완료 시각이며 센서 노출 시작 시각을 보장하지 않습니다.
시뮬레이션은 렌더러의 원래 시각과 같은 프레임의 CameraInfo를 보존합니다.
일시정지·영상 중단·시각 역행으로 새 프레임이 오지 않으면 실제 경과 시간 기준으로 실패합니다.
JPEG 품질은 실물 `camera_jpeg_quality`, 시뮬레이션 sensor_config의 기존 설정을 사용합니다.

최대4개 대기 요청을 허용하며 초과 요청·잘못된 형식/시간·변환 실패는 이미지 없는 오류 응답입니다.
동시에 기다리던 요청들은 같은 새 프레임을 받을 수 있습니다. 요청마다 독립된 촬영을 예약하는
하드웨어 트리거/버스트 기능은 아닙니다. 서비스의 시간 제한은 서버 새 프레임 대기 기준이며,
네트워크 전송 시간까지 보장하지 않습니다. 저장 CLI는 서비스 발견5초와 응답 추가 대기를 제한합니다.

## 전송 부하와 보정 UI

서비스 호출만으로 원본 영상 토픽 구독이 생기지 않으며 선택한 한 장만 응답으로 전송합니다.
다른 프로그램이 원본 토픽을 구독하면 그 프로그램을 위한 원본 스트림은 계속 발행됩니다.
보정 UI는 JPEG를 상시 구독하고, 학습·독립 검증·바닥 보드 촬영 버튼에서만 raw 서비스를
호출합니다. UI의 상시 원본 구독은 제거했으며 원본 촬영 PNG·CameraInfo·Bag·CSV를 보존합니다.
구체적인 저장 구조와 서버 갱신은 [ROS_UI.md](../pinky_soccer_calibration/ROS_UI.md)를 참고하세요.

카메라 장치 자체를 추가로 열지 않으므로 스트림·서비스가 카메라 점유를 두고 충돌하지 않습니다.
실물 노드는 별도 실행 스레드에서 캡처를 수행하고 서비스/제한 시간 처리를 병행합니다.
