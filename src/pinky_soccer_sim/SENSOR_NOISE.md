# 실측 기반 센서 오차

Gazebo와 MuJoCo의 기본 실행에 `config/measured_sensor_noise.yaml`을 사용합니다.
네이티브 관측은 `/sim/lidar/scan`, `/sim/imu`, `/sim/ultrasonic_link/scan`으로 받고,
공통 `sensor_adapter`가 한 번만 오차를 더해 기존 `/scan`, `/imu_raw`, `/us_sensor/range`를 발행합니다.
Gazebo 런타임 모델에서 기존 LiDAR Gaussian(표준편차 20mm)을 제거합니다. 공식 submodule은 수정하지 않습니다.
평가용 MuJoCo bridge도 같은 LiDAR remapping을 유지합니다. `/sim/*`는 판단 입력이 아닙니다.

```bash
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py imu_enabled:=true adc_enabled:=true
ros2 launch pinky_soccer_sim gazebo_soccer.launch.py imu_enabled:=true adc_enabled:=true
# 비교용 이상적 관측 (네이티브 Gazebo 20mm 잡음도 꺼진 상태)
ros2 launch pinky_soccer_sim mujoco_soccer.launch.py sensor_noise_enabled:=false
# 모델과 재현 가능한 seed 선택
ros2 launch pinky_soccer_sim gazebo_soccer.launch.py sensor_noise_config:=/absolute/noise.yaml sensor_noise_seed:=7
```

기본 seed는 `20261124`입니다. 센서별 독립 난수 스트림이므로 IMU/카메라 활성화 여부가 LiDAR 난수열에
영향을 주지 않습니다. 센서 timestamp가 뒤로 이동하면 해당 스트림을 초기화합니다.

## 측정 근거와 구현

원본은 실물 프로젝트 `ros_20261123_195718_b5b380`입니다. YAML에 프로젝트 SHA256,
실험 ID/시각/표본 수, 원본 LiDAR Bag 이름과 사용 시간 구간을 남깁니다. 입력 원본은 수정하지 않습니다.

| 센서 | 실측 자료 | 적용 |
|---|---|---|
| LiDAR | 범퍼~벽 150/300mm, 100/101회 + 해당 구간 원본 스캔 | 센서 기준 참거리 208.496/358.496mm에서 편차 −6.860/−6.354mm. 광선별 독립 σ 0.786/0.631mm + 스캔 공통 σ 0.373/0.247mm |
| 초음파 | 범퍼~벽 150/300/610mm, 각 200회 | 센서 기준 참거리 164.796/314.796/624.796mm에서 편차 −18.284/−1.299/+45.089mm, σ 0.453/2.224/1.442mm |
| 자이로 | 정지 3회, 합계 2,776표본 | 평균 편차 XYZ = −0.004231/+0.003708/+0.001209rad/s, 축별 σ = 0.010234/0.007365/0.003915rad/s, 축간 공분산 포함 |
| 가속도 | 동일 정지 원본의 quaternion과 가속도 | 기록 자세로 중력 9.81m/s²를 센서 좌표로 변환해 뺀 잔차를 추가(튀는 자세 표본은 정상 평균 자세 사용). 편차 XYZ ≈ +0.0960/+0.1015/−0.0053m/s², σ ≈ 0.0825/0.0815/0.1018m/s² |
| 자세 | 정지 각 실험의 평균 자세를 제거한 회전 벡터 | 일반 변동 σ XYZ 약 0.100/0.108/0.118°·상관시간 약 1.48초 + 큰 오차 10/2,776회(약 0.36%) 경험 분포. quaternion 곱으로 적용·단위 길이 유지 |

거리 표의 첫 열은 **센서 원점 기준 참거리(m)**입니다. 범퍼에서 잰 거리를 직접 입력하지 않습니다.
`table` 행은 `[reference_m, bias_m, independent_sigma_m, common_sigma_m]`입니다.
표 사이에서는 편차와 σ를 선형 보간합니다. 측정 범위 밖은 끝점의 **오차 파라미터**를 유지하며,
거리 자체를 끝점으로 제한하지 않습니다. 이는 검증된 원거리 특성이 아니라 명시적인 연장 가정입니다.
전방 평면 벽의 특성을 LiDAR 모든 방향/물체에 적용하는 것도 근사입니다.
원본 NaN/±Inf·범위 밖 관측은 보존하고, 유효한 측정값에 오차를 더해 범위를 벗어나면 ±Inf로 표시합니다.

LiDAR σ는 섹터 중앙값의 σ를 광선마다 그대로 복사하지 않습니다. 원본 Bag의 시간 구간에서 전방 광선의
각 평균을 제거한 시간 공분산을 계산해 공통/독립 성분으로 분리합니다. 원본 투영거리와 프로젝트 요약의
일치 및 정지성도 확인합니다. 가속도 원시 평균에 포함된 기울어진 중력은 추가 바이어스로 복제하지 않습니다.
IMU covariance는 추가된 정상상태 잔차의 공분산이며 평균 바이어스의 불확실성은 포함하지 않습니다.
`imu_native`는 같은 잡음 표본을 deg/s로 변환합니다. 자기장은 이상적 물리 자세로 계산하는 기존 가정을 유지합니다.

## 식별되지 않은 부분

이 모델은 Gaussian 평균·분산·축간/스캔 공분산과 IMU 첫 시차 상관을 맞춥니다. IMU는 측정한 수신 간격과
실험별 연속 표본으로 지수 상관시간을 추정하고, 시뮬레이션 시간 간격에 따라 잔차를 연속 갱신합니다.
자세의 큰 오차는 실험별 강건 평균에서의 회전 잔차 중 `max(0.02rad, 중앙값+10×MAD)` 밖의 표본으로
분리해 실측 빈도로 재생합니다. 이 표본들은 단순한 상시 Gaussian 떨림으로 섞지 않습니다.
튐의 하드웨어/드라이버 원인은 미확인이며, 절대 자세의 정답이 아닌 정지 평균 대비 잔차를 복제합니다.
발행 자세 covariance에는 일반 변동과 이 경험 분포의 혼합 분산을 포함합니다.
복잡한 시간 상관·장기 drift·이동 중 오차,
온도·재질·입사각 의존성과 결측률까지 동일하다고 볼 수는 없습니다.

- 카메라 재투영 오차 0.476px는 밝기 잡음의 표준편차가 아닙니다. 영상 밝기·픽셀 jitter를 임의 생성하지 않습니다.
- 공 X +13mm는 사용자가 지정한 **검토용 거리 보정**입니다. 이를 영상 센서 잡음으로 간주하거나 시뮬 픽셀에 적용하지 않습니다.
- IR ADC/배터리/encoder·odom/자기장 오차는 참값 대비 반복 자료가 없어 추가하지 않습니다.
- 시각 동기화가 확인되지 않아 측정 header age를 전송 지연으로 주입하지 않습니다.
- 카메라 높이·각도·바퀴 유효 기하는 별도의 `calibration_profile`로 적용합니다. 노이즈 모델이 장착/운동 모델을 변경하지 않습니다.

## 원본에서 다시 생성

ROS 환경을 source하고, 해당 거리 실험 전체를 포함하는 원본 Bag SQLite 파일을 지정합니다.
이 도구는 원본 프로젝트를 읽기만 하며 수정하거나 잠금을 해제하지 않습니다.

```bash
ros2 run pinky_soccer_sim fit_sensor_noise /absolute/project.yaml \
  --lidar-bag /absolute/raw_bag/raw_bag_0.db3 --output /absolute/noise.yaml
```

측정 후 저장된 프로젝트와 Bag에서 생성하며, 새 측정을 추가하면 명시적으로 다시 생성합니다.
프로젝트 YAML과 결과 YAML은 별도로 보존합니다.
