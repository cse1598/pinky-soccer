# pinky_soccer_dynamics

Pinky Pro의 바퀴 단위 동역학 모델, 보정 결과를 반영하는 파라미터 로더, `/cmd_vel` 명령 변환기,
응답 식별·모델 검증 도구입니다. NMPC(`pinky_soccer_mpc`)가 이 모델을 그대로 씁니다.
실물 보정 절차는 [CALIBRATION.md](CALIBRATION.md)에 있습니다.

## 모델

상태 `X = [x, y, θ, Ω_L, Ω_R]`, 입력 `U = [Ω_L,cmd, Ω_R,cmd]`
(바퀴 속도는 rad/s, 각 바퀴가 로봇을 앞으로 미는 방향이 +)

```
v  = (r_R Ω_R + r_L Ω_L) / 2          ω  = (r_R Ω_R − r_L Ω_L) / b
ẋ  = v cos θ      ẏ = v sin θ         θ̇  = ω
Ω̇_L = (Ω_L,cmd − Ω_L) / τ_L           Ω̇_R = (Ω_R,cmd − Ω_R) / τ_R
```

선택 확장(파라미터로 켜고 끔): 바퀴 가속도 한계 `|Ω̇| ≤ wheel_accel_max`(XL330 속도 프로파일),
명령 지연 `delay`(입력 버퍼). NumPy 구현(`model.f`, `rk4`, `Simulator`)과 CasADi 구현
(`model.casadi_model`, acados용)이 같은 식이며 테스트로 일치를 확인합니다.

## 파라미터와 출처

`config/dynamics.yaml`에 대상(`real`, `mujoco`, `gazebo`)별 기본값이 있고, 모든 값에
`source`가 붙습니다: `official_default`, `simulation_model`, `assumption`, `measured`,
`simulation_identified`. 실측이 아닌 값은 절대 `measured`로 표시되지 않습니다.

병합 순서 (`params.load(target, profile=..., identification=...)`)

1. `dynamics.yaml` 기본값
2. 보정 UI의 `profile.yaml`(pinky_soccer_calibration, schema 1, `human_verified`)
   - `results.wheels.left_radius_m / right_radius_m / separation_m` → 모델 기하 (`measured`)
   - `launch.wheel_radius / wheel_separation` → **드라이버** 파라미터(공식 드라이버가 이 값으로 시작됨)
   - `environment: rehearsal` 프로필은 실물 대상에 거부
3. 응답 식별 파일(이 패키지 형식 `identification_schema: 1`, `accepted: true`일 때만)
   - `tau_left`, `tau_right`, `delay`, `wheel_speed_max`, `wheel_accel_max`

profile.yaml에는 응답 파라미터 키가 없으므로 새 키를 만들지 않고 별도 식별 파일에 둡니다.
파라미터가 바뀌면 NMPC 솔버는 파라미터 해시가 달라져 다시 생성됩니다.

## 명령 변환 (보정값 이중 적용 방지)

MPC는 바퀴 목표 속도를 내고, 로봇은 `/cmd_vel`만 받습니다. 드라이버는 **자기 반경 r_d·간격 b_d**로
`/cmd_vel`을 바퀴 속도로 바꾸므로, 변환기는 그 역변환 `v = r_d(Ω_R+Ω_L)/2, ω = r_d(Ω_R−Ω_L)/b_d`만
씁니다. 실측 유효 반경·간격은 동역학 모델에만 들어갑니다. `driver_applied`는 실물 드라이버의
100 rpm 비례 축소·0.229 rpm 버림, MuJoCo 브리지의 0.25 m/s·1.5 rad/s 개별 제한을 재현합니다.

## 바퀴 부호 (확인 필요)

URDF 두 바퀴 관절 축은 모두 +y라 시뮬레이터 `/joint_states`는 전진 시 두 바퀴 모두 +입니다.
실물 공식 드라이버는 오른쪽 모터가 반대로 달려 있어 명령(`rpm_r = −…`)과 odom(`delta_r = −…`)에서만
부호를 바꾸고, `/joint_states`에는 원시값을 냅니다. 우리 `wheel_joint_state_relay`는 이름만 바꿉니다.
따라서 실물 오른쪽 바퀴 부호는 −1로 추정(`joint_sign.right`, `assumption`)하며
`check_wheel_signs`로 확인해야 합니다.

## 실행

```bash
# casadi/acados를 쓰는 패키지는 .venv 파이썬으로 빌드 (실행 파일이 .venv를 사용)
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --symlink-install --packages-select pinky_soccer_dynamics pinky_soccer_mpc
.venv/bin/python -m colcon test --packages-select pinky_soccer_dynamics pinky_soccer_mpc

ros2 run pinky_soccer_dynamics show_params --target real [--profile profile.yaml] [--identification id.yaml]
ros2 run pinky_soccer_dynamics check_wheel_signs      # 사람이 로봇을 손으로 앞으로 민다. 명령 발행 없음
ros2 run pinky_soccer_dynamics identify_sim --out identification_mujoco.yaml   # MuJoCo 전용
```

## 설계 제약

- 실물 로봇에 명령을 보내는 도구는 이 패키지에 없습니다(1단계). 응답 실험 도구는 2단계에서 안전 조건과 함께 추가합니다.
- 공식 `pinky_pro` 서브모듈은 수정하지 않습니다.
- 시뮬레이터 정답값은 식별·검증 도구에서만 씁니다(판단 계층 입력 아님).
