# 실물 MPC 모터 응답 보정 적용 — 2026-11-26

사용자 요청으로 저장된 실물 모터 실험의 좌우 시정수를 MPC에 연결했다.
원본 프로젝트/CSV/메타데이터와 별도 P3 작업 파일은 수정하지 않았다.
기존 진단 도구 작업은 유지한다. 전진/정지 반복 증상의 실제 원인을 확정하거나 해결한 것으로 해석하지 않는다.

## 적용값과 범위

| 항목 | 기존 | 적용 |
|---|---:|---:|
| 좌우 τ (s) | 0.05 / 0.05 | 0.09031918634164283 / 0.08894523651011192 |
| 모터 ramp (rad/s²) | 74.9 | 74.9 유지 |
| 모델 delay (s) | 0 | 0 유지 |
| 바퀴 명령 속도/가속도 한계 | 기존 설정 | 유지 |
| MPC 가중치·100 ms slot·안전 정지 | 기존 설정 | 유지 |
| Gazebo/MuJoCo 기본 모델 | 기존 설정 | 유지 |

이 값은 **기존 ramp를 고정한 no-delay 모델의 유효 시정수**다.
모터만 분리한 물리 시정수·Wi-Fi/명령 전달 지연의 실측값이라고 주장하지 않는다.
가속도와 τ를 동시에 맞추면 46.958 rad/s²와 τ 67.056/64.928 ms가 나오지만,
반복별 spread 검사에서 좌 τ/가속도가 식별 불충분으로 분류됐다. 이 가속도는 적용하지 않았다.
대신 기존 모델 구조·ramp·한계를 보존한 채 τ 두 개만 적합했다.

## 데이터와 검증

- 프로젝트 `ros_20261123_195718_b5b380`, revision 428, `pinky-pro`, 요가매트.
- 완료 실험 48회: 전진12·후진12·좌회전9·우회전9·복합6.
- 모터 진단의 serial write 완료와 Present Velocity bulk read 중간 시각을 사용한 `robot_clock` 시간축.
  재timestamp된 최종 joint_states는 적합에 사용하지 않았다. 원시 입력으로부터 드라이버 기하를 재구성한 출처도 manifest에 남겼다.
- 모든 원래 purpose는 identification이었다. 종류별 프로젝트 순서에서 매 4번째 실험을 **오차를 보기 전에** 검증용으로 분리했다.
  학습37·보류검증11이며 원본 purpose를 바꾸지 않았다. 다른 날짜/새 실험에 대한 독립 검증은 아니다.
- 기존 P3 식별기의 연속시간 ramp+lag residual과 SciPy bounded least-squares를 사용했다.
  고정 ramp에서 τ만 [0.003, 0.6] s 범위로 적합했다. 로컬 공분산 판정은 두 값 모두 estimated다.
- 반복별 상대 표준편차: 좌11.24%·우9.51%. 실험 단위 bootstrap 10회(seed0)의 구간:
  좌 [0.090080, 0.092968] s, 우 [0.088406, 0.090288] s. 적은 bootstrap 횟수의 참고 구간이다.
- 작은/큰 입력별 τ: 좌0.095387/0.089188 s, 우0.094122/0.088064 s.
- 검증 실험별·바퀴별 속도 RMSE 평균은 **0.251703→0.136738 rad/s(약45.7% 감소)**,
  상승구간 RMSE 0.683412→0.350051, 감속구간 0.613958→0.325246 rad/s.
- **모든 지표가 좋아진 것은 아니다.** 0.1초 적분 각도 오차 평균0.004520→0.006040 rad,
  2초0.075922→0.108427 rad로 커졌다. 최종 각도 오차는 거의 동일하다.
  따라서 속도 응답 적합 개선과 실제 이동 성능 개선을 구분하며, 실물 GoToPose 재측정이 필요하다.

입력144개 파일의 SHA256·실험 분할·진단 출처는
`/tmp/pinky-real-motor-fit-20261126/data_manifest.json`, 적합/반복/검증 상세는
`/tmp/pinky-real-motor-fit-20261126/fixed_ramp_tau_fit.json`에 보존했다.
배포 스냅샷 `src/pinky_soccer_mpc/config/real_motor_response.yaml`에는 실험 ID·각 파일의 요약 해시·
분포/검증 요약·식별기 소스 해시를 기록했다. 원본은 프로젝트의 `motor_response/`에 그대로 있다.

## 코드 적용과 실행

`response_calibration.apply_response()`가 실물에만 스냅샷을 읽어 τ 두 개만 복사한다.
단위·실물 target/environment·모델 구조·기존 ramp와의 일치·유효 범위·식별상태·학습/검증 분리와
검증 오차를 검사한다. legacy 응답 파일이나 실물 파일의 시뮬 적용은 거부한다.
실물 `identification` 빈 값/`auto`는 이 로봇 스냅샷을 사용한다. 기존 game launch의 인자 전달을 그대로 쓴다.
시뮬 빈 값/`auto`/`none`은 원래 params 객체를 그대로 반환한다. 기존 모델/프로필 파일은 수정하지 않았다.

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m colcon build --symlink-install --packages-select pinky_soccer_mpc
source install/setup.bash
# 기존 실제 실행 명령과 다른 인자를 유지해 MPC 노드를 재시작한다.
# game.launch 기본 real 응답은 새 snapshot, 아래는 명시 경로 사용 예다.
ros2 launch pinky_soccer_viz game.launch.py mode:=real controller:=mpc \
  wheel_feedback:=true verify:=true \
  identification:=/home/cse/pinky-soccer/src/pinky_soccer_mpc/config/real_motor_response.yaml
# 기존 profile/namespace/localization 등 실제 실행 인자는 같은 값으로 추가한다.
# 원복: 동일 명령에서 identification:=none
```

위 명령은 사용자가 기존 구동 조건으로 재시작할 때 참고하는 예다. 이번 작업 중 실물 주행/노드 재시작은 하지 않는다.
다른 로봇에는 그 로봇의 응답 파일이나 `identification:=none`을 사용한다.
새 모델은 startup 때 캐시를 생성할 수 있으므로 시작 신호 전에 기동 완료를 확인한다.
로그 `Real motor response`와 status `motor_response`의 τ/해시로 실제 적용을 확인한다.
도착 여부/반복 정지 평가는 [기존 bag 수집 절차](GOTOPOSE_DIAGNOSTICS.md)로 전후 비교한다.

## 구현 검증

- 새 보정 로더·복원·거절 조건·시뮬 params 동일성·symbolic/scalar/numeric 일치와 기존
  schedule/observer/dynamics/acados 폐루프 회귀 **77 tests PASS**.
- 기존 bag 진단 도구도 새 응답 선택 인자를 반영한 뒤 **8 tests PASS**.
- `pinky_soccer_mpc` symlink build/스냅샷 설치 확인, 새 코드 F lint/diff 검사 PASS.
- 새 실물 파라미터로 native acados 솔버 생성 후 0.12 m 목표/0.08 m/s 상한의 최적화 해 채택 확인.
  ROS 노드 spin이나 실제 주행을 실행한 검사가 아니다.
- 원본 모터 자료144개 파일의 SHA256과 적합/manifest 해시 일치 확인.
- 실물 GoToPose 재주행과 증상 bag 비교는 아직 수행하지 않았다.
