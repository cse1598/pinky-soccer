# 실물 기본 GoToPose MPC 시험 — 2026-11-27

사용자 지시에 따라 준비 이동의 `start_avoid_walls`, `start_avoid_opponents` 기본값을
false로 복귀했다. 일반 GoToPose의 회피 옵션도 기본 false다. 회피 solver는 명시적으로
옵션을 켠 요청에서만 사용하며 독립 충돌 정지·watchdog는 유지한다.

실물 MPC의 외부 속도 상한 기본값은 `max_lin=0`, `max_ang=0`으로 바꿨다.
이는 기존 MPC의 바퀴·차체 입력·입력 변화율 제약만 사용한다는 뜻이다.
시뮬레이션의 기존 기본값은 유지했다. 사용자가 지정한 실행 설정은
`controller=mpc`, `mpc_backend=acados`, `mpc_damping_scale=2.0`, `mpc_observer_mode=align`이다.
마지막 두 옵션은 아래 명령에 명시하며 공용 launch 기본값 자체를 변경하지 않았다.

## 직접 확인한 원인과 수정

1. 이전 회피 MPC 준비 실패: 초기화는 성공했지만 worker 실행109~201ms로
   100ms slot 6개를 놓쳤고, 결과도 모두 `ok=false`였다. 같은 시작 상태·보정·속도
   조건의 오프라인 비교는 기본 solver 3/3 성공, 회피 solver 0/3 성공이었다.
   오프라인 비교가 실제 센서/스케줄 전체를 재현한 것은 아니다.
2. 기본 MPC 첫 실물 시험: 약30cm 이동 후 실제 앞쪽 상대 Pinky에 대해 독립 안전
   감시가 정지했다. 주행 후 카메라와 상대 추적 기록으로 장애물을 확인했다.
3. 장애물 제거 후: 목표 근처 정상 보정 중 새 반전 감지가 약2초 만에 실패를
   선언했다. 제동 확인 중 이전 반전 이력은 초기화하고, 반전 여부는 진단으로
   유지하되 실패 판정은 원래의4초 무진전에 맡겼다. 성공 오차·정지 조건은 그대로다.
4. 초기 준비 GUI는 `rclpy.Node.clients` 읽기 전용 속성과 이름이 충돌해 종료됐다.
   `action_clients`로 수정하고 native ROS 생성 검사를 통과했다.
5. 최신 GUI 문제 조사에서는 실제 `debug_enabled=true`와 지도 영상3초70장을
   확인했다. 이전 bag의 false를 최신 실행의 옵션 누락으로 해석한 설명은 정정했다.
   debug 옵션은 실행 중에도 반영하며 OFF일 때 창에 안내한다. 골대 영상처리는
   초기화 이후 중단되므로 연속 실시간 골대 마스크 발행을 기대하지 않는다.

## 요청 설정의 최종 실물 결과

임시 시험 목표는 `(-0.3, 0, 0)`이고 시작 추정 Pose는 약`(-0.601, -0.009, 0.008)`이다.
시험 후 임시 목표는 제거하고 일반 launch의 기존 시작 목표`(-0.4, 0, 0)`로 복원했다.

| 항목 | 결과 |
|---|---|
| 준비 이동 | READY 도달 후 명시적 STOP, 제어 구간 약3.49초 |
| 마지막 위치/각도 오차 | 약2.98mm / 2.74° — Localization 기준, 독립 실측 GT 아님 |
| 외부 속도 상한 | 선속도0 / 각속도0 |
| MPC 내부 차체 입력 한계 | 약0.268625m/s / 5.47584rad/s |
| 최대 선속도 | 명령0.268625m/s, 바퀴 피드백 환산 약0.27174m/s |
| 활성화 slot | 24개 중 누락0, 완료 후 취소된 job은 별도 기록 |
| worker 실행 시간 | 평균31.86ms / P95 73.84ms / 최대79.47ms |
| worker 대기 / 결과 pickup P95 | 약24.79ms / 17.57ms |
| 주행 중 PC cmd / drive | 약48.72Hz / 20.00Hz, 수신 기준 |
| 회귀 검사 | 최종51 passed, 기존 acados 경고27건 |

raw joints/odom은 약30Hz, joint_state_publisher 최종 joints는 약20Hz다.
최종 joints의 timestamp는 재발행 시각이므로 물리 측정 시각과 혼동하지 않는다.
`align`에서 사용한 timestamp 유효성은 확인했지만 이 시각이 모터 bulk-read 중간 시각이라는 뜻은 아니다.

이번 시험에는 도착부 보정·짧은 zero 구간이 남아 있다. 모든 끊김이나 전후진 보정이
해결됐다고 보고하지 않는다. damping/align/반전 판정 수정이 함께 적용된 단일 성공
시험이므로 각각의 기여도를 분리한 성능 비교나 장거리·모든 초기 자세 검증은 아니다.

원시 bag과 CSV/동일 시간축 그래프는 로컬 미추적
`docs/validation/real_prepare_20261127/base_align2/`에 보존했다.
`live_metrics.json`은 실제 활성화 사건별 집계이고, 일반 분석기의 전체 bag 평균과 구분한다.

## PC 재실행

현재 실행 중인 PC launch를 먼저 종료한다. 로봇 bringup이 이미 있으면 중복 실행하지 않는다.

```bash
cd ~/pinky-soccer
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CAL="$PWD/ros_20261123_195718_b5b380/exports/6fa01d7d19d64855b878e2613e1d5824"
ros2 launch pinky_soccer_viz game.launch.py \
  mode:=real field:=hardboard team:=Y \
  controller:=mpc mpc_backend:=acados \
  mpc_damping_scale:=2.0 mpc_observer_mode:=align \
  verify:=true preparation:=true localization:=rectangle \
  calibration_profile:="$CAL/profile.yaml" camera_yaml:="$CAL/camera.yaml" \
  ball_offset_source:="$CAL/profile.yaml" \
  identification:="$PWD/src/pinky_soccer_mpc/config/real_motor_response.yaml" \
  wheel_feedback:=true localization_debug:=true max_lin:=0 max_ang:=0
```

일반 실행은 IDLE에서 정지 대기한다. 준비→READY→별도 시작 순서를 유지한다.
실물 하드웨어에는 새 소스를 배포하지 않았다. 기존 설치 bringup을 동일한 보정으로
기동했으며 원격 로그는 핑키의`/tmp/pinky-codex-bringup-20261127.log`에 있다.
모터 보정 원본·MPC 비용/제약 소스·물리 형상·공식 submodule·Gazebo는 수정하지 않았다.
코드 마무리 변경은 미커밋이다.
