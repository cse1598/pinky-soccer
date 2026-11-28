# IMU 없는 준비 절차 MuJoCo 시험 — 2026-11-28

사용자 요청으로 시뮬레이션 회차 2개만 수행하고 종료했다. Domain193,
headless, 두 로봇, localization_heading_source=odom, Yellow strategy=path_mpc,
Blue strategy=legacy. 경기 START는 한 번도 보내지 않았다. Path MPC는 준비 이동에
사용되지 않으므로 이 결과는 MPC 공격 성능 검증이 아니다.

| 회차 | 시작 조건 | 결과 |
|---|---|---|
| 1 | 양팀 기본 시작 위치/방향 | 심판 보정 완료 전 PREPARE로 STOPPED. 같은 회차에서 심판 준비 후 재요청하여 양쪽 READY |
| 2 | Yellow (-0.25,0.18,0.7rad), Blue 기본값 | 재요청 없이 양쪽 READY; 드라이버 계측 7.15sim초 |

첫 회차 전체 계측 53.4sim초에는 시험 측 재요청 대기가 포함된다. 두 번째 PREPARE
상태는 t=52.35, 양팀 READY는 t=53.45(약1.1초)이며, 이미 시작 위치에 놓여 있었다.
이를 최초 실행부터 1.1초 만에 준비된 결과로 해석하면 안 된다. 두 번째 회차는
CALIBRATING t=1.95 → 양팀 READY t=8.95(상태 토픽 기준7.0초)다.

공통 확인:
- 양팀 localization source=rectangle_lidar_goal_odom, heading_source=odom.
- IMU yaw는 None이며 초기화/추적에서 IMU 관측을 사용하지 않는다.
  시뮬레이터의 IMU 발행 자체를 끈 시험은 아니다(odom 모드는 이를 구독하지 않음).
- LiDAR 외벽 정합 + HLS 골대 색상으로 초기 대칭을 해소했다.
- 양팀 시작 위치 도착 및 odom 정지 조건을 만족하여 READY 진입.
- 두 번째 회차 Yellow 참 위치 기준 도착 오차 약9.9mm, 나머지 약0.07mm.
  Ground Truth는 평가만 사용. 이 수치는 제한된 시작 조건의 단일 회차 값이다.
- 종료 시 양팀 STOPPED 및 바퀴 속도 정지 확인. 모든 launch 자식 프로세스 종료 확인.

자료는 trial_1/, trial_2/의 launch.log, driver.log, topics.jsonl, result.json.
실물 연결/SSH/주행 없음. 실제 준비 시간 개선과 실물 오차는 미검증.
별도 합성 센서/기존 회귀 74PASS, 저장 영상 없는 4SKIP, PC 두 패키지 빌드PASS.
