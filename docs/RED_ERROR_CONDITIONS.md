# 경기 로봇 빨간 점멸 조건 — 2026-11-28

현재 통합 경기 launch에서는 match_preparation이 LED를 소유한다.
preparation_core.Preparation.state == ERROR일 때만 빨간색(96,0,0)이 점멸한다.
아래는 기존 오류 조건을 유지한 번호표다. 사용자 지정에 따라9번은 ERROR 대신
RECOVERING 대기로 변경했다. 다른 번호의 오류 정책은 유지한다.

| 번호 | 상태/이유 | 실제 조건 |
|---|---|---|
| 1 | Localization: GOAL_COLOR_FAILED | 초기 골대 두 방향에서 모두 색 판별 실패 |
| 2 | Localization: CALIBRATION_TIMEOUT | 연결 조건 충족 후 초기화 제한60초 초과 또는 누적 회전2.5π 이상 |
| 3 | Localization: ABORTED | 초기화 중 abort 요청 또는 IMU 모드의 방위 기준 불연속 등으로 중단; odom 모드는 IMU를 사용하지 않음 |
| 4 | Localization: REINITIALIZATION_REQUIRED | 신뢰 가능한 외벽 보정이3초 넘게 없거나 재초기화 플래그 설정 |
| 5 | Localization: ROTATION_LIMIT | 준비 관리자가 수신 시 처리하는 예약 상태명. 현재 rectangle 노드는 회전 예산 초과를2번으로 보고함 |
| 6 | Calibration service rejected request | 초기화 Trigger 서비스가 success=false 응답 |
| 7 | Calibration service: ... | 초기화 서비스 future 처리 예외 |
| 8 | Preparation calibration timeout | 준비 시작부터90초 초과. 서비스 미발견·센서/영상/TF 대기도 포함 |
| 9 (변경) | RECOVERING | MOVING_TO_START/READY/PLAYING 중 localization.ready=false 또는 상태 수신1초 이상 중단 시 정지 대기. 위치 복구 후 이전 단계 자동 재개. ERROR/빨간 점멸 대신 초록 점멸 |
| 10 | GoToPose: ... | 준비 이동의 현재 goal_id에 해당하는 MotionResult.FAILURE 수신. 현재 attack_controller에서는30초 이동 제한 또는 비정상 pose/target 숫자(NaN/Inf)가 명시적 실패 원인. 선택한 다른 GoToPose 제어기의 실패도 여기에 전달됨 |
| 11 | GoToPose / stop confirmation timeout | 시작 위치 이동/정지 확인이60초 초과. 도착 이후 선속도≤0.01m/s·각속도≤0.05rad/s를0.3초 유지해야 정지 확인 |
| 12 | GoToPose result missing or stale | 이동 시작3초 이후에도 아직 도착하지 않았고, 결과 수신이0.7초 넘게 없음 |

1~5는 CALIBRATING 중 새 초기화 세션의 상태를 받았을 때 직접 ERROR로 전환된다.
초기화가 끝난 이후에는9번 대기로 처리될 수 있다. 10번은 MOVING_TO_START에서
해당 goal_id의 결과만 인정한다. 모든 시간은 현재 기본 설정 기준이다.

## 빨간 ERROR로 직접 바뀌지 않는 정지/표시

- 경로 없음: 1초 정체 후 충돌 검사에 통과하는 약10cm 직선 후진 우선,
  바퀴 피드백 적분 거리 도달 시 정지/재계획. 뒤가 막히거나 후진이 진전하지 않으면
  기존 전진/회전 대체 탈출 후보를 평가. 준비 이동이 오래 실패하면10/11로 이어질 수 있음.
- MPC 해 미도착·활성화 거부: zero command 및1초 정체 복구. 이것 자체는 ERROR가 아님.
- 장애물 정지: 정지 예측 검사와 탈출 후보 평가. 이것 자체는 ERROR가 아님.
- 심판 데이터 단절/유효하지 않음, START 확인 실패: STOPPED. 팀 LED 유지.
- 사용자 STOP, GOAL/OUT에 의한 정지/복귀: 직접 ERROR가 아님.
- LED 서비스 미응답: 상태에 unavailable/timeout을 기록하며 이것만으로 ERROR 전환하지 않음.
- LiDAR 디버그 화면의 빨간 포인트: 외벽 정합에서 제외된 점 표시이며 오류가 아님.

RECOVERING에서는 일반 제어 명령을 전달하지 않는다. 오래 끊긴 외벽 정합은
기존 odom 예측/180° 후보/위치·각도 오차 제한을 통과한 새 외벽 관측3개로 복구한다.
이전 좌표계 기준을 보존하며, 임의의 반대편 Pose로 초기화하지 않는다.
사용자 STOP·심판 정지/단절은 대기를 취소하며, 이후 관측이 돌아와도 자동 출발하지 않는다.

근거: src/pinky_soccer_viz/pinky_soccer_viz/{match_preparation,preparation_core,attack_controller}.py,
src/pinky_soccer_world_model/pinky_soccer_world_model/rectangle_localization_node.py.
