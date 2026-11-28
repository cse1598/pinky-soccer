# main 통합 후 MuJoCo 두 로봇 경기 시험 — 2026-11-28

기준 main 4597ae0. 정확히 2회 시도. Legacy 대 Legacy, ball_source=hybrid,
headless=true, field=hardboard, ROS_DOMAIN_ID=193. 기존 worktree/다른 ROS overlay를
source하지 않고 main의 .venv 및 install/local_setup.bash 사용. 실물/SSH/Gazebo/직접 영상 열람 없음.

성공 기준은 한 팀의 영상 심판 득점 3점. 시험 드라이버만 중앙 공/양팀 준비 확인 후
1초 뒤 START를 보낸다. Ground Truth는 로그 평가에만 사용한다.

| 회차 | 팀 득점 노랑:파랑 | 결과 |
|---|---|---|
| 1 | 0:0 | 양팀 World Model JSON 발행 예외로 종료, 조기 중단 |
| 2 | 1:0 | 첫 골·양팀 복귀·공 중앙 배치·재개 확인. 다음 랠리 75 sim초 초과 실패 |

3점 선승 완주 0/2. trial_*/result.json의 scores는 팀 득점이 아니라
[노란 골대 진입수, 파란 골대 진입수]이므로 팀 득점과 순서가 반대다.
1회차 result.json은 중단 후 launch/topic 로그를 근거로 작성했으며 수동 작성임을 명시했다.
2회차 result.json은 기존 verify_attack_match.py가 작성했다.

## 발견 및 수정

1. Hybrid referee XY가 NumPy 배열이므로 접촉 영역 비교가 numpy.bool_을 반환한다.
   false인 경우 Python and 단락 평가로 그 값이 유지되어 json.dumps(contact)가 TypeError를
   발생시켰다. World Model 발행 경계에서 present를 Python bool로 변환했다.
   2회차에는 해당 예외가 재발하지 않았다.
2. 두 번째 랠리에서 공은 약 (0.823, 0.137)에 머물렀다. 노란 로봇은 약 (0.763,-0.006)
   골문 근처에서 진전하지 못했다. 뒤쪽 (0.525,-0.034)으로 먼저 나가는 경로가 생성되었으나
   goal_id=0의 GoBehindBall은 기본 전진 추종만 허용하여 후퇴 경로 앞에서 회전하려 했다.
   후퇴 탈출 경로 첫 구간의 방향을 유지하고 다음 구간 진입 시 해제하도록 수정했다.
   로봇/공 추정과 기록된 참 위치를 비교한 결과 이 정체를 위치 추정의 큰 오차로 설명할
   증거는 없었다. 공격 방향 선택/경합 등 다른 경기 정체 원인을 모두 해결했다는 의미는 아니다.

## 수정 후 검증 범위

- Hybrid 실제 node.tick JSON 발행 회귀, 기존 Hybrid/심판 전용 모드 테스트 통과.
- 관측된 후퇴 경로를 양팀 방향으로 재현한 바퀴 명령 테스트 통과.
- 실제 AttackController.tick에서 뒤로 빠지는 명령과 다음 구간의 방향 해제 테스트 통과.
- 기존 이동/안전/활성 탈출 회귀 포함 총 27개 통과.
- Path MPC 추가 후 갱신되지 않았던 기존 테스트 더블 2개에 path_mpc=None 추가.
- 수정한 World Model/Viz 두 패키지 재빌드 성공.
- 두 번째 수정 뒤 전체 경기 추가 실행 없음. 3점 경기 해결 여부 미검증.
- 시험에서 생성한 모든 launch 프로세스 종료 확인. MPC solver/비용/안전 제약,
  보정 데이터, BT 전략, 경기장/물리 설정은 변경하지 않음. 커밋하지 않음.

원본: trial_1/launch.log, topics.jsonl, driver.log 및 trial_2/ 동명 파일.
trial_2/yellow_probe.json은 정지 직전 진단으로, 경로는 확인되었지만 goal/cmd는 이미
시험 STOP으로 바뀐 최신 값이므로 동일 시각의 활성 명령이라고 해석하지 않는다.
