# 제어 Phase 0/1 실측 자료

2026-11-27 [codex]. 해석·조건·실행 방법은 [보고서](../../CONTROL_PHASE0_PHASE1.md).

- `baseline/`: 변경 전 실제 dirty 소스384개/해시/환경/요청 원문. 별도 checkout에 복구한다.
- `effective_models.json`: target/profile 기본 loader 출력. 실물 실행 노드에서 읽은 값은 아니다.
- `preservation.json`: baseline 대비 변경 및 진단문 제거 후 controller AST 비교.
- `results.json`: MuJoCo 10세션(고정입력1+GoToPose9), GoToPose34건의 성공33/실패1 모두 포함.
- 각 세션: receipt 시간축 `motion.png`/`timing.png`, `summary.json`, episode/command/pose/solver/slot/job/CPU CSV.
- 각 세션 `samples.jsonl.gz`: 모든 정규화 표본. source stamp와 ROS receipt 및 monotonic receipt를 구분한다.
- 각 세션 `bag.tar.gz`: 선택 토픽의 원본 CDR SQLite bag. `launch.log.gz`, 입력 episode와 metadata도 보존한다.
- `artifacts.json`: 세션 산출물 SHA256. `implementation.tar.gz`/`implementation.json`: 이번 도구·진단·문서 스냅샷.
- `tests.baseline.txt`/`tests.final.txt`: 실제 pytest 출력. `checks.txt`: archive/AST/F/compile 확인.

반복은 recorder 개정, 초기 상태, 진단 ON/OFF, 기동 barrier가 다르다.
성능 개선율/정확한 일반화 성공률/제어기 순위를 이 합계로 만들지 않는다.
`mujoco_basic_trace`에 실제 연속 slot 누락 FAILURE가 있다. 최종 단독 ON은6/6 성공/slot119개 누락0이지만
OFF에도 주행 중 zero가 관측됐으므로 안정화 완료가 아니다.
초기 반복에는 별도 그래프 생성 부하가 병행된 경우가 있으며 최종 단독 두 세션은 별도로 실행했다.

bag 분석용 복원 예시: 새 임시 디렉터리에서 `tar -xzf PATH/bag.tar.gz`.
이 기록은 완전한 QoS/TF-static/영상 재생 세션이 아니다. 실물 명령에 직접 재생하지 않는다.
Gazebo 이전 원시 시험은 `/tmp/pinky-control-baseline/gazebo_*`에 남겨 두었으나, 사용자 지시에 따라
이후 업데이트·추가 시험/비교 보고 대상으로 사용하지 않았다. 실물 검증은 수행하지 않았다.
