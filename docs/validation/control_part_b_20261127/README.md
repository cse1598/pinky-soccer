# PART B 검증 기록

[기능·실행 방법·검증 범위](../../CONTROL_PART_B.md).

- `results.json`: 개발 단계별 성공·실패 및 중간 zero 구간을 포함한 관측 결과.
- `strict_stop_fast_final/`: 빠른 안전 경계를 적용한 6개 이동. 모두 도착했지만 중간 zero는 남음.
- `lease_final/`: 다음 slot의 유효기간 확인 후 직선 이동 완료, 중간 zero0.
- `pass_through/`, `handoff/`: 실제 MuJoCo BT에서 두 목표 완료. 각각 인계 주변 명령7개/zero0.
- `checkpoint_before_residual_fix/`: 정지 히스테리시스에 잔여 오차가 남아 실패한 수정 전 자료.
- `strict_stop_final/`: 해당 대기 문제 수정 후6/6 완료.
- `development_runs.json`: 이전 개발 중 실패도 보존. 다른 revision/시작 상태의 속도를 직접 비교하지 않음.
- `tests_release.txt`: 최종 정책·포트·기존 UI 요청·launch 53 tests PASS.
- `tests_selected.txt`, `tests_focused.txt`, `tests_core.txt`, `tests_bt.txt`: 이전 통과 기록(서로 중복).
- `safety_microbenchmark.json`: 먼360포인트 합성 상태의 함수 실행 시간. 경기 전체 성능 수치가 아님.
- `final_source.json`, `final_fix.patch`: 최종 실행 소스 해시와 중간 커밋 이후 PART B 코드 수정.
- `preserved_inputs.json`: 기존 MPC solver/기하/동역학/월드 모델 관련47개 파일 보존 검사.
- `manifest.json`: 보관 자료 SHA256. raw bag은 일부 최종 실행의 `bag.tar.gz`로 보관.

최종 소스는 미커밋 수정이 포함된다. 별도 심판 패키지의 병행 변경은 이 작업에서 수정하거나 되돌리지 않았다.
실물·Gazebo·모든 동적 장애물 조건 및 임의 Skill 인계 검증은 미완료다. 검증 자료는 Git에 추가하지 않았다.
