# 보관 안내 (2026-11-23)

이 폴더는 연습으로 만든 스킬 계층입니다. 빌드·실행 대상이 아니며 참고용으로만 둡니다.

- 새 작업 방향: 공격만 먼저, 실물 캘리브레이션 → MPC 제어로 지정 위치 이동 검증 → 공 접근 → 상대 회피 이동 → 드리블.
- 다시 쓸 만한 부분: `pinky_soccer_skills/sim/`(ROS 없는 MuJoCo 시험대·센서 모델·채점·궤적 그림), `scenarios/`, `SKILLS.md` 2절의 측정 사실
  (범퍼 밀착 공은 카메라에서 안 보이고 간격 4~5 cm부터 보임, 재질 구름 감속 0.35 m/s² 가정 등).
- `integration.patch`: 당시 `pinky_soccer_viz` launch(`decision:=skills|bt`, 시뮬레이션 속도 기본 0.25/1.5)와 README·ARCHITECTURE 수정 내용. 저장소에는 적용하지 않았습니다.
- 다시 빌드하려면 `src/` 아래로 옮기고 `COLCON_IGNORE`를 지웁니다. 파일 경로(`SKILLS.md`의 그림 등)는 패키지 폴더 기준입니다.
