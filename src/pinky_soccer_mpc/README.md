# pinky_soccer_mpc

Pinky Pro용 acados NMPC 패키지입니다. **현재는 1단계**로 acados 환경 연결과 솔버 검증만 있습니다.
목표 위치 이동 NMPC(3단계), 장애물 회피(5단계)는 이후 단계에서 추가합니다.

## acados 연결 (`acados_setup.py`)

acados는 `external/acados` 서브모듈(커밋 `c15f50a9a`, upstream main과 같음)입니다. acados를 쓰는 코드는
`pinky_soccer_mpc.acados_setup.acados()`로만 불러옵니다. 이 함수가

1. acados 위치를 찾고(`ACADOS_SOURCE_DIR` → 이 파일 위쪽의 `external/acados`) 환경변수를 설정
2. **NumPy 호환**: acados_template(`acados_ocp.py`)이 NumPy 2 전용 별칭 `np.bool`을 씁니다.
   Jazzy의 NumPy 1.26에는 없으므로, 없을 때만 `np.bool = np.bool_`(NumPy 2의 정의와 동일)를 추가합니다.
   upstream main(2026-11-22)에도 아직 그대로이며 setup.py가 NumPy 2를 요구하지도 않습니다.
   서브모듈·NumPy 버전은 바꾸지 않습니다. upstream이 `np.bool_`로 고치면 이 처리는 지웁니다.
3. acados 공유 라이브러리를 미리 적재해 `LD_LIBRARY_PATH` 설정 없이 생성 솔버가 동작

생성 C 코드는 `~/.cache/pinky_soccer_mpc`(또는 `PINKY_MPC_CACHE`)에 만들고 저장소에는 두지 않습니다.

## 검증

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run pinky_soccer_mpc check_acados
```

핑키 바퀴 동역학(`pinky_soccer_dynamics`)으로 점대점 OCP를 생성·컴파일·SQP로 풀고
반환 상태 0, KKT 잔차, 입력 한계, 바퀴 가속 경로 제약, NumPy 모델 재시뮬레이션 일치, 목표 도달을 확인합니다.
`test/test_acados_verify.py`가 같은 검사를 자동으로 합니다(casadi/acados가 없으면 건너뜀).

주의: acados의 경로 비선형 제약(`con_h_expr`)은 1..N−1 단계에만 걸립니다. 실제로 로봇에 보내는
첫 입력의 제약은 `con_h_expr_0`/`lh_0`/`uh_0`로 따로 넣어야 합니다(검증에서 실제로 위반을 확인 후 수정).

## 빌드

casadi·acados_template은 `.venv`에만 있으므로 `.venv` 파이썬으로 빌드합니다.

```bash
.venv/bin/python -m colcon build --symlink-install --packages-select pinky_soccer_dynamics pinky_soccer_mpc
```
