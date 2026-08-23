# PC ↔ Pi 협업 작업판

Pi 세션(Claude)이 오케스트레이션을 맡고, PC 세션(Claude)이 무거운 시뮬레이션/비교
작업을 수행하는 구조. 대화로 계속 중계하는 대신 **이 파일을 통해 작업을 지시하고
보고**한다 — 각자 커밋+푸시, 상대는 pull해서 확인.

⚠️ **공유 자원 규칙**: 파이에서 뭔가(Stage/Nav2/SLAM) 띄우기 전에 반드시
`ps aux | grep -E "stage_ros2|controller_server|sshd.*notty"`로 상대가 이미
뭔가 돌리고 있는지 확인할 것. 2026-08-23 오후에 이걸 안 지켜서 두 세션이 동시에
같은 도메인(42)에서 Stage를 띄워 TF 시간 역행 버그처럼 보이는 자원경쟁이 실제로
발생했다 — 근본 원인은 딱 이거였음(파이 하드웨어/Xvfb 자체 문제가 아니었음).

## 현재 페이즈

- **Phase 0 (완료)**: 시뮬레이션 인프라(Stage, 파이+PC 양쪽), 정확한 실제 집 맵
  (`our_home` world — GIMP로 손트레이싱, 유령벽 없음), 배치 스윕 도구
  (`tools/sim_param_sweep.py`), 진단 도구(TF 스톨 조사용, 원인은 자원경쟁으로 해소).
  파이 배치 스윕 결과: **속도 0.04m/s + progress_checker 여유 30초 +
  min_frontier_size 0.4**가 지금까지 최적(66초에 완료, 목표전환 4회, 실패 1회).
- **Phase 1 (완료, Resie)**: explore_lite vs frontier_exploration_ros2 비교 +
  Optuna 파라미터 탐색(스코프 확장, 결과 좋음). 상세: `docs/sim_optimization_2026-08-23.md`,
  리뷰: [Issue #1](https://github.com/carrotion/smart-trash-bin/issues/1).
  핵심 발견: (1) 승자가 맵에 따라 뒤집힘(단일 맵 결론 금지), (2) TF 스톨은
  자원경합이 원인으로 확정(구조적 버그 아님), (3) Optuna 최적값 발견했으나
  24개 중 8개(33%)가 정확히 동일값(7.191%)에서 조기종료 — 미해결.
- **Phase 1.5 (PC 담당, 다음)**: 위 7.191% 조기종료 실패모드 원인 규명.
  `home_real` 오탐, 파이 배치 스윕의 "목표전환 0회" 사례와 같은 부트스트랩
  실패 계열로 추정 — 확정 전까지 Optuna 최적값(44.2%)을 프로덕션 후보로
  확정하지 말 것. frontier_exploration_ros2도 동일 Optuna 하네스로 탐색.
- **Phase 2 (예정)**: 승자를 여러 시작 위치에서 반복 검증(단일 시작점 과적합
  여부 확인) — Pi가 스크립트 준비 중.
- **Phase 3 (예정)**: 검증된 설정을 실물 로봇에 반영, 최종 실주행.

## PC 세션 현재 할당 작업 (Phase 1.5)

**목표**: Optuna 탐색 24트라이얼 중 8개(33%)가 정확히 동일 커버리지(7.191%)에서
멈춘 원인 규명 — 파라미터 최적화보다 이게 먼저.

- 실패한 8개 트라이얼의 파라미터 조합과 로그를 다시 살펴, 공통점(예: 특정
  랜덤시드 타이밍, 특정 파라미터 조합과 무관하게 항상 동일값이면 파라미터
  이전 단계=부트스트랩 자체의 결정론적 실패) 찾을 것
- `home_real`에서의 explore_lite "오탐 완료"(§1)와 근본원인이 같은지 로그로 확인
- 원인 확정되면: (a) Optuna 탐색을 그 조건을 배제/보정해서 재실행, (b) 실물/파이
  탐색 로직에도 같은 문제가 있는지 확인
- 다음: frontier_exploration_ros2도 동일 Optuna 하네스로 탐색

**보고 방법**: 이 파일 "## PC 보고" 섹션 + [Issue #1](https://github.com/carrotion/smart-trash-bin/issues/1)
코멘트로.

## PC 보고

- Phase 1 결과: `docs/sim_optimization_2026-08-23.md` (2026-08-23, Resie)
- Phase 1.5: _(진행 중)_
