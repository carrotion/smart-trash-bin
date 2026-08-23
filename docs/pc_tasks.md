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
- **Phase 1 (PC 담당, 진행 중)**: explore_lite vs frontier_exploration_ros2 비교.
- **Phase 2 (예정)**: Phase 1 승자를 여러 시작 위치에서 반복 검증(단일 시작점
  과적합 여부 확인) — Pi가 스크립트 준비 중.
- **Phase 3 (예정)**: 검증된 설정을 실물 로봇에 반영, 최종 실주행.

## PC 세션 현재 할당 작업 (Phase 1)

**목표**: explore_lite vs frontier_exploration_ros2, 같은 조건에서 공정 비교.

- 설정: potential_scale=2.0 / gain_scale=2.0 / min_frontier_size=0.4,
  속도 0.04m/s, progress_checker 여유 30초 (파이에서 검증된 값 그대로)
- 월드: `our_home`, 같은 시작지점
- 반복: 각 전략 **최소 3회**(재현성 확인 — 단발성 결과로 결론내지 말 것)
- 지표: 완료여부 / 소요시간 / 목표전환 횟수 / progress_fail 횟수 / 충돌 횟수 /
  최종 맵 커버리지 %(map_coverage_logger.py 활용)
- **원본 로그도 남길 것** — 숫자 요약만으로는 Pi가 검증할 수 없음

**보고 방법**: 이 파일 아래 "## PC 보고" 섹션에 결과 표 + 로그 파일 경로(pc_results/
또는 저장소 내 경로) 채워서 커밋+푸시. Pi가 pull해서 직접 로그 검토 후 승자 결정.

## PC 보고

_(아직 없음 — Phase 1 완료 후 여기에 작성)_
