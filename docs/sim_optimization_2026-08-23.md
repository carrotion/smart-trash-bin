# 시뮬레이션 기반 탐색 전략 비교 + 파라미터 최적화 (2026-08-23)

`docs/exploration_strategy_research.md`(시나리오 A~D 계획)의 후속 — 실제로
PC(WSL, 12코어)와 Pi 양쪽에서 시뮬레이션을 돌려 얻은 실증 결과.

## 1. explore_lite vs frontier_exploration_ros2 비교

- 환경: PC(WSL, ROS_DOMAIN_ID=43), Stage 시뮬레이터, `home_real.world`(SLAM 스냅샷
  변환, 이후 폐기) → `our_home.world`(사용자가 실제 도면을 GIMP로 직접 트레이싱,
  오차 없는 벽 — 채택)
- `home_real`에서는 explore_lite가 3번 실패 후 57초 만에 "완료"를 오탐(맵이
  거의 안 그려진 채 종료). frontier_exploration_ros2는 포기 안 하고 400초
  내내 재시도.
- `our_home`에서는 반대로 explore_lite가 188초에 정상 완료, frontier_exploration_ros2는
  400초 동안 계속 재시도만 함 — **맵에 따라 우열이 뒤집힌다.** 한 맵의 결과로
  전략을 확정하면 안 된다는 근거.
- 상세 로그: `pc_results/comparison_results.md` (Pi로 scp 전송됨, git 미포함 —
  `.gitignore`의 `pc_results/`)

## 2. "Transform data too old" 스톨 — 원인은 알고리즘이 아니라 자원경합이었다

### 처음 세운 가설과 반증
- PC에서 `home_real`/`our_home` 양쪽에서 `Transform data too old`(map->odom TF
  지연)가 ~28초 주기로 반복 재현됨.
- `transform_tolerance`를 0.3→1.0으로 올려도 실패율 불변 — 오차 간격이
  tolerance+~0.1s로 그대로 따라옴(여유값 문제가 아님을 시사).
- `tools/cpu_profiler.py`(`/proc/[pid]/stat` 델타 기반 순간 CPU%, `ps -o pcpu`의
  누적평균 함정을 피함)로 스톨 순간을 봤을 때 관련 프로세스 전부 15% 미만 —
  compute-bound가 아니라는 결론까지 냈었음.

### 실제 원인 (Pi에서 재현 후 확정)
사용자(Unkle Karro)가 Pi에서 좀비 시뮬레이션 프로세스(이전 세션이 정리 안 되고
계속 CPU를 먹고 있던 것)를 발견해 제거 — load average 6.55 → 0.56으로 급감.
같은 조건(max_vel=0.04, movement_time_allowance=30s, min_frontier_size=0.8,
`tools/sim_param_sweep.py` 사용)으로 재현:

| 조건 | 부트스트랩 실패(목표전환 0회) | Transform data too old |
|---|---|---|
| 좀비 프로세스 있음 + 대시보드 4개 실행 중 | 3/6 (SIGKILL 몰살 포함, SSH 단절도 1회) | — |
| 좀비 제거 + 대시보드 4개 중지 | **0/6** | **0/6** |

**결론: 오늘 추적한 TF 스톨은 구조적 버그가 아니라 자원경합(좀비 프로세스 +
상시 대시보드 서비스)이 원인이었다.** PC 쪽에서도 좀비 `Xvfb` 프로세스가
발견돼(4시간 가까이 방치) 같은 계열의 문제였을 가능성이 높다.

**교훈**: 시뮬레이션 실행 전 항상 `ps aux | grep -E 'ros2 launch|stage_ros2|Xvfb'`로
잔여 프로세스 확인. Pi에서는 절대 병렬로 여러 시뮬레이션을 동시에 돌리지 말 것
(4코어/4GB + 실물 로봇 서비스 상시 실행 중이라 자원경합에 취약). 재현 도구:
`tools/startup_stall_repro.py`.

## 3. Optuna 기반 explore_lite 파라미터 최적화 (PC, 병렬)

### 방법론
- `tools/optuna_search.py`: TPE 샘플러, 목적함수 = t=150초 시점 실측 맵
  커버리지%(`tools/map_coverage_logger.py`, self-report 아닌 `/map` 직접 구독).
  "완료 선언"을 신뢰하지 않는 이유는 §1의 explore_lite 오탐 전례 때문.
- 동시성: N=1/2/4를 각각 60초씩 실측(`robot/concurrency_bench.sh`) 후 N=4 채택 —
  12코어 기준 순간 최대 598%(~6코어)로 여유 있었고 치명적 실패 없음. 단, 표본이
  작아 확정적 결론은 아님.
- 각 트라이얼은 `ROS_DOMAIN_ID` 100~103 중 하나를 락으로 빌려 격리.
- 탐색 파라미터: `potential_scale`(0.5~4.0), `gain_scale`(0.5~4.0),
  `min_frontier_size`(0.1~1.0), `min_frontier_distance`(0.2~1.0).

### 버그 2건과 수정 (실행 중 발견)
1. `study.optimize()`에 `catch` 없이 실행 → 트라이얼 1개의 `TimeoutExpired`가
   전체 탐색을 중단시킴(24개 목표 중 6개만 처리하고 죽음). `catch=(Exception,)`
   추가로 해결.
2. `subprocess.run(timeout=...)` 만료 시 직접 띄운 bash만 죽고 그 안에서
   `&`로 띄운 xvfb-run/ros2 launch 자식들은 안 죽어 고아가 될 위험 —
   `tools/sim_param_sweep.py`(Remi 작성)와 동일하게 `setsid`+`killpg`로 프로세스
   그룹 전체를 정리하도록 수정.

### 결과 (24 트라이얼, world=our_home, duration=150s)

**최적값**: `potential_scale=1.56, gain_scale=2.50, min_frontier_size=0.92,
min_frontier_distance=0.58` → 커버리지 **44.2%**

현재 프로덕션 값(`potential_scale=2.0, gain_scale=2.0, min_frontier_size=0.8,
min_frontier_distance=0.6`)과 비교하면 `min_frontier_size`/`min_frontier_distance`는
거의 같은 영역인데, `potential_scale:gain_scale` 비율이 다르다
(1.56:2.50≈0.62 vs 2.0:2.0=1.0 — gain 쪽이 더 우세).

**⚠️ 더 중요한 발견 — 뚜렷한 조기종료 실패 모드**: 24개 중 **8개(33%)가 정확히
동일한 값(7.191%)에서 멈췄다.** 우연이 아니라 특정 조건에서 부트스트랩 직후
탐색이 조기 종료되는 패턴 — §1의 `home_real` 오탐과 같은 계열의 문제로 보인다.
파라미터만으로는 명확히 구분되지 않음(min_frontier_size가 0.17~0.92까지 실패군에
고르게 분포) — 표본 24개로는 원인 특정에 부족, 추가 조사 필요.

study DB: `/tmp/optuna_search/study_d150.db`(로컬, git 미포함) —
`optuna-dashboard sqlite:///study_d150.db`로 시각화 가능.

## 다음 단계 제안
1. "7.191% 조기종료" 실패 모드 원인 규명 — 어떤 조건에서 왜 부트스트랩 직후
   포기하는지(§1 `home_real` 오탐과 같은 근본원인인지 확인).
2. 트라이얼 수를 늘려(현재 24개는 4차원 공간 탐색에 적음) 최적값 신뢰도 확보.
3. frontier_exploration_ros2 쪽도 동일한 Optuna 하네스로 별도 탐색.
4. 최적 후보를 여러 맵(cave/hallway/our_home)에서 n≥3회씩 재검증(오버피팅 방지).
5. 최종 후보를 Pi(실물 하드웨어)에서 교차검증 후 프로덕션 반영 여부 결정.
