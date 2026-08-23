# PC ↔ Pi 협업 작업판

Pi 세션(Claude)이 오케스트레이션을 맡고, PC 세션(Claude)이 무거운 시뮬레이션/비교
작업을 수행하는 구조. 대화로 계속 중계하는 대신 **이 파일을 통해 작업을 지시하고
보고**한다 — 각자 커밋+푸시, 상대는 pull해서 확인.

⚠️ **공유 자원 규칙**:
1. (상대방 확인) 파이에서 뭔가(Stage/Nav2/SLAM) 띄우기 전에 반드시
   `ps aux | grep -E "stage_ros2|controller_server|sshd.*notty"`로 상대가 이미
   뭔가 돌리고 있는지 확인할 것. 2026-08-23 오후에 이걸 안 지켜서 두 세션이 동시에
   같은 도메인(42)에서 Stage를 띄워 TF 시간 역행 버그처럼 보이는 자원경쟁이 실제로
   발생했다 — 근본 원인은 딱 이거였음(파이 하드웨어/Xvfb 자체 문제가 아니었음).
2. **(자기 자신 확인, 2026-08-23 밤 추가)** 배치/병렬 실행 스크립트는 실행 전후로
   **자기 이전 실행분의 잔존 프로세스**도 확인할 것 — 상대 세션과의 충돌이 아니라
   **같은 세션의 이전 배치가 남긴 좀비**도 똑같은 자원경쟁을 일으킨다(Resie가
   Optuna 배치에서 실제로 발견: trial0/trial13이 ~2시간 살아남아 새 실행과
   같은 ROS_DOMAIN_ID를 충돌시킴 → Phase 1 비교 결과 6개 무효화됨).
   **원인**: 프로세스 그룹 정리(`killpg`)가 타임아웃 예외 경로에만 있고
   정상반환 경로엔 없으면, "정상 종료처럼 보이는데 자식이 안 죽는" 케이스를
   놓친다. **수정 방향**: `killpg`를 예외 처리와 무관하게 매 실행 끝에
   무조건 호출할 것(`tools/sim_param_sweep.py`는 이미 이렇게 돼있음 — 확인됨,
   `tools/optuna_search.py`의 `run_trial`은 아직 수정 필요).

## 미션과 계층 구조 (2026-08-23 밤 재정립)

**최종 목표**: 로봇이 집 안을 스스로 돌아다니며 정확한 지도를 그릴 수 있어야
"일을 할 수 있다"(쓰레기통으로서의 실제 기능). 이게 안 되면 2호기 제작 등
다른 모든 후속 작업이 막힌다.

의존 관계는 이렇게 계층으로 본다:

```
Layer 0: 자기 위치 정확히 인식 (오도메트리+IMU+센서융합, 실물)
   ↓ 이게 안 되면 아무리 좋은 전략도 실물에선 배신당함
Layer 1: 정확한 지도 그리기 (Layer 0이 좋아야 SLAM도 좋음)
   ↓
Layer 2: 탐색 전략/커버리지 알고리즘
   ↓
Layer 3: 실제 작업(쓰레기 수거 등) — "상상력과 소프트웨어의 영역"
```

⚠️ **중요한 자기반성**: 지금까지의 Track A(Phase 0~1.5, 아래)는 전부 **Layer 2만
다뤘다.** Stage 시뮬레이터는 완벽한 오도메트리를 주기 때문에 — 실물이 겪는 자이로
드리프트/바퀴슬립/자기장 프리즈 같은 Layer 0 문제가 시뮬 안엔 아예 없다. 즉
"탐색 전략 최적값"을 아무리 잘 찾아도, 그건 "위치를 완벽히 아는 상태"를 가정한
것이라 실물에 그대로 못 옮긴다. 그래서 **Track 0을 병렬로 새로 세운다.**

## Track 0 — 자기위치 인식 정확도 (Remi 담당, 실물 로봇 전용)

**목표**: `our_home`(GIMP로 정확히 손트레이싱한, 오차 없는 정답 지도)을 **고정된
정답지**로 써서, 실물 로봇이 스스로 추정한 위치/궤적이 이 정답과 얼마나 어긋나는지
정량적으로 측정하고 좁힌다. 시뮬레이션으로는 원리적으로 할 수 없는 작업 — 실물
센서(자이로, 휠 엔코더, EKF)가 대상이라 Resie가 기여할 부분이 없다(PC는 참여 불가).

- 기존에 만들어뒀던 `tools/round_trip_drift.py`(왕복 주행 후 원위치 이탈 측정)를
  먼저 실행 — 이미 있었는데 한 번도 안 돌려봄
- 알려진 지점(예: 시작점 → 특정 방 중앙 → 시작점 복귀) 간 실측 오차를
  `our_home` 좌표와 대조
- 이미 열려있는 Layer 0 미해결 항목들을 이 안에서 같이 정리:
  회전 중 한쪽 바퀴 안 도는 현상(`turn_debug` 로깅 활용), 바퀴 슬립 인과관계 불명,
  자기장 프리즈(Davi 담당분과 조율 필요)
- **PC 대비 R&R**: Track 0은 Remi가 실물로 직접 수행. Resie가 도울 수 있는 부분은
  "오차 계산/비교 도구"를 코드로 만들어 검증해두는 것(합성 노이즈를 넣은 가짜
  궤적으로 도구 자체를 먼저 테스트 — 실물 데이터 없이도 가능) — 필요해지면 별도
  이슈로 요청.

**진행 상황 (2026-08-23 밤, Remi)**:
- `round_trip_drift.py` 실행(20회 왕복, 실물): 휠단독 160.9cm/+13.1° vs
  EKF 162.0cm/**+33.1°** — **EKF 융합이 휠 단독보다 요(yaw)를 2.5배 더 틀어지게
  만듦**(원래는 EKF가 더 나아야 정상). 사용자가 실측(로봇이 시작위치보다 뒤로
  약 1.3m)으로 교차검증 — 계산값과 오더 일치.
- 원인 진단(`tools/round_trip_gyro_diag.py` 신규, 연속 로깅): **모터 시작/정지/
  역전 순간에 자이로z가 정지 기준선(표준편차 0.057rad/s)의 5~8배로 순간 튐**
  (예: +0.31~+0.44 rad/s). 실제 회전이 아니라 모터 전류서지/진동으로 추정.
  전환 후 경과시간별 분석: 20~100ms 들뜸, 120~150ms부터 기준선 복귀(표본 15개,
  거친 추정).
- **수정 완료**: `imu_publisher_node.py`가 `/cmd_vel` 전환을 추적해서, 전환 후
  150ms 동안 자이로z 공분산을 0.0004→50.0으로 부풀려 EKF가 그 구간을 덜
  믿게 함(데이터를 버리지 않고 신뢰도로 표시 — 기존 orientation_covariance=-1
  관례와 동일 원리). 라이브로 게이팅 타이밍 검증 완료(0~140ms=50.0, 160ms부터
  0.0004로 정확히 복귀).
- **검증 완료 (재측정, 20회 왕복)**: 요(yaw) 이탈 — 휠 +13.1°→+5.3°,
  **EKF +33.1°→+5.2°(휠과 동등 또는 소폭 우위로 역전)**. 수정 전 "EKF가 휠보다
  2.5배 나쁨"이 수정 후 해소됨 — Track 0 첫 성과. (전체 드리프트 절대량은
  160.9cm→87.7cm로 같이 줄었는데 휠 채널도 같이 줄어서 측정 간 자연 변동으로
  추정 — 진짜 신호는 휠-EKF 상대 비교.) 상세: [Issue #2](https://github.com/carrotion/smart-trash-bin/issues/2)

## Track A′ — 탐색 전략 (Resie 담당, 시뮬레이션, Layer 2 전용임을 명시)

⚠️ 이 트랙의 결과는 **"위치를 완벽히 안다면 어디로 갈지"**에 대한 답이지,
"실물에서 바로 쓸 수 있는 최종 설정"이 아님 — Track 0이 어느 정도 안정돼야
여기서 나온 전략을 실물에 반영하는 게 의미 있다. 그래도 시뮬레이션은 실물 위험이
없어 계속 병렬로 진행 가치 있음.

- **Phase 0 (완료)**: 시뮬레이션 인프라, `our_home` 월드, 배치 스윕 도구.
- **Phase 1 (진행 중, 재작업 필요)**: explore_lite vs frontier_exploration_ros2
  비교 + Optuna 파라미터 탐색. 상세: `docs/sim_optimization_2026-08-23.md`,
  리뷰: [Issue #1](https://github.com/carrotion/smart-trash-bin/issues/1).
  ⚠️ **2026-08-23 밤: 비교용 6회 실행분(explore_lite/frontier 각 3회)이 자기
  좀비 프로세스 오염으로 무효 처리됨** — `optuna_search.py`의 `run_trial`이
  타임아웃 예외 경로에서만 `killpg`를 호출해서, 정상반환처럼 보이는 경로에서
  자식이 안 죽는 case를 놓쳤다(위 "공유 자원 규칙" 2번 참고). **우선 이 버그부터
  고치고(killpg를 예외와 무관하게 항상 호출), 그 다음 6회 비교를 재실행할 것.**
- **Phase 1.5**: Optuna 24트라이얼 중 8개(33%)가 정확히 동일값(7.191%)에서
  조기종료된 원인 규명 — Phase 1 재실행과 같이 진행 가능(별개 이슈).
- **Phase 2 (예정)**: 승자를 여러 시작 위치에서 반복 검증 — Remi가 스크립트 준비 예정.

## PC 세션 현재 할당 작업

1. ~~`optuna_search.py` `run_trial`의 killpg 버그 수정~~ **완료** (Resie, finally로 이동)
2. ~~Phase 1 비교 6회 재실행~~ **완료** — explore_lite 28.2%(변동폭 6배, 불안정)
   vs frontier 30.7%(일관적, 완료 0/3)
3. ~~Phase 1.5 원인 규명~~ **완료** — explore_lite가 첫 목표 실패(부트스트랩 시점
   TF 지연) 단 1회만으로 블랙리스트 오판하는 게 근본원인으로 확정됨
4. ~~Phase 1.5 완화책 (a)~~ **완료, 순차 조건에서만 유효**: `e07eed4`,
   순차 재검증(변동폭 6배→1.3배)은 성공했으나 N=4 병렬(Optuna와 동일 조건)에서는
   **4/4(100%) 조기종료 재현** — "Managed nodes are active"가 SLAM TF 안정성을
   보장하지 않음이 확인됨. (a)는 유지, 위에 (b) 추가 필요.
5. **(진행) Phase 1.5 완화책 (b)**: explore_lite에 `frontier_goal_selection.patch`와
   같은 방식으로 패치 — 첫 N회(N=2~3부터 시작, sweep으로 조정) 목표 실패는
   블랙리스트 대신 재시도. 결정 근거: [Issue #1 코멘트](https://github.com/carrotion/smart-trash-bin/issues/1#issuecomment-5385653990)
   ("SLAM 신호 대기"보다 일반적 — 부트스트랩/런 중간 자원경합 둘 다 방어).
   `robot/patches/README.md`에 적용법 기록할 것. 완료 후 N=4 병렬로 재검증.
6. frontier_exploration_ros2도 Optuna 하네스로 별도 탐색 — (5)와 병행 가능

**보고 방법**: 이 파일 "## PC 보고" 섹션 + [Issue #1](https://github.com/carrotion/smart-trash-bin/issues/1)
코멘트로.

## PC 보고

- Phase 1 결과: `docs/sim_optimization_2026-08-23.md` (2026-08-23, Resie) — 비교
  6회분은 오염으로 무효, 나머지(맵별 승자 뒤집힘, TF 스톨 자원경합 확정,
  Optuna 24트라이얼)는 유효
- **killpg 버그 수정 완료** (2026-08-23 밤, Resie): `tools/optuna_search.py`,
  `robot/phase1_compare.py` 둘 다 `finally`로 옮겨서 정상반환/타임아웃 무관하게
  항상 프로세스 그룹 정리하도록 고침. 오염됐던 좀비(trial0/trial13, PID
  70404/70406/75003/75005 + Xvfb 70400)는 정확한 PID로 kill 확인, 재확인 결과
  깨끗함.
- **Phase 1 재실행 완료** (좀비 정리 후 깨끗한 상태, `our_home`, 200초 캡, 3회씩):

  | explorer | rep | goal_switches | coverage% | completed |
  |---|---|---|---|---|
  | explore_lite | 0 | 1 | 7.2 | True (조기종료) |
  | explore_lite | 1 | 10 | 44.4 | True |
  | explore_lite | 2 | 19 | 32.9 | False (진행중) |
  | frontier | 0 | 13 | 32.2 | False (진행중) |
  | frontier | 1 | 7 | 28.4 | False (진행중) |
  | frontier | 2 | 13 | 31.5 | False (진행중) |

  평균 커버리지는 explore_lite 28.2% vs frontier 30.7%로 비슷하지만,
  **explore_lite는 변동폭이 6배(7.2~44.4%)로 매우 불안정**한 반면
  frontier는 28.4~32.2%로 훨씬 일관적(대신 200초 내 완료 0/3).
  원본 로그: `/tmp/phase1_compare/*.log`(로컬, git 미포함 — 필요하면 요청).

- **Phase 1.5 원인 확정** (좀비와 무관한 진짜 버그): `explore_lite_r0.log`를
  직접 확인한 결과, 정확한 인과관계가 나왔음:
  ```
  t=6s  Begin navigating (0,0) -> (0.52,0.18)   <- 첫 목표
  t=9s  Transform data too old (map->odom, 0.4s 지연) -> Goal failed
  t=13s explore_node: "All frontiers traversed/tried out, stopping."
  ```
  **시뮬레이션 시작 극초반(SLAM/코스트맵 초기화 부하가 몰리는 시점)에 TF가
  살짝 지연된 상태에서 explore_lite가 첫 목표를 보냈고, 그게 실패하자
  재시도 없이 단 1번의 실패만으로 "모든 프론티어 소진"으로 오판하고 완전히
  포기했다.** `home_real`의 "3번 실패 후 포기", 파이의 "0회 전환" 사례와
  같은 계열 — **explore_lite의 프론티어 블랙리스트 로직이 초반 부트스트랩
  타이밍의 일시적 TF 지연에 지나치게 취약**한 게 근본원인으로 보임.

  완화 방향(미착수, 논의 필요): (a) Nav2/SLAM이 안정된 뒤 explore_node를
  지연 기동, (b) explore_lite 패치로 첫 N회 실패는 블랙리스트 대신 재시도,
  (c) 초반 구간만 transform_tolerance를 넉넉히.

- **완화책 (a) 적용 + 재검증 완료** (2026-08-23 밤, Resie): Remi 결정대로 (a)
  먼저 적용. 고정 시간 지연 대신 `lifecycle_manager`의 "Managed nodes are
  active" 로그를 `OnProcessIO`로 감지해서 그 시점에 explore_node/frontier_node를
  시작하도록 `sim_autonomous_mapping.launch.py` 수정(부하에 따라 부트스트랩
  시간이 달라져도 견고). 60초 스모크 테스트로 첫 목표부터 정상 진행 확인 후,
  Phase 1과 동일 조건(3회씩, 좀비 없는 상태)으로 재검증:

  | explorer | rep0 | rep1 | rep2 | 평균 | 변동폭 |
  |---|---|---|---|---|---|
  | explore_lite (수정 전) | 7.2% | 44.4% | 32.9% | 28.2% | 6배 |
  | **explore_lite (수정 후)** | 42.0% | 35.1% | 32.0% | **36.4%** | **1.3배** |
  | frontier (수정 후) | 30.2% | 28.6% | 32.9% | 30.6% | 수정 전과 거의 동일 |

  **"조기종료(1회 시도 후 포기)" 패턴 완전히 사라짐.** explore_lite가 이제
  frontier보다 평균 커버리지도 높고(36.4% vs 30.6%) 안정성도 크게 개선됨 —
  (b) 패치 없이 (a)만으로 충분해 보임. 단 3회는 여전히 작은 표본, 확정하려면
  반복 수를 더 늘려볼 가치 있음.

- **⚠️ N=4 병렬 조건 재검증 — (a) 무효화됨** (2026-08-23 밤, Resie): Remi 요청대로
  Optuna와 같은 조건(ROS_DOMAIN_ID 100~103, N=4 병렬, explore_lite, 200초)에서
  재검증. **결과: 4/4(100%) 전부 조기종료 재현** — 순차에서 해소됐던 게 병렬에선
  전혀 안 통함. 로그로 원인 확인:
  ```
  t=945s Managed nodes are active
  t=950s Begin navigating (첫 목표, +5s)
  t=953s Transform data too old (sim 11.1s vs 10.7s, +3s) -> Goal failed
  t=954s All frontiers traversed, stopping.
  ```
  순차 스모크 테스트와 타이밍 구조가 거의 동일 — 즉 **"Managed nodes are
  active"는 "SLAM이 실제로 안정된 TF를 발행 중"을 보장하지 않는다.** Nav2
  lifecycle 전이(controller/planner/behavior/bt_navigator activate)와 SLAM의
  실제 TF 안정화는 별개 신호이며, 병렬 조건의 CPU 경쟁 때문에 activation 이후에도
  SLAM이 계속 지연될 수 있음. **(a)만으로는 부족 — (b) 착수 필요.**
