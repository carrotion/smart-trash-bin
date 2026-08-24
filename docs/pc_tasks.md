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

**진행 상황 (2026-08-24, Remi)**:
- **지자기 방위 이중 부호반전 버그 발견/수정**: 사용자가 실제로 동쪽(≈90°)을
  보는데 대시보드가 282°를 표시해서 발견. 2026-08-21 밤 Davi 커밋 2건
  (`01f5a38` 지자기 X축 원시값 부호반전, `dfb71af` 대시보드 atan2 부호 되돌림)이
  서로 다른 근거로 부호를 반대로 건드려 상쇄되지 않고 겹쳐서 정확히
  180°(+잔차) 틀어진 방위가 나오고 있었음. `01f5a38`의 근거("차체 전후 180°
  반전")는 문서 어디에도 없는 사건 — 08-17엔 라이다 상판→하판 재장착(라이다
  자체)과 IMU 위치 이동(`b65fef5`, 회전 아닌 이동)만 있었음. 둘 다 2026-08-17에
  폰으로 검증했던(-atan2, 8° 이내 일치) 원래 공식으로 되돌림 — 라이브 재검증
  완료(사용자가 실제 방향과 일치 확인). 새 하드아이언 오프셋(ox=-45.75/
  oy=+22.95, IMU 재장착에 따른 재보정)은 유지.
- Davi IMU 커밋 나머지 2건(`4fe270a`, `5ed0bd5`)도 팩트체크 — 문제없음(상세는
  `CLAUDE.md` "다음 세션은 여기서 시작" 2번 참고).
- **웨이포인트 대조 첫 실측 — 거리 스케일 오차 발견(가설, n=2)**: `our_home`
  지도로 계산한 목표(거실 시작점→옆방 문, 5.06m)로 `tools/waypoint_drift.py`
  (신규, EKF 실시간 거리 도달 시 자동 정지)를 왕복 2회 실행, 매번 사용자가
  줄자로 실제 이동거리 실측 대조:

  | 구간 | 실측(줄자) | EKF 보고 | 휠 보고 | EKF 오차 | 휠 오차 |
  |---|---|---|---|---|---|
  | 전진 | 5.160m | 5.06m | 5.04m | +1.98% | +2.38% |
  | 복귀 | 5.165m | 5.09m | 5.06m | +1.47% | +2.08% |

  둘 다 같은 방향(과소 보고), 비슷한 크기 — 노이즈보다 **바퀴 지름 상수
  자체가 작게 잡혀있을 가능성**. 현재 `diff_drive_node.py`의
  `wheel_radius=0.0335m`(지름 67mm, "실측"이라 주석돼있음)를 이 오차로
  보정하면 지름 **~68.5mm**. ⚠️ n=2로는 확정 부족(이 프로젝트에서
  같은 조건 2회 측정이 11% 차이난 전례 있음, PWM 캘리브레이션 참고) —
  사용자 판단으로 지금은 가설만 기록, 추가 반복은 보류하고 다음 항목으로
  이동. 원본 로그: `docs/calibration/waypoint_A_*.json`.
- **웨이포인트 오차 계산 도구 확장/정합**: Resie가 만든
  `tools/waypoint_error.py`(오차 3분해: 위치/거리/방위, 자체검증 15/15
  통과 확인함, [Issue #3](https://github.com/carrotion/smart-trash-bin/issues/3))
  와 로그 스키마를 맞추기 위해 `waypoint_drift.py`의 출력 키를
  `final_raw/final_ekf` → `end_raw/end_ekf`로 통일(기존 로그 3개도
  같이 패치).
- **✅ 회전 중 한쪽 바퀴가 몇 초씩 안 도는 현상 — 원인 확정·수정 완료
  (2026-08-22부터 미확정이었던 Layer 0 항목)**: `turn_debug` 로그로 재현
  시험(방향반전 6회 + 단일방향 지속회전)한 결과, 진짜 원인은 방향전환도
  자이로 노이즈도 아니었음 — **`motor_pwm_offset`(M3 직진 드래그 보정용
  상수, 08-20 추가)이 회전 중에도 그대로 적용돼서 좌우 비대칭을 만들고
  있었다.** 폐루프가 계산한 대칭 명령(좌-33/우+33)이 실제로는
  M1=-108/M2=-33/**M3=+273**/M4=+33로 나가서 M3만 8배 가까이 세게
  밀림 — 이 오프셋은 회전 게인(08-19 튜닝)보다 하루 늦게 추가돼서
  한 번도 같이 검증된 적이 없었다. 08-19의 "스크럽 마찰+폐루프 보정이
  피드포워드를 깎는다" 가설은 틀렸음이 확인됨.

  순수 회전(v≈0)에서 오프셋을 끄도록 `_apply_pwm`에 `apply_offset` 인자
  추가(정지 시 오프셋 미적용이라는 기존 선례의 논리 연장):

  | 시험 | 수정 전 | 수정 후 |
  |---|---|---|
  | 단일방향 지속회전(0.6 rad/s, 3초) 표준편차 | 0.7~1.0(부호까지 뒤집힘) | **0.164** |
  | 단일방향 지속회전 평균 | 불안정 | **0.603 rad/s**(명령과 거의 정확히 일치) |
  | 방향반전 6회 데드존 경고 건수 | 17건 | **3건**(82%↓) |

  남은 3건은 방향전환 순간의 짧은 스파이크로 보임 — 원래 08-22 가설과
  같은 계열의 훨씬 작은 잔여 문제, 우선순위 낮음. 참고로 첫 시도(EKF
  쪽에 썼던 150ms 안정화 창을 회전 폐루프에도 적용)는 효과 없어서
  원복함 — 실제 원인이 자이로 노이즈가 아니라 위 오프셋 문제였기 때문.

- **⚠️ Layer 1 첫 시도 — 실물 SLAM 매핑, 지도 품질 나쁨(원인 미확정)**:
  오늘 고친 것들(요 드리프트/지자기/회전 발진)을 전부 반영한 상태로
  거실→A방→거실 복귀 루프를 직접 조종으로 매핑. 결과: **지도에 A방으로
  보이는 부분이 회전된 채 중복돼서 그려짐**(2026-08-23 이전 `home_real`
  때 겪었던 "회전 드리프트로 벽 중복" 패턴과 시각적으로 유사). 사용자가
  시작 지점과 "비슷하지만 정확히는 다른" 곳에 도착했다고 보고, EKF 최종
  방위는 시작 대비 +138.6°(경로상 여러 번 방향전환이 있어 그 자체는
  비정상 아님).

  **원인 미확정** — 이번 주행은 자이로/EKF 연속 로깅 없이 매핑만 돌려서
  최종 지도만 있고 과정(정확히 어느 구간에서 오차가 붙었는지)을 못 봄.
  오늘 고친 회전 발진 버그가 82%만 개선(완전 해소 아님)이라 잔여분이
  A방 안 방향전환 때 드러났을 가능성이 유력한 가설이지만, 확정 아님 —
  그럴듯한 이야기로 메우지 말 것.

  **다음 세션 시작점**: 같은 루프를 `round_trip_gyro_diag.py` 스타일로
  자이로/EKF/odom 연속 로깅하면서 재현 — 정확히 어느 지점(A방 안 방향전환
  추정)에서 오차가 붙는지 시계열로 짚어낼 것. 지도 원본:
  `docs/calibration/maps/layer1_test1_20260824.pgm/yaml`(+`_view.png`
  3배 확대본).

- **🔴 [최우선] 휠 오도메트리 실주행 중 조용한 "동결" — 원인 미확정, 종합
  리뷰 + 배제 시험 완료 (2026-08-24 밤)**: 위 웨이포인트 시험용으로 만든
  `tools/loop_gyro_diag.py`(연속 로깅)로 처음 몇 분짜리 실주행을 들여다보다
  발견. `/odom`이 수십 초씩 완전히 멈추는 동안 EKF는 계속 갱신됨(`ekf.yaml`이
  x,y/vx는 휠 의존이라 동결 구간의 실이동을 EKF도 놓침 — 큰 위치 오차의
  유력한 메커니즘). motor_dashboard 포트경합으로 오판했다가 코드 확인 후
  정정(`/cmd_vel` 발행 방식으로 이미 리팩터링돼 있어 무관, CLAUDE.md/
  diff_drive_node.py 낡은 경고 문구도 같이 정정함). CPU 부하설은
  `power_trace.csv` 대조로 반증. Phase A(정지 10분)/B(도크 무부하 모터
  구동 5분) 둘 다 재현 안 됨 — **실제 바닥 하중 걸린 주행에서만 재현**돼서
  USB 케이블/커넥터 접촉불량 쪽으로 후보 좁혀짐. `_read_encoder()`에 실패
  진단 로깅(시작/heartbeat/복구+원시바이트) 추가해둠.
  **다음 세션 시작점(Phase E, 접촉불량 확인)**: `docs/odometry_comprehensive_review_20260824.md`
  "Phase E" 절 참고 — 케이블 손으로 스트레스 주기(E-1) → dmesg 동시 캡처(E-2)
  → 케이블 고정 A/B(E-3) → 재체결(E-4) 순서.
- Davi의 2026-08-17 제안 재검토(`rf2o_laser_odometry`, ZUPT, 슬립 감지 공분산
  등) + 외부 리서치(광학플로우 PMW3901 등) 포함 종합 정리:
  `docs/odometry_comprehensive_review_20260824.md`. 요약: 센서 추가보다
  위 오도메트리 동결부터 잡는 게 먼저, 그다음 `rf2o_laser_odometry`(비용 0원,
  이미 추천됐던 것) 시도가 다음 순서.

## Track A′ — 탐색 전략 (Resie 담당, 시뮬레이션, Layer 2 전용임을 명시)

⚠️ 이 트랙의 결과는 **"위치를 완벽히 안다면 어디로 갈지"**에 대한 답이지,
"실물에서 바로 쓸 수 있는 최종 설정"이 아님 — Track 0이 어느 정도 안정돼야
여기서 나온 전략을 실물에 반영하는 게 의미 있다. 그래도 시뮬레이션은 실물 위험이
없어 계속 병렬로 진행 가치 있음.

**⚠️ 2026-08-24 갱신 — 아래 항목들의 최신 상태는 이 파일 뒷부분의 전용 섹션
참고, 여기는 요약만:**

- **Phase 0 (완료)**: 시뮬레이션 인프라, `our_home` 월드, 배치 스윕 도구.
- **Phase 1 (완료)**: explore_lite vs frontier_exploration_ros2 비교 + Optuna
  파라미터 탐색. → "🏁 explore_lite vs frontier_exploration_ros2 — 최종 결론"
  섹션. **frontier_exploration_ros2 권장**으로 확정(안정성 압도적 우위).
- **Phase 1.5 (완료)**: 조기종료 버그 원인 규명 + 완화책 (a)(b) 적용 +
  근본원인(`getRobotPose()` empty_pose) 확정·수정 — 단 조기종료율 자체는
  크게 개선 안 됨(58%→62.5%, 오차범위 내). → "🔬 explore_lite 조기종료
  근본원인 조사" 섹션.
- **Phase 2 (완료)**: frontier_exploration_ros2 최적값을 6개 시작위치에서
  검증, 전부 정상 완료. → "🏠 Phase 2" 섹션.
- 상세 리뷰: [Issue #1](https://github.com/carrotion/smart-trash-bin/issues/1)
  (전체 조사 과정이 댓글로 시간순 기록됨).

## 🏁 explore_lite vs frontier_exploration_ros2 — 최종 결론 (2026-08-24 아침, Resie)

killpg 버그 2건(아래 상세)을 완전히 수정하고 좀비 없는 깨끗한 상태에서
explore_lite Optuna 24트라이얼을 재검증했다(`study_explore_lite_v6_final.db`,
프로세스 완전 클린 확인됨). **좀비/자원경합은 이제 원인이 아니지만, 여전히
58.3%(14/24)가 조기종료(coverage 7.2%)됐다.** frontier_exploration_ros2는
이미 확정된 24/24(100%) 정상 완료.

| | explore_lite (v6, 좀비버그 수정 후) | frontier_exploration_ros2 |
|---|---|---|
| 트라이얼 수 | 24 | 24 |
| 조기종료 비율 | **58.3%(14개)** | **0%** |
| 최고 coverage | 41.5% | 37.4% |
| 정상 트라이얼 평균 | ~34% (10개 중) | ~29.7% (24개 전체) |
| tf_stale 범위 | 0~7 | **228~332** (훨씬 많음!) |

**결정적 관찰 — 조기종료가 파라미터 문제가 아니라 비결정적 실행 문제임을
입증**: trial14(41.5%, 성공)와 trial15/16(7.2%, 실패)의 파라미터가 거의
동일하다(`potential_scale≈0.52~0.59, gain_scale≈3.65~3.91,
min_frontier_size≈0.13~0.17, min_frontier_distance≈0.68~0.70`) — 같은
파라미터 조합이 실행마다 성공/실패가 갈린다. 게다가 trial16은
`tf_stale=0`(TF 지연이 전혀 없었음)인데도 실패했다 — **TF 지연이 원인이
아닌 실패 경로가 아직 남아있다는 뜻.** (b) 패치(재시도 로직)가 있는데도
결국 실패로 귀결된다는 건, 재시도가 발동은 해도 매번 다시 실패해서
결국 blacklist되는 경우가 있다는 것.

부가 관찰(표본 작아 확정 아님): 도메인별로 나눠보면 `ROS_DOMAIN_ID=101`에서
6/6(100%) 전부 실패, `103`에서는 1/6만 실패 — 도메인 자체의 문제라기보다는
Optuna의 트라이얼-도메인 배정이 우연히 편향됐을 가능성이 높지만(확률
0.5^6≈1.5%로 낮긴 함), 추가 조사 가치는 있음.

**결론 및 권고**: coverage 최댓값 자체는 explore_lite가 근소 우위(41.5%
vs 37.4%)지만, **안정성 측면에서 frontier_exploration_ros2가 압도적으로
우위**다 — tf_stale이 explore_lite의 30~40배(228~332 vs 0~7)인데도
단 한 번도 조기종료되지 않았다. 즉 frontier_exploration_ros2의 재시도/
블랙리스트 로직이 explore_lite보다 훨씬 견고하다. **실물 로봇에 적용할
경우 58%의 조기종료 확률은 받아들이기 어려운 리스크 — 현재 상태로는
frontier_exploration_ros2를 권장.** explore_lite를 계속 쓰려면 (b) 패치가
왜 일부 케이스에서 재시도해도 결국 실패하는지 추가 규명이 필요(다음
세션 후보 작업).

## 🏠 Phase 2 — frontier_exploration_ros2 최적값 다중 시작위치 검증 (2026-08-24, Resie)

`our_home.png` 비트맵에서 0.5m 반경 완전 개방인 지점을 자동 탐색해(원형
마스크 브루트포스 스캔, `scipy` 미설치라 numpy만으로 구현) 집 안 서로 다른
5개 구역(거실 다른 지점, 침실1, 침실2, 왼쪽 복도, 오른쪽 아래 공간=주방
추정)에서 시작점을 하나씩 골랐다. Stage 좌표계 매핑(비트맵 row 0 = world
+y)은 원래 시작점(-2.7, 3.43)과 픽셀 좌표가 정확히 일치하는 것으로 검증.
world 파일 변형(`our_home_start_*.world`)은 `robot/src/stage_ros2/world/`에
저장(서브모듈이라 gitignore됨 — 재현하려면 `tools/phase2_multi_start.py`의
`STARTS`/좌표 계산 로직 참고). 원래 Optuna 탐색 기준점(거실)도 `baseline_living`
으로 포함해 총 6개 시작위치, N=4 병렬로 `tools/phase2_multi_start.py` 신규
작성해 실행.

| 시작위치 | 구역 | coverage% | tf_stale | progress_fail | collisions |
|---|---|---|---|---|---|
| baseline_living | 거실(원래 Optuna 기준점) | 21.4 | 145 | 1 | 0 |
| living2 | 거실 다른 지점 | 31.5 | 157 | 1 | 0 |
| bed1 | 침실1 | 22.1 | 140 | 4 | 0 |
| bed2 | 침실2 | 44.2 | 135 | 4 | 0 |
| hallway | 왼쪽 복도/안방 | 35.5 | 138 | 2 | 0 |
| kitchen | 오른쪽 아래(주방 추정) | 71.9 | 133 | 1 | 0 |

**전부 정상 완료(조기종료 0/6), 충돌 0건.** coverage는 21.4~71.9%로 시작
위치에 따라 편차가 크지만(3배 이상) — 이건 위치별로 150초 안에 실제 도달
가능한 개방 공간의 크기/형태가 다르기 때문으로 보이며, 안정성과는 별개
지표다. tf_stale은 이번에도 130~157회로 explore_lite(0~7)보다 훨씬 많은데도
단 한 번도 조기종료로 이어지지 않았다 — 24트라이얼 Optuna 탐색에서 확인된
"frontier가 TF 지연에 훨씬 강건하다"는 결론이 시작 위치와 무관하게 일반화됨을
확인. **frontier_exploration_ros2 권장 결론이 특정 시작위치(거실)에 우연히
맞춰진 결과가 아님이 확정됨.**

⚠️ coverage 편차(21.4~71.9%)가 위치별 공간 크기 차이 때문인지, 아니면 다른
요인(예: 시작 지점이 벽/문에 얼마나 가까운지)이 섞여 있는지는 미분석 —
필요하면 각 위치의 도달가능 면적을 정규화해서 재비교할 가치 있음(우선순위
낮음, Phase 2의 핵심 질문인 "안정성이 위치와 무관한가"는 이미 확정 답변됨).

## 🔬 explore_lite 조기종료 근본원인 조사 (2026-08-24 낮, Resie) — 버그 확정, 완전 해결은 아님

run6 trial15/16(파라미터가 거의 같은데 성공/실패가 갈림, tf_stale=0인데도
실패)을 직접 로그 분석했다.

**확정된 버그**: `costmap_client_.getRobotPose()`(`costmap_client.cpp:222`)가
TF lookup 실패(LookupException/ExtrapolationException 등) 시 예외를 삼키고
**원점(0,0,0)의 기본생성 Pose를 반환**한다. `makePlan()`은 이 반환값이
성공인지 실패인지 구분할 방법이 없어서, 실패 시에도 "로봇이 원점에 있다"고
믿고 그 위치 기준으로 프론티어를 재탐색한다 — 로봇이 이미 이동한 상태라면
완전히 엉뚱한(대개 이미 blacklist된) 프론티어를 골라 "All frontiers
traversed"로 오판한다. Phase 1.5 (b) 패치가 `retry_timer_`로 makePlan()을
재호출할 때, 그 시점에 TF가 아직 stale이면 이 문제가 발동한다.

**수정 시행착오 세 번** (전부 실측으로 검증, `robot/src/m-explore-ros2`
서브모듈 `explore.cpp` `makePlan()` 시작부):
1. `pose.orientation.x==y==z==w==0`으로 판별 → **완전히 무효**.
   `geometry_msgs/msg/Quaternion.msg`의 IDL 기본값이 `w=1`(identity)이라
   이 조건은 항상 거짓 — 방어코드가 한 번도 안 발동함을 로그로 확인, 12트라이얼
   재검증에서 조기종료율이 오히려 58%→83%로 나빠짐(수정 자체는 무효했고
   표본 편향이었을 뿐).
2. `pose.position.x==y==z==0`으로 판별 → **정반대로 항상 참**. 로봇이
   Stage 시뮬레이션에서 정확히 원점(0,0,0)에서 스폰되므로, 탐색 시작 직후의
   **정상적인** makePlan() 호출도 진짜로 position이 (0,0,0)이라 전부 오탐 —
   24/24(100%) 조기종료로 완전히 악화됨. **반환값만으로는 성공/실패를 구분할
   방법이 없다**는 걸 이 두 실패로 확정.
3. **최종(채택)**: `costmap_client_`와 동일한 조건(`transform_tolerance`
   파라미터, 호출 시점 `now()`)으로 `Explore` 자신의 `tf_buffer_`에 직접
   `lookupTransform()`을 시도해 **예외 발생 여부로 정확히 판별**, 실패 시
   해당 사이클을 skip. `transform_tolerance`는 `costmap_client_` 생성자가
   이미 이 노드에 declare해뒀으므로 get만 함(중복 declare 시 예외 발생 주의).

**검증 결과 (24트라이얼, N=4, `study_explore_lite_v10_final.db`)**: 조기종료
15/24(62.5%) — 수정 전(run6, 14/24=58.3%)과 **실질적으로 동일**. 즉 이
버그는 실재했고 코드 정확성 관점에서 고칠 가치가 있었지만(안전성 문제이기도
함 — 잘못된 위치 기준 탐색), **조기종료의 주된 원인은 아니었다.**

**결론**: 조기종료는 단일 원인이 아니라 `reachedGoal`의 ABORTED→blacklist
분기가 `makePlan()`을 즉시 재호출하지 않고 `exploring_timer_`(주기
`1000/planner_frequency_`≈6.67초)까지 기다려야 하는 설계, `retry_timer_`가
wall-clock 기반이라 sim_time과 시간축이 다른 문제, N=4 병렬의 심한 TF 지연
빈도(재시도 2회로도 부족한 경우가 실제 다수) 등이 복합적으로 작용하는
것으로 보인다. 이는 어제 결론(frontier_exploration_ros2가 TF 지연에 30~40배
더 강건하고 한 번도 조기종료 안 함)을 다시 한번 강하게 뒷받침한다 — **패치로
땜질하기보다 frontier_exploration_ros2를 쓰는 게 훨씬 실용적**이라는 권고를
유지/강화한다. `getRobotPose()` empty_pose 수정 자체는 정당한 버그 수정이라
패치에 유지하되, 여기서 추가 investigation은 중단한다(수확체감 — 이미
명확한 대안이 있는 상태에서 구조적 재설계급 작업에 더 투자할 실익이 낮음).

## 🐛 killpg 버그 2건 완전 수정 기록 (2026-08-23 밤~2026-08-24 새벽, Resie)

**버그 1 — `os.getpgid(proc.pid)`가 reap 이후 실패**: `proc.wait()`가
정상 반환(exit 0)되면 그 시점에 파이썬이 이미 `proc.pid`를 reap한다.
그 뒤 `os.getpgid(proc.pid)`를 부르면 ProcessLookupError가 나고, 바로
아래 `except: pass`가 조용히 삼켜서 killpg가 사실상 실행되지 않았다 —
grandchildren(ros2 launch가 띄운 노드 전부)이 좀비/고아로 남음. 실제로
도메인 100~103 각각에 11시간 넘게 눌러앉은 좀비 트리 4개가 발견됨.
`preexec_fn=os.setsid`로 시작했으므로 pgid는 항상 `proc.pid`와 동일 —
getpgid 조회 자체를 없애고 `proc.pid`를 pgid로 직접 사용하도록 수정
(커밋 `c0415f2`).

**버그 2 — killpg가 job-control로 갈라진 하위 프로세스 그룹을 못 잡음**:
버그 1을 고친 뒤에도 24트라이얼 중 1개가 17분 넘게 안 죽었다. 원인은
`inner_cmd`(`timeout ... ros2 launch ...`)가 bash 안에서 `&`로 백그라운드
실행되면서 setsid로 만든 최상위 그룹과 **분리된 새 프로세스 그룹**을
얻는 경우가 실측 확인됨(해당 트라이얼의 `timeout` 프로세스가 자기 자신을
pgid로 갖고 있었음). `/proc`의 ppid 체인은 프로세스 그룹과 무관하게
항상 유효하므로, `kill_process_tree()`로 재귀 탐색 + 개별 SIGKILL로
교체(`tools/killproc.py`/`mapping_cleanup.py`와 같은 원리, 커밋 `549bf3c`).
throwaway sleep 트리로 유닛 테스트 후 재검증 — 24트라이얼 전부 좀비 없이
완전히 클린 확인됨.

**교훈**: 두 버그 모두 "정상 종료처럼 보이는 케이스"에서만 발동해서
오랫동안 안 잡혔다. `docs/pc_tasks.md` "공유 자원 규칙" 2번(자기 자신의
이전 배치 좀비도 확인)을 실제로 지키지 않은 채 장시간 여러 차례 재시도를
반복한 게 사고를 키웠다 — 다음에 유사한 병렬 배치 스크립트를 쓸 때는
매 실행 전후로 `ps -eo pid,ppid,pgid,etimes,cmd | grep -E 'ros2
launch|stage_ros2|Xvfb'`가 완전히 비어있는지 반드시 확인할 것.

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
5. ~~Phase 1.5 완화책 (b)~~ **완료, N=4 병렬에서도 해소 확인** — `max_frontier_retries`
   파라미터(기본 2) 추가, `reachedGoal`의 ABORTED 케이스가 실패 횟수를 세서
   문턱 미만이면 blacklist 대신 재시도. 패치/README: `robot/patches/`.
   N=4 병렬 재검증: **조기종료 0/4**(적용 전 4/4), goal_switches 14~23회,
   재시도 로직 런당 10~13회 실제 발동. 상세는 아래 PC 보고.
6. ~~explore_lite Optuna 재실행 — (b) 적용 상태로~~ **완료, 하지만 새 버그 2건
   발견/수정 후에도 58.3% 조기종료 잔존**: 재실행 과정에서 optuna_search.py의
   killpg 자체가 이중으로 고장나 있었음을 발견(getpgid reap 버그, job-control
   그룹 분리 버그 — 둘 다 커밋 완료, 상세는 위 "killpg 버그 2건" 섹션). 완전히
   좀비 없는 상태로 재검증해도 58.3%(14/24)가 여전히 조기종료 — 좀비/자원경합이
   원인이 아니라 explore_lite 자체의 비결정적 실패 경로가 남아있음이 확정됨.
   상세: 위 "🏁 최종 결론" 섹션.
7. ~~frontier_exploration_ros2 Optuna 탐색~~ **완료** — 24/24(100%) 정상 완료,
   최적값 37.4%, `study_frontier.db`.
8. ~~explore_lite의 잔존 조기종료 원인 규명~~ **완료(부분 해결) — 진짜
   버그 하나는 고쳤지만 조기종료율은 그대로**. 상세는 아래 "🔬 explore_lite
   조기종료 근본원인 조사" 섹션.
9. ~~Phase 2~~ **완료** — frontier_exploration_ros2 최적값을 6개 시작 위치에서
   검증, **전부 조기종료 없음(0/6), 충돌 0건**. 상세는 아래 "🏠 Phase 2" 섹션.

**Track A′는 이걸로 완료 처리 (2026-08-24, Remi 리뷰).** 새 작업:

10. ~~웨이포인트 오차 계산/비교 도구~~ **완료** (2026-08-24, Resie) —
    `tools/waypoint_error.py`(핵심 계산: 위치/거리/방위 오차 3종,
    `--self-test`로 합성 데이터 15케이스 전부 통과) +
    `tools/waypoint_capture.py`(실물 캡처, 로봇 몰지 않고 시작/도착 시점
    Enter로 pose 스냅샷). 상세: [Issue #3](https://github.com/carrotion/smart-trash-bin/issues/3)
    코멘트.

**보고 방법**: 이 파일 "## PC 보고" 섹션 + [Issue #3](https://github.com/carrotion/smart-trash-bin/issues/3)
코멘트로 (Issue #1은 Track A′ 완료로 종결).

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

- **Phase 1.5 완화책 (b) 적용 + N=4 병렬 검증 완료** (2026-08-23 밤, Resie):
  m-explore-ros2에 패치 추가(`robot/patches/`) — 좌표별 실패 횟수를 세서
  `max_frontier_retries`(기본 2) 미만이면 blacklist 대신 makePlan() 재호출로
  재시도. (a)와 함께 적용한 채로 같은 N=4 병렬 조건(도메인 100~103, 200초)
  재검증:

  | run | goal_switches | tf_stale | 재시도 발동 | 결과 |
  |---|---|---|---|---|
  | 0 | 19 | 10 | 12 | 정상 진행(타임아웃) |
  | 1 | 14 | 9 | 10 | 정상 진행(타임아웃) |
  | 2 | 17 | 9 | 12 | 정상 완료(All frontiers traversed) |
  | 3 | 23 | 6 | 13 | 정상 진행(타임아웃) |

  **조기종료(goal_switches=1) 0/4 — (a) 단독 4/4 재현에서 완전히 해소됨.**
  TF 스톨 자체는 여전히 자주 발생하지만(런당 6~10회), 매번 재시도 로그로
  넘어가 탐색은 계속됨. patches/README.md에 배경/검증 기록 완료. 다음:
  frontier_exploration_ros2 Optuna 탐색 착수.

- **🔴 explore_lite Optuna 24트라이얼 재탐색에서 79%(19/24) 조기종료 재현 —
  원인은 (a)/(b)가 아니라 도메인 슬롯 풀 자체의 좀비 누수였다** (2026-08-23 밤
  ~2026-08-24 새벽, Resie): (b) 적용 후 `retry_delay_ms`(즉시 재시도 →
  1500ms 지연 재시도로 개정, `same_goal` 최적화를 우회하려고 `prev_goal_`을
  NaN으로 무효화하는 처리도 함께 넣음)까지 적용했는데도 24트라이얼 재검증에서
  87.5%(run4 이전 배치) → 79%(run4)로 조기종료가 거의 안 줄었다. 일부는
  `tf_stale=0`인데도 실패해서 TF 지연과 무관한 원인을 의심하고 로그를 보니
  `slam_toolbox` lifecycle 전이 실패(`Failed to make transition
  'TRANSITION_CONFIGURE'`, `No transition matching 3 found for current state
  active`)와 `Detected jump back in time. Clearing TF buffer.`가 찍혀 있었음.

  프로세스 트리를 전수조사한 결과 **도메인 100/101/102/103 각각에 좀비 트리가
  하나씩, 11시간 넘게(etimes 39500~41700초) 눌러앉아 있었다** — `[stage_ros2]`,
  `[planner_server]`, `[behavior_server]`, `[bt_navigator]`는 `<defunct>`
  (진짜 죽은 좀비)였지만, 그 부모인 `timeout`/`ros2 launch` 파이썬 프로세스
  자체는 **살아있는 상태**였다. 즉 이번 run4의 24트라이얼 전부가 4개 도메인
  전부 이미 오염된 채로 실행됐다 — (a)/(b) 로직이 잘못된 게 아니라애초에
  **깨끗한 도메인에서 실행된 트라이얼이 하나도 없었다.**

  **좀비의 근본원인 — `optuna_search.py`의 `os.killpg(os.getpgid(proc.pid),
  ...)` 자체가 버그였다.** `proc.wait()`가 정상 반환되면(타임아웃이 아니라
  exit 0으로 끝나면) 그 시점에 파이썬이 이미 `proc.pid`를 reap한 상태다.
  그 뒤 `os.getpgid(proc.pid)`를 부르면 이미 사라진 PID라 `ProcessLookupError`
  (ESRCH)가 나고, 바로 아래 `except: pass`가 이를 조용히 삼켜서 **killpg가
  사실상 한 번도 실행되지 않았다** — grandchildren(ros2 launch가 띄운 노드
  전부)이 그대로 좀비/고아로 남았다. 타임아웃 경로에서만 pid가 아직 살아있어
  우연히 성공했던 것. `preexec_fn=os.setsid`로 시작했으므로 그룹 리더의
  pgid는 항상 `proc.pid`와 동일(reap 여부와 무관하게 이미 알고 있는 값) —
  `getpgid` 조회 자체를 없애고 `proc.pid`를 pgid로 직접 쓰도록 수정
  (`tools/optuna_search.py` `run_trial()`).

  좀비 4개 그룹을 `kill -9 -<pgid>`로 전부 정리, `mapping_cleanup.clean_orphan_shm()`으로
  DDS shm 잔재 정리, 프로세스 완전히 0개 확인 후 수정된 코드로
  `study_explore_lite_v5_fixed.db`에 24트라이얼 재검증 중(결과는 아래 이어서
  기록).

  ⚠️ **교훈**: (a)/(b) 자체의 재검증(N=4, 4회)은 전부 순수 신규 프로세스로
  돌렸기 때문에 그 결과(조기종료 0/4)는 여전히 유효하다. 문제는 그 이후
  **장시간 Optuna 배치(24트라이얼, 여러 차례 재시도)를 거치며 좀비가 누적됐고,
  그 누적을 매 배치 시작 전에 확인하지 않은 것**이다 — `docs/pc_tasks.md`
  "공유 자원 규칙"에 있던 "자기 자신의 이전 배치 좀비도 확인" 원칙을 정확히
  어긴 사례. `optuna_search.py`를 재실행하기 전에는 항상
  `ps -eo pid,ppid,pgid,etimes,cmd | grep -E 'ros2 launch|stage_ros2|Xvfb'`로
  0개인지 확인할 것 — 프로세스가 남아있으면 `kill -9 -<pgid>`(음수 pgid,
  즉 killpg)로 그룹째 정리. (아래 "Phase 1.5 완화책 (b) 적용" 항목에 이어짐 —
  같은 내용이 중복 기록됐던 것을 여기서 정리함, 2026-08-24)
