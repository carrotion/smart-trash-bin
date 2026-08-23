# explore_lite vs frontier_exploration_ros2 — 시뮬 비교 (2026-08-23)

## 2차: `our_home.world` (사용자가 실제 도면 기반으로 직접 트레이싱, 2026-08-23 13:08 추가)

`home_real.world`는 SLAM 스캔 스냅샷을 그대로 옮긴 거라 회전 오차로 생긴 유령
벽이 섞여 있었다. 사용자가 이걸 폐기하고 **NAVER 부동산 평면도(34평/전용
84㎡)를 GIMP로 직접 트레이싱**해서 오차 없는 벽 구조를 새로 그렸다
(14.9×9.85m, 거실 시작점 (-2.7, 3.43)). 문 3개(안방/침실1/침실2)만 벽에 틈으로
존재하고 진입 금지 구역(욕실 2곳/발코니/현관/다용도실/팬트리/대피공간/주방
조리대)은 경계 밖으로 파서 그림 — `home_real`보다 훨씬 현실적이고 깨끗한
테스트 맵이다. 조건은 1차와 동일(각 400초 캡, ROS_DOMAIN_ID=43, 나머지 설정 동일).

### 결과 요약

| 지표 | explore_lite | frontier_exploration_ros2 |
|---|---|---|
| 목표 전환 횟수 | 16 | 15 |
| "Failed to make progress" 횟수 | 3 | **0** |
| 충돌 수 (Collision Ahead) | 0 | 0 |
| Goal failed | 7 | 14 |
| Goal preemption(자연 재선택, 실패 아님) | 9 | 0 |
| 완료 시간 | **188초에 "All frontiers traversed" — 이번엔 실제로 방 전체를 훑고 정상 종료** | 400초 캡까지 계속 재시도, 미완료 |

### 핵심 발견

**이번 맵에서는 두 설정 다 실제로 움직이고 탐색했다.** `home_real`에서 봤던
"3번 실패하고 바로 포기" 같은 조기 오판은 없었다 — explore_lite는 좌표가
(0,0)→(1.31,-1.55)→(2.03,-4.48)까지 방 전체로 넓게 퍼지며 188초간 탐색한 뒤
정상적으로 "완료"를 선언했다. frontier_exploration_ros2도 (0,0)→(4.34,-5.96)까지
꾸준히 이동하며 400초 내내 새 프론티어를 계속 찾아 시도했다(완료 신호는
안 뜸 — 아직 갈 곳이 남아있다고 판단한 것으로 보임).

**다만 두 설정 모두 여전히 "Unable to transform robot pose into global plan's
frame"(`tf_help: Transform data too old`) 패턴이 반복적으로 나타난다** —
특히 frontier_exploration_ros2 쪽은 **거의 매 목표마다** (14/15회) 이 에러로
`Goal failed`가 찍혔는데, 흥미롭게도 `Failed to make progress`(진행 정체
판정)는 **한 번도 안 떴다** — 즉 로봇이 목표 근처까지 실제로 이동은 하는데
막판에 TF 타임아웃으로 그 목표만 "실패" 처리되고 다음 목표로 넘어가는
패턴으로 보인다(좌표가 계속 전진하는 게 그 증거). explore_lite도 정도는
약하지만 같은 에러가 섞여 있다(Failed to make progress 3회).

**타이밍이 매우 규칙적이다** — frontier 런에서 이 에러가 발생한 시각이
t=25s, 55s, 84s, 114s, 144s로 약 28~30초 간격으로 거의 정확히 반복된다.
무작위 시뮬 버벅임이 아니라 **결정론적인 TF 갱신 주기 문제**로 보인다.
`nav2_params.yaml`의 `transform_tolerance: 0.3`(0.3초)에 비해 실제 지연이
그보다 살짝 크게(로그상 0.4~0.8초) 나오는 패턴이 반복됨 — 이 여유값이
지금 시뮬 환경(Xvfb+WSL)의 실제 map→odom TF 갱신 주기를 못 따라가는 것일
수 있다. **1차 `home_real` 비교의 3번째 핵심 발견과 정확히 같은 현상이
다른 맵에서도 재현됐다** — 두 프론티어 전략과 무관한 공통 원인일 가능성이
더 굳어졌다. 다음 우선순위는 이 TF 타이밍 쪽 조사로 보인다(단, PC 시뮬
고유의 실시간성 문제일 수도 있어 Pi에서 교차검증 필요 — 아래 1차 결과의
참고 사항과 동일).

### 원본 로그
- `run2_explore_lite.log` — 473줄, t0~t_end 378초
- `run2_frontier.log` — 680줄, t0~t_end 378초

---

## 1차: `home_real.world` (SLAM 스캔 스냅샷 변환, 폐기된 맵 — 아래는 참고용 기록)

## 테스트 조건
- 환경: PC(WSL Ubuntu 24.04, ROS2 Jazzy) — Pi가 아님. `ROS_DOMAIN_ID=43`(Pi의 42와 분리)
- 월드: `home_real.world` (2026-08-22 23:30 실물 세션 맵 스냅샷 변환, 15.15×15.50m)
- 시작 좌표: (-0.488, 1.02) — 월드 파일에 박제된, 실제로 로봇이 갇혔던 지점
- Nav2/SLAM 설정: `nav2_params.yaml`, `mapper_params_online_async.yaml` 실물과 동일 파일 그대로 재사용
- explore_lite: 현재 프로덕션 튜닝값(potential_scale/gain_scale 2.0/2.0, min_frontier_size 0.8, min_frontier_distance 0.6) 그대로
- frontier_exploration_ros2: 패키지 기본값(MRTSP-dp 등) 그대로, robot_base_frame=base_link + 거리 파라미터 2개만 로봇 반경 기준(0.35m)으로 조정
- 각 1회, 400초 캡(타임아웃 도달 시 미완료로 기록)

## 결과 요약

| 지표 | explore_lite | frontier_exploration_ros2 |
|---|---|---|
| 목표 전환 횟수 | **3** | **12** |
| "Failed to make progress" 횟수 | 2 | 4 |
| 충돌 수 (Collision Ahead) | 0 | 0 |
| 목표 성공 횟수 | **0** | **0** |
| 완료 시간 | **57초에 "All frontiers traversed/tried out" 선언 후 조기 종료** | 400초 캡까지 계속 재시도, 미완료 |

## 핵심 발견

### 1. explore_lite는 57초 만에 "탐색 완료"를 잘못 선언한다
3번의 목표 시도가 전부 실패하자 후보 프론티어가 블랙리스트로 소진되어
"All frontiers traversed/tried out, stopping."을 찍고 그대로 정지했다. 맵은
거의 안 그려진 채였다 — 실제로는 갇힌 것인데 대시보드에는 "완료"로 보이는
**조용한 오탐**이다. 어젯밤 실물에서 본 무한 반복과는 다른 실패 양상이지만
(그쪽은 계속 재시도, 이쪽은 3번에 포기), 둘 다 "Failed to make progress"가
근본 방아쇠라는 점은 같다.

### 2. frontier_exploration_ros2는 포기하지 않고 계속 다른 목표를 시도한다
같은 400초 동안 12번 서로 다른(때로 반복되는) 좌표로 목표를 바꿔가며 끝까지
재시도했다. "No more frontiers found"(이쪽의 완료 신호)는 한 번도 안 떴다 —
즉 조기 오판을 안 한다. suppression/escape 로직이 실제로 작동하는 것으로 보임.

### 3. ⚠️ 가장 중요한 발견 — 두 설정 모두 목표를 단 한 번도 성공하지 못했다
`Goal succeeded` 카운트가 **양쪽 다 0**. 실패 로그를 보면 두 설정 모두 같은
패턴을 공유한다: `Failed to make progress` → `Unable to transform robot pose
into global plan's frame` (`tf_help: Transform data too old when converting
from odom to map`) → `Goal failed`. 이건 explore_lite만의 목표선정 문제가
아니라 **컨트롤러/TF 타이밍 쪽의 공통 원인**일 가능성이 높다는 뜻이다.
지난 세션에서 세운 "좁은 통로 때문에 코스트맵이 막혔다" 가설과는 별개로,
"map→odom 변환이 progress_checker의 15초 판정 안에 제때 갱신되는지"를
먼저 봐야 할 것 같다 — 프론티어 선택 전략을 아무리 바꿔도 이 TF 지연이
안 풀리면 둘 다 똑같이 막힌다.

(참고: PC 시뮬은 Xvfb+WSL이라 실시간성이 실물/Pi와 다를 수 있어 TF 타이밍
문제가 시뮬 환경 자체의 부하 때문일 가능성도 배제 못 한다 — Pi에서 같은
`home_real` 월드로 재현되는지 교차검증이 필요하다.)

## 원본 로그
- `run_explore_lite.log` — 294줄, t0~t_end 381초
- `run_frontier.log` — 898줄, t0~t_end 380초

## 이번에 만든/고친 것
- `robot/src/smart_trash_bringup/launch/sim_autonomous_mapping.launch.py`:
  `explorer:=explore_lite|frontier` 인자 추가, frontier_exploration_ros2 노드 통합.
  bt_navigator의 `default_nav_to_pose_bt_xml`이 실물 Pi 홈 경로로 하드코딩돼
  있어 시뮬 환경(계정명 다름)에서 못 열던 문제를 노드-스코프 임시 yaml
  오버라이드로 해결(와일드카드 `/**` dict 오버라이드는 노드명 명시 항목한테
  파일 순서와 무관하게 항상 진다는 걸 확인 — 순서를 바꿔서 될 문제가 아니었음).
- `robot/src/smart_trash_bringup/setup.py`: `behavior_trees/*.xml`을 share에
  설치하도록 `data_files` 추가(기존엔 소스 경로만 참조해서 설치본이 없었음).
- `robot/src/frontier_exploration_ros2/` 신규 clone (mertgulerx/frontier_exploration_ros2, main).
- 아직 git commit 안 함 — PC(WSL)의 로컬 변경 상태.
