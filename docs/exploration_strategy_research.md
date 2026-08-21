# 탐색 전략 재설계 — 선행연구 조사 + 실험 시나리오 (2026-08-21)

## 배경

3차례 실주행에서 반복된 문제: 벽에 여러 번 부딪힘, 좌우로 흔들며(지그재그) 주행,
구석에 갇혀 벽을 미는 복구 동작 반복. 사용자 지적: *"넓고 주변에 장애물이 없는곳으로만
피해서 이동해도 충분히 좋은 맵을 그릴수 있을것 같은데... 큰 그림을 그리는게 그렇게
어려운건가? 그림 그릴때 어떻게 하니? 크게 스케치하고, 그리고 디테일 아니야?"*

이 시점까지의 대응은 매번 **파라미터를 하나씩 고치고 실주행으로 반증**하는 방식이었다
(`potential_scale`/`gain_scale` 조정, `BaseObstacle.scale` 수정, `Twirling` critic
추가 등). 사용자가 명시적으로 이 방식을 중단시켰다: *"확실히 성공할 수 있는 방법론을
연구한 후에 실험 방법을 몇가지 시나리오를 만들어서 하나씩 테스트 해보자. 지금은 완전
맨땅에 헤딩이야. 좀 전문가답게 해보자."* — 이 문서는 그 요청에 대한 결과물이다.

## 조사 결과

### 1) "크게 스케치 → 디테일"은 실제 업계 표준이다

로봇청소기 SLAM 문헌에 정확히 이 패턴이 문서화돼 있다: 로봇은 먼저 **한쪽 이상의
벽을 따라가며 시작점에서 루프를 닫아 방 경계를 확정**하고, 그 벽을 지도의 기준
경계로 삼은 뒤에 커버리지(전체 바닥 훑기)를 시작한다. 커버리지 자체도 **방의 긴
변에 맞춰 평행선을 그어 회전을 최소화**하는 방식(boustrophedon, 지그재그 아님 —
방향이 있는 왕복 직선)이다.

→ 사용자의 "크게 스케치하고 디테일"이라는 직관은 맨땅에 헤딩이 아니라 검증된
업계 방법론과 일치한다.

### 2) Nav2에는 "방 경계부터"를 그대로 하는 기성 패키지가 없다

Nav2/ROS2 생태계 조사 결과, "탐색 중에 방 경계를 먼저 확정하고 그 다음 디테일"을
그대로 구현한 패키지는 발견되지 않았다. 벽 추종(wall-following)은 주로 생체모방
로봇이나 다른 맥락에서 쓰이고, Nav2의 표준 탐색기(`explore_lite`,
`frontier_exploration_ros2` 등)는 전부 프론티어 기반이라 "가까운 미지영역"을
쫓아가는 방식 — 이게 지금 우리가 겪는 지그재그의 근본 원인이다.

### 3) 대신 "커버리지" 단계에 쓸 수 있는 진짜 기성 부품이 있다 — `opennav_coverage`

Open Navigation(Nav2 공식 관리 조직)이 유지보수하는 **`opennav_coverage`**
(`Fields2Cover` 라이브러리를 감싼 Nav2 Lifecycle Task Server)는 **폴리곤(경계)이
주어지면** 그 안을 boustrophedon 패턴으로 훑는 경로를 계산해 Nav2로 실행해준다.

⚠️ **정직한 한계**: 이 패키지는 "이미 아는 폴리곤"을 커버하는 도구다. 폴리곤을
**실시간으로 발견**하는 문제(=지금 우리가 겪는 탐색 단계)는 이 패키지의 범위가
아니다. 즉 이것은 "1단계: 경계 확보"의 대체재가 아니라 **"2단계: 디테일 채우기"의
대체재**다.

## 결론 — 2단계 아키텍처

| 단계 | 목표 | 방법 | 새 코드 필요? |
|---|---|---|---|
| **1. 대략적 경계** | 벽만 크게 스캔, 작은 장애물은 무시 | 기존 `explore_lite`의 `min_frontier_size`를 크게(0.5~1.0m) 키워서 큰 프론티어만 쫓게 함 | ❌ 파라미터만 |
| **2. 경계 폴리곤 추출** | 1단계 결과 맵에서 방 경계 폴리곤 뽑기 | occupancy grid → 최대 자유공간 컨투어 → 폴리곤 (간단한 스크립트) | ✅ 신규, 작음 |
| **3. 디테일 커버리지** | 폴리곤 안을 평행선으로 훑기 | `opennav_coverage`/`Fields2Cover` | ❌ 기성품 |

각 단계가 독립적으로 검증 가능하다는 게 핵심 — 지금까지처럼 전체를 한 번에
실주행으로 검증할 필요가 없다.

## 실험 시나리오 (하나씩 순서대로)

### 시나리오 A — 1단계만 격리 검증 (가장 먼저, 가장 낮은 리스크)

기존 `explore_lite`를 그대로 쓰되 `min_frontier_size`를 0.5 → 0.8~1.0m로 올려서,
작은 프론티어(가구 틈, 구석)를 무시하고 큰 벽만 쫓는지 확인한다.
- **바꾸는 것**: `explore_params.yaml`의 `min_frontier_size` 하나
- **측정**: `nav_session_logger.py`로 궤적 기록 → 회전 비율/직진 비율이 지금(회전
  많음)보다 나아지는지, 초반 몇 분간 "큰 스윕" 형태가 나오는지 궤적 플롯으로 확인
- **성공 기준**: 회전 비율 감소 + 궤적이 방 둘레를 도는 모양

### 시나리오 B — 폴리곤 추출 스크립트, 오프라인 검증

시나리오 A의 결과 맵(pgm/yaml)을 갖고, 실주행 없이 **오프라인으로** 폴리곤 추출
스크립트를 검증한다(occupancy grid 최대 연결 자유공간 → 컨투어 → 단순화된 폴리곤).
- **측정**: 추출된 폴리곤을 실제 방 모양과 육안 비교(맵 대시보드에 오버레이)
- **성공 기준**: 폴리곤이 방 벽선과 대체로 일치, 자기 파손 없음(자기교차 없는 단순
  다각형)
- 이 단계는 로봇을 움직이지 않으므로 몇 번이든 반복 조정 가능

### 시나리오 C — `opennav_coverage` 단독 검증 (수동 폴리곤으로)

시나리오 B 없이도 먼저 해볼 수 있는 것 — 지금 있는 방을 손으로 대충 측정해 만든
**수동 폴리곤**을 `opennav_coverage`에 직접 넣어서, 패키지 자체가 우리 환경에서
정상 동작하는지(설치, 경로 생성, Nav2 실행 연동)만 먼저 검증한다.
- **바꾸는 것**: 새 패키지 설치 + 설정, 로봇 코드는 안 건드림
- **측정**: 생성된 경로가 방 긴 변에 맞춰 평행선인지, Nav2가 그 경로를 실제로
  따라가는지
- 이걸 먼저 떼어놓고 검증하면, 나중에 A+B와 합칠 때 "커버리지 자체가 안 되는 건지
  폴리곤이 나쁜 건지" 원인을 분리할 수 있다

### 시나리오 D — 전체 파이프라인 결합 (A→B→C)

A, B, C가 각각 개별 검증을 통과한 뒤에만 진행. 실제 순서: A로 경계 스캔 →
결과 맵에서 B로 폴리곤 추출 → C로 커버리지 실행.

## 진행 순서 제안

**A부터 시작.** 새 코드가 없고, 리스크가 가장 낮고, 지금 겪는 문제(회전 과다)에
직접적으로 관련된 가설을 검증한다. A 결과가 나쁘면 B/C로 넘어가기 전에 프론티어
비용함수 자체를 더 봐야 한다는 뜻이므로, 순서대로 가는 것이 합리적이다.

## Sources

- [Robot Vacuum Cleaner Part 3/3 — CSE 468/568 Robotic Algorithms](https://medium.com/cse-468-568-robotic-algorithms/robot-vacuum-cleaner-part-3-3-2bc317cf17db)
- [CSE 568 Project Blog Post 1](https://medium.com/cse-468-568-robotic-algorithms/cse-568-project-blog-post-1-bce87fb50b0b)
- [Robot Vacuum Mapping Technology Explained: SLAM, LiDAR Guide](https://www.guidespot.com/robot-vacuum-mapping-technology-explained/)
- [Robot Vacuum Navigation Types Explained — Tech Advisor](https://www.techadvisor.com/article/2283951/robot-vacuum-navigation-types-explained.html)
- [opennav_coverage (Open Navigation, 공식 Nav2 패키지)](https://github.com/open-navigation/opennav_coverage)
- [Coverage Server — Nav2 documentation](https://docs.nav2.org/configuration/packages/configuring-coverage-server.html)
- [ComputeCoveragePath — Nav2 BT plugin docs](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputeCoveragePath.html)
- [Fields2Cover ROS wrapper](https://github.com/Fields2Cover/fields2cover_ros)
- [ROS2 mobile robotics algorithm survey (arXiv)](https://arxiv.org/pdf/2307.15236)
- [Husarion Nav2 exploration tutorial](https://husarion.com/tutorials/ros2-tutorials/10-exploration/)
