# m-explore-ros2 패치: 프론티어 목표 지점 선택 + 실패 시 재시도
#
# 적용 대상 : https://github.com/robo-friends/m-explore-ros2.git
# 브랜치    : feature/slam_toolbox_compat
# 기준 커밋 : 9b64d8d35213dd50bc44f1b5b4959c1f58c32d2e
# 작성      : 2026-08-17 (목표 지점 선택), 2026-08-23 밤 개정 (실패 시 재시도 추가)

## 왜 필요한가

`m-explore-ros2`는 우리 fork가 아니라 upstream을 그대로 서브모듈로 쓴다.
따라서 **재클론하거나 `git submodule update`를 하면 아래 패치가 사라진다.**
그러면 자율탐색이 다시 "맵 10%도 안 그렸는데 `All frontiers traversed`로 종료"
상태로 돌아간다.

## 패치가 고치는 것

explore_lite는 프론티어 덩어리의 **무게중심(centroid)**을 목표로 삼는다.
탐색 초기엔 프론티어가 로봇을 빙 둘러싼 **고리 모양**이라, 그 평균이 고리
한가운데 — 즉 **로봇이 이미 서 있는 자리**가 된다. 목표가 현재 위치이니
즉시 "도달"로 처리되고, 그렇게 프론티어를 소진하면 탐색이 영구 종료된다.

`middle`(최근접 프론티어 점)로 바꿔도 해결되지 않았다. 라이다가 자기
구조물에 가려 있어서 **막힌 방향은 로봇에 딱 붙은 곳부터 미지영역 쐐기**가
생기고, 최근접 점이 그 쐐기(0.12m)로 잡히기 때문이다.

그래서 **덩어리 안에서 `min_frontier_distance` 이상 떨어진 점 중 가장 가까운
것**을 목표로 삼도록 바꿨다. 항상 실제 미지 경계 위에 있고, 반드시 이동을
요구한다. 이동하면 사각지대도 같이 움직여 쐐기는 스스로 메워진다.

`min_frontier_distance` 파라미터(기본 0.6)도 신설했다.
설정은 `smart_trash_bringup/config/explore_params.yaml`.

**검증**: 적용 전 4회 연속 교착(주행 0.00m). 적용 후 110초에 11.57m 주행,
맵 3.3 -> 57.3 제곱미터.

## 적용 방법

```bash
cd ~/smart-trash-bin/robot/src/m-explore-ros2
git apply ../../patches/m-explore-ros2_frontier_goal_selection.patch
cd ~/smart-trash-bin/robot
colcon build --symlink-install --packages-select explore_lite
```

적용 여부 확인:

```bash
grep -c min_frontier_distance_ robot/src/m-explore-ros2/explore/src/explore.cpp
# 2가 나오면 적용된 상태 (get_parameter + 비교문)
```

## TODO

`gh` CLI나 토큰이 없어 지금은 fork를 못 만들었다. 나중에 `ldlidar_stl_ros2`처럼
`carrotion/m-explore-ros2`로 fork하고 정식 커밋한 뒤, 서브모듈 remote를 그쪽으로
바꾸면 이 패치 파일은 필요 없어진다.

## 검증

패치가 제대로 적용됐는지 확인:

```bash
grep -c min_frontier_distance_ explore/src/frontier_search.cpp   # -> 6
grep -c min_frontier_distance_ explore/include/explore/frontier_search.h  # -> 1
grep -c min_frontier_distance_ explore/src/explore.cpp           # -> 2
```

## 이 패치가 고치는 것 (2026-08-17 개정)

핵심은 **"비용 계산과 목표점이 같은 점을 가리킨다"는 불변식**이다.

원본은 프론티어 덩어리에서 로봇에 **가장 가까운 셀**을 목표(`middle`)로 잡고 그
거리를 비용 입력(`min_distance`)으로 썼다. 우리 환경에서는 그 목표가 0.1m 앞이라
Nav2가 즉시 "도달"로 처리하고, 로봇은 이동 없이 다음 프론티어로 넘어가며 제자리
회전만 반복했다(실측: 순수 회전 45~71%).

1차 시도는 `explore.cpp`에서 **목표점만** 따로 골랐다. 그러자 순위는 최근접
거리로 매기고 목표는 다른 점이 되는 **불일치**가 생겼다 — 로봇 코앞에서 시작하는
큰 덩어리들은 `min_distance`가 똑같이 0.1m라 순위를 못 가리는데 목표점은 전혀
다른 방향에 찍혔다. 목표가 계속 튀었다.

2차(현재)는 선정을 `FrontierSearch::buildNewFrontier`로 옮겼다.
`min_frontier_distance` 이상 떨어진 점 중 가장 가까운 것을 목표로 삼고,
`min_distance`에도 **그 점까지의 거리**를 넣는다. 구조적으로 불일치가 생길 수 없다.

예외 처리 두 가지:
- 덩어리 전체가 `min_frontier_distance` 안에 들어오면 가장 먼 점을 쓴다(거리도 그에 맞춤)
- `points`가 비는 고립 셀이면 `middle`을 `initial`로 둔다. 안 그러면 기본값 (0,0),
  즉 **맵 원점**이 목표가 된다

## 이 패치가 고치는 것 (2026-08-23 밤 추가 — 실패 시 재시도)

시뮬레이션(`docs/pc_tasks.md` Phase 1.5)에서 새로 발견된 문제: 목표가
`ABORTED`되면 원본 코드는 **단 1회 실패만으로 즉시 영구 blacklist**였다
(`reachedGoal`). 시뮬 부트스트랩 극초반(SLAM/코스트맵 초기화 부하가 몰리는
시점)에 map->odom TF가 살짝 지연되면 첫 목표가 그 순간 실패하고, 그 즉시
blacklist에 들어가 다음 계획에서 후보가 하나도 안 남아 "All frontiers
traversed"로 완전히 포기해버린다.

**진단 과정**: 처음엔 launch 쪽에서 SLAM/Nav2가 활성화된 뒤 explore_node를
지연 기동시키는 완화책(a)를 시도했다 — 순차 실행에서는 완전히 해소됐지만
(변동폭 6배→1.3배), **N=4 병렬(자원경합 심함) 조건에서는 4/4(100%) 그대로
재현됐다.** 원인: "Managed nodes are active"(Nav2 lifecycle 전이)는 "SLAM이
실제로 안정된 TF를 발행 중"을 보장하지 않는다 — 별개 신호다. 병렬 조건에서는
activation 이후에도 SLAM이 계속 지연될 수 있다.

그래서 "부트스트랩 시점에 더 똑똑하게 기다리기"(문턱을 옮기는 것) 대신, **"일시적
실패를 영구 실패로 오판하지 않는다"를 explore_lite 자체의 불변조건으로 만든다**
— 좌표별 실패 횟수(`frontier_failure_counts_`, `getFailureCount`)를 세서
`max_frontier_retries`(기본 2) 미만이면 blacklist 대신 `makePlan()` 재호출로
재시도한다. 부트스트랩 때뿐 아니라 런 중간의 자원경합에도 동일하게 방어된다.

`max_frontier_retries` 파라미터(기본 2)도 신설. 설정은
`smart_trash_bringup/config/explore_params.yaml`.

**검증**: (a)만 적용한 N=4 병렬 4회 — 4/4 조기종료(goal_switches=1). (a)+(b)
적용 후 N=4 병렬 4회 재검증 — **조기종료 0/4**, goal_switches 14~23회,
"retrying instead of blacklisting" 로그가 런당 10~13회 발동(재시도 로직이
실제로 여러 번 발동해서 탐색을 이어갔다는 직접 증거). 상세: `docs/pc_tasks.md`,
[Issue #1](https://github.com/carrotion/smart-trash-bin/issues/1).

---

# rf2o_laser_odometry 패치: Jazzy 빌드 수정 (2026-08-29)

적용 대상: https://github.com/linuxsen/rf2o_laser_odometry_ros2.git (main)
패치 파일: `rf2o_laser_odometry_jazzy_build_fix.patch`

## 왜 필요한가

포크가 아니라 upstream을 그대로 클론해 쓴다(`.gitignore`에 디렉터리 통째로
제외돼 있음 — 재클론하면 이 패치가 사라진다). 원본이 Humble 기준이라 Jazzy에서
바로 빌드가 안 됨:

1. **최신 Eigen과의 비호환**: `CLaserOdometry2D.cpp`에서 `float`를 행렬 인덱스로
   바로 넣던 코드(`wacu(std::round(uwarp))`)가 최신 Eigen에서 `IndexedView`로
   해석되며 컴파일 실패(`YOU_TRIED_CALLING_A_VECTOR_METHOD_ON_A_MATRIX`).
   `int`로 명시 캐스팅해서 해결.
2. **tf2 헤더 확장자**: Jazzy부터 `tf2_geometry_msgs/tf2_geometry_msgs.h`가
   `.hpp`로 바뀜(`.h`는 삭제됨). `CLaserOdometry2DNode.h`에서 확장자만 수정.

기능은 원본 그대로 — 순수 빌드 호환성 패치.

## 적용 방법

```bash
cd ~/smart-trash-bin/robot/src
git clone https://github.com/linuxsen/rf2o_laser_odometry_ros2.git rf2o_laser_odometry
cd rf2o_laser_odometry
git apply ../../patches/rf2o_laser_odometry_jazzy_build_fix.patch
cd ~/smart-trash-bin/robot
colcon build --symlink-install --packages-select rf2o_laser_odometry
```

## 실행 시 주의

`launch/rf2o_laser_odometry.launch.py` 기본값 `publish_tf: True`를 **반드시
`False`로 덮어쓸 것** — 그대로 두면 `smart-trash-ekf.service`가 이미 발행하는
`odom->base_link` TF와 충돌한다. 지금은 비교 로깅 단계(`/odom_rf2o` 토픽만
받아 EKF와 대조)이지 융합/교체가 아니다 — `docs/midterm_review_20260829.md`
계획 2번 참고.

## TODO

fork 안 만듦(Jazzy 대응이 upstream에 없어 언젠가 필요할 수 있음). 채택이
확정되면 `carrotion/rf2o_laser_odometry`로 fork 후 서브모듈 전환 검토.
