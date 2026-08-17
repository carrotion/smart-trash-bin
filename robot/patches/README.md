# m-explore-ros2 패치: 프론티어 목표 지점 선택
#
# 적용 대상 : https://github.com/robo-friends/m-explore-ros2.git
# 브랜치    : feature/slam_toolbox_compat
# 기준 커밋 : 9b64d8d35213dd50bc44f1b5b4959c1f58c32d2e
# 작성      : 2026-08-17

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
