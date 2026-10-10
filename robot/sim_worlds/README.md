# 시뮬 세계 파일 보관 (2026-10-10)

`robot/src/stage_ros2/`는 통째로 gitignore라서, 사용자가 손으로 트레이싱한 집 정답 지도(`our_home`)가 **이 파이의 SD 카드에만** 있었다.
여기가 버전 관리되는 원본이다. Stage가 읽는 곳은 `robot/src/stage_ros2/world/`이므로 고치면 그쪽으로 복사한다:

    cp robot/sim_worlds/our_home.world robot/src/stage_ros2/world/
    cp robot/sim_worlds/bitmaps/* robot/src/stage_ros2/world/bitmaps/
    cp robot/sim_worlds/include/smart_trash.inc robot/src/stage_ros2/world/include/
    # 새 파일은 설치 폴더에 링크가 없다(파일별 심볼릭 링크) → 한 번 걸어 준다(또는 colcon build --packages-select stage_ros2)
    ln -sf $PWD/robot/src/stage_ros2/world/include/smart_trash.inc robot/install/stage_ros2/share/stage_ros2/world/include/

- `our_home.world`, `bitmaps/our_home.png`: 14.9 × 9.85 m, 0.05 m/px (2026-08-23, GIMP 트레이싱)
- `tools/step_nav_map_sim.py`, `tools/inflation_probe/`도 이 지도를 쓴다
- `include/smart_trash.inc`: **우리 로봇 모델**(0.22 × 0.21 m, 라이다 앞 0.054 m·360°·12 m). 10-10 전의 Stage 결과는 전부 Pioneer(0.44 × 0.38 m)였다
- 문 폭(10-10 사용자 실측): A(가운데 위 방)·B(오른쪽 위 방) 77 cm → 지도 0.75 m, C(왼쪽 큰 방) 87 cm → 0.85 m. `bitmaps/our_home_doors_abc.png`에 위치 표시
- `bitmaps/our_home_narrowdoors.png`: 08-23 원본(문 0.45 m) — 좁은 틈 통과 시험용(`tools/step_nav_map_sim.py` 기본 사례)
