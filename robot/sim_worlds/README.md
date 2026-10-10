# 시뮬 세계 파일 보관 (2026-10-10)

`robot/src/stage_ros2/`는 통째로 gitignore라서, 사용자가 손으로 트레이싱한 집 정답 지도(`our_home`)가 **이 파이의 SD 카드에만** 있었다.
여기가 버전 관리되는 원본이다. Stage가 읽는 곳은 `robot/src/stage_ros2/world/`이므로 고치면 그쪽으로 복사한다:

    cp robot/sim_worlds/our_home.world robot/src/stage_ros2/world/
    cp robot/sim_worlds/bitmaps/* robot/src/stage_ros2/world/bitmaps/

- `our_home.world`, `bitmaps/our_home.png`: 14.9 × 9.85 m, 0.05 m/px (2026-08-23, GIMP 트레이싱)
- `tools/step_nav_map_sim.py`, `tools/inflation_probe/`도 이 지도를 쓴다
