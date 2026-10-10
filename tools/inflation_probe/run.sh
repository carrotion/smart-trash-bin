#!/usr/bin/env bash
# 전역 코스트맵 팽창 반경 판정 시험 — 도메인 42에서 map_server(our_home 정답 지도) + planner_server만 띄우고 무작위 60쌍 경로 측정.
#   tools/inflation_probe/run.sh 0.35     # 끝나면 프로세스 정리까지
# 결과(09-27): 0.20 → 60/60·7.48 m·장애물 0.21 m 안 4.2% / 0.35 → 60/60·7.48 m·1.4% / 0.50 → 60/60·7.71 m·1.3%
set -e; cd "$(dirname "$0")"; source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42
# ⚠️ 10-10: 끝날 때 이름으로 정리하므로(map_server·planner_server·lifecycle_manager) 실물 시험 스택이 떠 있으면 그것까지 죽인다 → 시작 거부
if ps -eo args | grep -E "/(planner_server|map_server|controller_server|async_slam_toolbox_node) " | grep -v grep > /dev/null; then
  echo "❌ 이미 Nav2 프로세스가 떠 있다 — 실물 시험 스택/다른 시뮬을 먼저 내릴 것"; exit 1
fi
R=$1; P=/tmp/inflation_probe_$R.yaml; sed "s/RADIUS/$R/; s#yaml_filename: map.yaml#yaml_filename: $PWD/map.yaml#" params.template.yaml > $P
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link > /tmp/ip_tf.log 2>&1 & T=$!
ros2 run nav2_map_server map_server --ros-args --params-file $P > /tmp/ip_map.log 2>&1 & M=$!
ros2 run nav2_planner planner_server --ros-args --params-file $P > /tmp/ip_planner.log 2>&1 & PL=$!
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_infl --params-file $P > /tmp/ip_lm.log 2>&1 & L=$!
sleep 8
python3 probe.py $R || true
# ⚠️ ros2 run은 파이썬 실행기라 그 PID를 죽여도 노드가 남는다(09-27 확인) → 이름으로 정리(서비스 소속은 killproc이 건너뜀)
python3 ../killproc.py --kill9 map_server planner_server lifecycle_manager >/dev/null 2>&1 || true
ps -eo pid,args | awk '/[s]tatic_transform_publisher --frame-id map --child-frame-id base_link/ {print $1}' | xargs -r kill
kill $L $PL $M $T 2>/dev/null || true
