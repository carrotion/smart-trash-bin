#!/bin/bash
# 5층 탐색 시뮬 한 번 돌리기(Stage, 도메인 42, 로봇 없이): 탐색기(frontier) + 4층 주행기 액션 서버 + 지도 넓이 기록.
#   tools/sim_explore_run.sh <출력 폴더> <탐색 시간 초> <world> [frontier 파라미터 yaml, 기본 tools/sim_frontier_step.yaml] [주행기 서버 0/1, 기본 1]
#   예: tools/sim_explore_run.sh /tmp/sim_a 600 our_home
# 결과: <출력>/coverage.csv(5초마다 지도 넓이), server.log, docs/calibration/step_nav_server_*_sim.jsonl(목표별 기록)
# ⚠️ 10-09엔 이 스크립트가 세션 임시 폴더에만 있다가 재부팅으로 사라졌다 → 저장소에 둠(10-10)
OUT=$1; DUR=$2; WORLD=$3; FP=${4:-/home/carrotian/smart-trash-bin/tools/sim_frontier_step.yaml}; STEP=${5:-1}
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash; source /home/carrotian/catkin_ws/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/carrotian/smart-trash-bin
ARGS="world:=$WORLD explorer:=frontier"; [ -n "$FP" ] && ARGS="$ARGS frontier_params_file:=$FP"
# 저장 지도에서 위치 잡기 시험: SLAM_LAUNCH=localization_launch.py SLAM_PARAMS=<localize_params.py --sim 으로 만든 파일>
[ -n "$SLAM_LAUNCH" ] && ARGS="$ARGS slam_launch:=$SLAM_LAUNCH slam_params_file:=$SLAM_PARAMS"
xvfb-run -a bash -c "timeout -s INT $((DUR + 45)) ros2 launch smart_trash_bringup sim_autonomous_mapping.launch.py $ARGS" > $OUT/launch.log 2>&1 &
sleep 25
# 금지 구역 서버: 시뮬 전용 파일(KEEPOUT_FILE, 기본 <출력>/keepout.json — 미리 구역을 써 두면 그대로 적용, --keep)
export KEEPOUT_FILE=${KEEPOUT_FILE:-$OUT/keepout.json}
python3 -u tools/keepout_server.py --keep --file "$KEEPOUT_FILE" > $OUT/keepout.log 2>&1 &
KP_PID=$!
# 탐색기용 지도 필터(닿을 수 있는 곳의 경계만 남김, /map_explore). 탐색기가 이 토픽을 보게 하려면 파라미터 파일에
#   map_topic: /map_explore (tools/sim_frontier_step_filter.yaml). EXPLORE_FILTER=0이면 띄우지 않는다
if [ "${EXPLORE_FILTER:-1}" = 1 ]; then
  python3 -u tools/explore_map_filter.py > $OUT/map_filter.log 2>&1 &
  MF_PID=$!
fi
if [ "$STEP" = 1 ]; then
  python3 -u tools/step_nav_action_server.py --odom-topic /odom --gyro-from-odom --label sim $STEP_NAV_ARGS > $OUT/server.log 2>&1 &
  SP=$!
  # ⚠️ 11-01: 서버가 인자 오류로 안 떴는데 시뮬이 15분씩 두 번 끝까지 돌았다 → 죽어 있으면 기다리지 않는다
  sleep 8
  if ! kill -0 $SP 2>/dev/null; then echo "❌ 주행기 서버가 뜨지 않음 — $OUT/server.log"; tail -2 $OUT/server.log; DUR=1; fi
fi
timeout $((DUR + 10)) python3 tools/map_coverage_logger.py --out $OUT/coverage.csv --sim-time --interval 5 --duration $DUR > $OUT/coverage.log 2>&1
[ -n "$SP" ] && kill -TERM $SP
kill -TERM $KP_PID $MF_PID 2>/dev/null
wait
python3 tools/killproc.py --kill9 stage_ros2 async_slam_toolbox_node controller_server planner_server behavior_server bt_navigator lifecycle_manager collision_monitor frontier_explorer Xvfb > $OUT/cleanup.log 2>&1
python3 -c "import sys; sys.path.insert(0,'tools'); import mapping_cleanup; mapping_cleanup.clean_orphan_shm()" >> $OUT/cleanup.log 2>&1
echo done > $OUT/DONE
