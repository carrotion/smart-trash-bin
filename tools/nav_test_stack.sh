#!/usr/bin/env bash
# Nav2 시험 스택(탐색기 없음) 켜기/끄기 — 로봇이 스스로 움직이지 않는 상태로 코스트맵 등을 시험할 때.
#
#   tools/nav_test_stack.sh start   # 라이다 서비스 내리고 autonomous_mapping(enable_explore:=false) 기동
#   tools/nav_test_stack.sh start explore   # 탐색 구성(주행기 서버 + 탐색기)으로 기동. 탐색기는 **멈춘 채로** 뜬다 —
#                                           # 지도 대시보드(8768)에서 금지 구역을 그리고 [탐색 시작]을 눌러야 움직인다(10-11)
#   MAP=home_20261011 tools/nav_test_stack.sh start [explore]   # **저장된 지도로 시작**(10-11): 지금 스캔으로 지도 위 자리를 찾아
#                                           # (locate_on_map.py) 거기서 위치 잡기를 켜고, 그 지도의 금지 구역을 불러온다. 새 지도를 그리지 않는다
#   ZONES_FROM=home_20261011 tools/nav_test_stack.sh start explore   # **새 지도를 그리되 저장 지도의 금지 구역을 옮겨 와 처음부터 적용**(11-01, tools/carry_zones.py)
#   tools/nav_test_stack.sh stop    # 런치와 자식 노드 전부 종료 확인 → 라이다 서비스 복구
#
# ⚠️ 2026-09-27: ros2 launch 본체에 SIGINT를 보냈더니 본체만 죽고 자식 노드 9개(라이다 드라이버 포함)가
#   남았다. 그 상태로 라이다 서비스를 켜서 드라이버 두 개가 같은 포트(/dev/lidar)에 붙을 뻔했다.
#   그래서 stop은 자식까지 확인한 뒤에만 라이다 서비스를 켠다.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /home/carrotian/catkin_ws/install/setup.bash
LOG=${NAV_TEST_LOG:-/tmp/nav_test_stack.log}
NODES="ldlidar_stl_ros2_node scan_to_scan_filter_chain scan_resample_node async_slam_toolbox_node localization_slam_toolbox_node controller_server planner_server behavior_server bt_navigator lifecycle_manager collision_monitor explore keepout_server.py explore_map_filter.py frontier_explorer step_nav_action_server.py static_transform_publisher"   # localization_slam_toolbox_node: 10-11 빠져 있어 저장 지도 방식의 SLAM이 남았는데 "남은 프로세스 0"으로 보고함   # static_transform_publisher: 런치의 라이다 자리 변환이 매번 남았다(10-11 발견, 서비스 소속 IMU 변환은 killproc이 건너뜀)   # collision_monitor: 09-27 빠져 있어 남음
KP=/home/carrotian/smart-trash-bin/tools/killproc.py

case "$1" in
  start)
    EXTRA=""
    if [ -n "$MAP" ]; then
      # 저장된 지도로 시작. ⚠️ 자리 찾기는 라이다 서비스의 스캔(관제 대시보드 경유)을 쓰므로 서비스를 내리기 **전에** 한다
      T=/home/carrotian/smart-trash-bin/tools; B=$HOME/maps/$MAP
      POSE=$(python3 $T/locate_on_map.py --map "$B" --whole --json | python3 -c "
import sys, json, math
r = json.load(sys.stdin); b = r[0]
if b['on_wall'] < 0.85 or (len(r) > 1 and r[1]['score'] > 0.85 * b['score']):
    sys.stderr.write('자리를 확신하지 못함: 벽 위 %.0f%%, 1등 %.2f / 2등 %.2f\\n' % (b['on_wall'] * 100, b['score'], r[1]['score'] if len(r) > 1 else 0)); sys.exit(1)
print('%.3f,%.3f,%.2f' % (b['x'], b['y'], math.degrees(b['yaw'])))") || { echo "❌ 저장된 지도에서 지금 자리를 찾지 못했다 — 로봇을 벽이 잘 보이는 곳으로 옮기거나 MAP 없이(새 지도) 시작"; exit 1; }
      python3 $T/localize_params.py --map "$B" --pose="$POSE" --out /tmp/slam_localize.yaml || exit 1
      python3 -c "
import sys; sys.path.insert(0, '$T'); import keepout
z = keepout.load('$B.keepout.json')['zones']; keepout.save(dict(keepout.load(), zones=z, prev=[])); print('금지 구역 %d개를 지도에서 불러옴' % len(z))"
      export KEEPOUT_KEEP=1
      EXTRA="slam_launch:=localization_launch.py slam_params_file:=/tmp/slam_localize.yaml"
    fi
    if [ -n "$ZONES_FROM" ]; then
      # 새 지도를 그리되 저장 지도의 금지 구역을 처음부터 적용(스캔으로 자리를 찾아 좌표를 옮김). 라이다 서비스를 내리기 전에 한다
      python3 /home/carrotian/smart-trash-bin/tools/carry_zones.py --map "$ZONES_FROM" --write || { echo "❌ 구역을 옮기지 못했다 — 시작하지 않음"; exit 1; }
      export KEEPOUT_KEEP=1
    fi
    sudo -n systemctl stop smart-trash-lidar || exit 1
    if [ "$2" = explore ]; then
      ARGS="enable_explore:=true navigator:=step explorer:=frontier explore_autostart:=false"
    else
      ARGS="enable_explore:=false navigator:=nav2"
    fi
    nohup ros2 launch smart_trash_bringup autonomous_mapping.launch.py $ARGS $EXTRA > "$LOG" 2>&1 &
    if [ -n "$MAP" ]; then
      # 위치 잡기 노드는 수명 주기 노드라 런치가 깨워 줘야(configure→activate) 지도 좌표계가 생긴다. 10-11 두 번째 실물 기동에서
      # 런치가 깨우지 못하고 unconfigured로 남았다(오류 기록 없음, 원인 미확인) → Nav2도 지도 좌표계를 60초 기다리다 기동 포기.
      # 그래서 상태를 직접 보고, 안 깨어 있으면 여기서 깨운다. Nav2의 60초 안에 끝나야 하므로 costmap 대기보다 먼저 한다
      sleep 4
      for i in $(seq 1 10); do
        S=$(timeout 10 ros2 lifecycle get /slam_toolbox 2>/dev/null | awk '{print $1}')
        [ "$S" = active ] && break
        if [ "$S" = unconfigured ] && [ $i -ge 3 ]; then echo "위치 잡기 노드가 안 깨어 있어 직접 깨움"; timeout 60 ros2 lifecycle set /slam_toolbox configure; fi
        [ "$S" = inactive ] && timeout 60 ros2 lifecycle set /slam_toolbox activate
        sleep 2
      done
      [ "$S" = active ] || { echo "❌ 위치 잡기 노드가 켜지지 않음(상태: ${S:-응답 없음}) — stop"; exec "$0" stop; }
    fi
    for i in $(seq 1 30); do
      sleep 1
      ros2 topic list 2>/dev/null | grep -q '^/local_costmap/costmap_raw$' && break
    done
    if ps aux | grep -v grep | grep -q '[e]xplore_lite/lib/explore_lite/explore'; then
      echo "❌ 탐색기가 떠 있다 — 즉시 stop"; exec "$0" stop
    fi
    if [ "$2" = explore ]; then
      echo "탐색 스택 기동(탐색기는 멈춤 — 지도 대시보드의 [탐색 시작]으로 출발). 로그: $LOG"; exit 0
    fi
    echo "Nav2 시험 스택 기동(탐색기 없음). 로그: $LOG"
    ;;
  stop)
    LP=$(pgrep -f "ros2 launch smart_trash_bringup autonomous_mapping")
    [ -n "$LP" ] && kill -INT $LP && sleep 6
    python3 "$KP" $NODES >/dev/null 2>&1   # 서비스 소속은 killproc이 건너뜀
    sleep 2
    # 런치 본체는 자식이 먼저 죽으면 SIGINT 뒤에도 매달려 있었다(09-27 실측) → PID로 TERM, 그래도 남으면 KILL
    LP=$(pgrep -f "ros2 launch smart_trash_bringup autonomous_mapping")
    [ -n "$LP" ] && kill -TERM $LP && sleep 2
    LP=$(pgrep -f "ros2 launch smart_trash_bringup autonomous_mapping")
    [ -n "$LP" ] && kill -KILL $LP && sleep 1
    # 떠돌이 = 관련 이름인데 systemd 서비스 소속이 아닌 프로세스(서비스 라이다는 정상)
    SVC=$(cat /sys/fs/cgroup/system.slice/smart-trash-*.service/cgroup.procs 2>/dev/null | tr '\n' ' ')
    STRAY=$(ps -eo pid,args | grep -v grep | grep -E "$(echo $NODES | tr ' ' '|')" | awk -v svc=" $SVC " 'index(svc, " "$1" ")==0 {print $1, $2}')
    if [ -n "$STRAY" ]; then
      echo "❌ 서비스 밖 관련 프로세스가 남아 있다 — 라이다 서비스 켜지 않음:"; echo "$STRAY" | sed 's/^/    /'
      exit 1
    fi
    sudo -n systemctl start smart-trash-lidar
    sleep 4
    echo "Nav2 시험 스택 종료(남은 프로세스 0), 라이다 서비스 $(systemctl is-active smart-trash-lidar)"
    ;;
  *) echo "사용: $0 start|stop"; exit 2 ;;
esac
