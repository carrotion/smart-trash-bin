#!/usr/bin/env bash
# Nav2 시험 스택(탐색기 없음) 켜기/끄기 — 로봇이 스스로 움직이지 않는 상태로 코스트맵 등을 시험할 때.
#
#   tools/nav_test_stack.sh start   # 라이다 서비스 내리고 autonomous_mapping(enable_explore:=false) 기동
#   tools/nav_test_stack.sh start explore   # 탐색 구성(주행기 서버 + 탐색기)으로 기동. 탐색기는 **멈춘 채로** 뜬다 —
#                                           # 지도 대시보드(8768)에서 금지 구역을 그리고 [탐색 시작]을 눌러야 움직인다(10-11)
#   tools/nav_test_stack.sh stop    # 런치와 자식 노드 전부 종료 확인 → 라이다 서비스 복구
#
# ⚠️ 2026-09-27: ros2 launch 본체에 SIGINT를 보냈더니 본체만 죽고 자식 노드 9개(라이다 드라이버 포함)가
#   남았다. 그 상태로 라이다 서비스를 켜서 드라이버 두 개가 같은 포트(/dev/lidar)에 붙을 뻔했다.
#   그래서 stop은 자식까지 확인한 뒤에만 라이다 서비스를 켠다.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /home/carrotian/catkin_ws/install/setup.bash
LOG=${NAV_TEST_LOG:-/tmp/nav_test_stack.log}
NODES="ldlidar_stl_ros2_node scan_to_scan_filter_chain scan_resample_node async_slam_toolbox_node controller_server planner_server behavior_server bt_navigator lifecycle_manager collision_monitor explore keepout_server.py frontier_explorer step_nav_action_server.py static_transform_publisher"   # static_transform_publisher: 런치의 라이다 자리 변환이 매번 남았다(10-11 발견, 서비스 소속 IMU 변환은 killproc이 건너뜀)   # collision_monitor: 09-27 빠져 있어 남음
KP=/home/carrotian/smart-trash-bin/tools/killproc.py

case "$1" in
  start)
    sudo -n systemctl stop smart-trash-lidar || exit 1
    if [ "$2" = explore ]; then
      ARGS="enable_explore:=true navigator:=step explorer:=frontier explore_autostart:=false"
    else
      ARGS="enable_explore:=false navigator:=nav2"
    fi
    nohup ros2 launch smart_trash_bringup autonomous_mapping.launch.py $ARGS > "$LOG" 2>&1 &
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
