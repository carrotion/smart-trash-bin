#!/usr/bin/env bash
# 실물 탐색 중 기록 다섯 가지를 한 번에 켜고 끈다 (2026-11-22).
# 9번째 탐색(11-14)에서 손으로 하나씩 띄우던 것을 묶었다. 로봇을 움직이지 않는다 — 기록만 한다.
#
#   tools/explore_logs.sh start explore10 2700   # 이름, 최대 초. logs/<이름>_<날짜>/ 에 남긴다
#   tools/explore_logs.sh stop                   # 켠 것만 끈다(PID 파일로. 이름으로 찾지 않는다 — pkill/pgrep -f 금지)
#
# 남기는 것:
#   bag.log        녹화(tools/record_bag.sh, bags/ 아래)
#   watch.csv      로봇 자세와 금지 구역 선까지 거리(tools/keepout_watch.py) — 아래 둘을 위치에 붙이는 기준
#   coverage.csv   지도 넓이 5초마다(tools/map_coverage_logger.py)
#   server_rtt.csv 서버까지 ICMP 왕복 매초(빈칸 = 잃음). 주소는 ~/.config/smart-trash-bin/server_url에서 — 저장소에 IP를 쓰지 않는다
#   wifi.csv       매초 집 Wi-Fi(wlan0)·핫스팟 동글 신호와 집 Wi-Fi 속도 — 위치별 Wi-Fi 지도의 재료(엣지 우선 7절 "음영 지역")
set -eo pipefail
cd "$(dirname "$0")/.."
PIDS=/tmp/explore_logs.pids
DONGLE=wlx9ca2f483b51e

case "${1:-}" in
start)
  NAME=${2:?이름}; SECS=${3:-2700}
  [ -s "$PIDS" ] && { echo "이미 켜져 있어요($PIDS). 먼저 stop"; exit 1; }
  source /opt/ros/jazzy/setup.bash; source ~/catkin_ws/install/setup.bash
  D=logs/${NAME}_$(date +%Y%m%d); mkdir -p "$D"; echo "$D" > "$PIDS.dir"
  HOST=$(sed -E 's#^[a-z]+://([^:/]+).*#\1#' ~/.config/smart-trash-bin/server_url)
  setsid tools/record_bag.sh "$NAME" "$SECS" > "$D/bag.log" 2>&1 & echo $! >> "$PIDS"
  setsid python3 tools/keepout_watch.py --duration "$SECS" --out "$D/watch.csv" > "$D/watch.log" 2>&1 & echo $! >> "$PIDS"
  setsid python3 tools/map_coverage_logger.py --out "$D/coverage.csv" --interval 5 --duration "$SECS" > "$D/coverage.log" 2>&1 & echo $! >> "$PIDS"
  echo "t,seq,rtt_ms" > "$D/server_rtt.csv"
  setsid bash -c "timeout $SECS ping -D -O -i 1 $HOST | awk '/time=/{match(\$0,/\\[([0-9.]+)\\]/,a);match(\$0,/icmp_seq=([0-9]+)/,s);match(\$0,/time=([0-9.]+)/,r);print a[1]\",\"s[1]\",\"r[1];fflush()} /no answer/{match(\$0,/\\[([0-9.]+)\\]/,a);match(\$0,/icmp_seq=([0-9]+)/,s);print a[1]\",\"s[1]\",\";fflush()}' >> $D/server_rtt.csv" > /dev/null 2>&1 & echo $! >> "$PIDS"
  echo "t,wlan0_dbm,wlan0_mbit,dongle_dbm" > "$D/wifi.csv"
  setsid bash -c "end=\$((SECONDS+$SECS)); while [ \$SECONDS -lt \$end ]; do a=\$(iw dev wlan0 link | awk '/signal/{s=\$2} /tx bitrate/{b=\$3} END{print s\",\"b}'); c=\$(iw dev $DONGLE link 2>/dev/null | awk '/signal/{print \$2}'); echo \"\$(date +%s.%N),\$a,\$c\"; sleep 1; done >> $D/wifi.csv" > /dev/null 2>&1 & echo $! >> "$PIDS"
  sleep 6
  echo "기록 폴더 $D"
  echo "  녹화:     $(tail -1 "$D/bag.log" | cut -c1-80)"
  echo "  자세:     $(wc -l < "$D/watch.csv" 2>/dev/null || echo 0)줄"
  echo "  서버 왕복: $(tail -1 "$D/server_rtt.csv")"
  echo "  Wi-Fi:    $(tail -1 "$D/wifi.csv")"
  ;;
stop)
  [ -s "$PIDS" ] || { echo "켜진 기록 없음"; exit 0; }
  # 켠 PID와 그 자손 전부(PPid를 따라). 그룹째로 보내면 안 된다 — record_bag.sh 안의 timeout이 자기 그룹을 따로 만들어
  # 녹화가 신호를 못 받고 계속 돌았다(11-22 시험). 먼저 INT(녹화가 파일을 닫게), 5초 뒤 남은 것에 TERM
  tree() { local all; all=$(ps -eo pid=,ppid=); local q; q=$(cat "$PIDS"); local out="$q"
    while [ -n "$q" ]; do q=$(echo "$all" | awk -v s=" $(echo $q) " 'index(s, " "$2" ") {print $1}'); out="$out $q"; done; echo $out; }
  T=$(tree)
  kill -INT $T 2>/dev/null || true
  sleep 5
  kill -TERM $T 2>/dev/null || true
  sleep 1
  # 남았는지는 위에서 모은 PID로만 본다(이름으로 찾으면 이 스크립트를 부른 셸까지 걸린다)
  left=""; for p in $T; do kill -0 "$p" 2>/dev/null && left="$left $p"; done
  [ -n "$left" ] && { echo "남은 프로세스:"; ps -o pid=,args= -p ${left// /,} | cut -c1-120; } || echo "기록 모두 멈춤 ($(cat "$PIDS.dir" 2>/dev/null))"
  rm -f "$PIDS" "$PIDS.dir"
  ;;
*) sed -n 2,14p "$0"; exit 1 ;;
esac
