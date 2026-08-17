#!/usr/bin/env bash
# 알고리즘 검증용 rosbag 녹화.
#
# 왜 필요한가
# -----------
# 2026-08-17까지 우리는 프론티어 비용함수·필터 같은 **로직**을 실물 로봇으로 검증했다.
# 1회에 수십 분이 들고 로봇이 벽을 밀어 사고 위험까지 생겼다. 같은 검증을 녹화본으로
# 하면 1회가 몇 초고 위험이 0이며, **결정론적이라 비교가 가능**하다.
#
# 역할 분담(CLAUDE.md 참고):
#   로직(목표 선정, 비용함수, 필터 기하) -> 이 녹화본 재생으로 검증
#   물리(미끄러짐, 자기 차폐, 스캔 왜곡, 전력) -> 실물로만 측정
#
# ⚠️ scan_raw를 반드시 같이 담는다. /scan만 담으면 **필터를 바꿔가며 비교할 수 없다**
#    (이미 걸러진 결과만 남으므로). 원본이 있어야 필터 A/B 비교가 된다.
#
# 사용법
# ------
#   tools/record_bag.sh [이름] [초]
#     예: tools/record_bag.sh push_around 180
#
# 모터를 쓰지 않고 데이터를 모으는 방법: **로봇을 손으로 밀고 다닌다.**
# 바퀴가 구르면 엔코더가 세므로 오도메트리도 같이 남는다. 승인도 필요 없고 안전하다.
set -eo pipefail

NAME="${1:-run}"
DURATION="${2:-120}"
OUTDIR="${BAG_DIR:-$HOME/smart-trash-bin/bags}"
STAMP=$(date +%Y%m%d_%H%M%S)
DEST="$OUTDIR/${STAMP}_${NAME}"

# ⚠️ `set -u`를 쓰면 안 된다. ROS의 setup.bash가 미정의 변수
# (AMENT_TRACE_SETUP_FILES 등)를 참조해서 그 자리에서 죽는다.
source /opt/ros/jazzy/setup.bash
source "$HOME/smart-trash-bin/robot/install/setup.bash"

TOPICS=(
  /scan_raw          # 원본 — 필터 A/B 비교에 필수
  /scan              # 현재 필터 적용 결과
  /map               # SLAM 결과
  /odom              # 휠 오도메트리
  /odometry/filtered # EKF 융합 결과
  /imu/data
  /imu/mag
  /wheel_encoders
  /tf
  /tf_static
  /cmd_vel
)

# ⚠️ 구동 계통 선행 확인. 2026-08-17에 정리 스크립트가 diff_drive_node를 죽이면서
# systemd 서비스까지 내려갔는데 모르고 녹화를 시작했다 — 사용자가 조종해도 로봇이
# 안 움직였고 /odom도 가방에서 빠졌다. 녹화 전에 반드시 확인한다.
echo "구동 계통 확인:"
DEAD=""
for svc in smart-trash-diff-drive smart-trash-ekf smart-trash-imu; do
  st=$(systemctl is-active "$svc" 2>/dev/null)
  printf "  %-26s %s\n" "$svc" "$st"
  [ "$st" != "active" ] && DEAD="$DEAD $svc"
done
if [ -n "$DEAD" ]; then
  echo
  echo "❌ 꺼진 서비스가 있다:$DEAD"
  echo "   이대로 녹화하면 조종해도 로봇이 안 움직이고 오도메트리도 안 남는다."
  echo "   먼저 실행할 것:  sudo systemctl start$DEAD"
  exit 1
fi
echo

echo "녹화 대상:"
MISSING=0
for t in "${TOPICS[@]}"; do
  if ros2 topic info "$t" >/dev/null 2>&1; then
    printf "  %-20s OK\n" "$t"
  else
    printf "  %-20s ⚠️ 없음\n" "$t"
    MISSING=$((MISSING+1))
  fi
done
if [ "$MISSING" -gt 0 ]; then
  echo
  echo "⚠️ 없는 토픽이 $MISSING개다. 빠진 채로 녹화하면 나중에 재생이 반쪽이 된다."
  echo "   mapping.launch.py와 diff-drive/EKF 서비스가 떠 있는지 확인할 것."
fi

mkdir -p "$OUTDIR"
echo
echo "녹화 시작 -> $DEST  (${DURATION}초)"
echo "지금부터 로봇을 손으로 밀고 다니면 실제 주행 데이터가 남는다."

# mcap: 기본 sqlite3보다 쓰기가 가볍고 파일이 작다. SD 수명에도 유리하다.
#
# ⚠️ `ros2 bag record &` 뒤에 `kill -INT $PID`를 쓰면 안 된다. `ros2`는 파이썬
#    런처라서 실제 녹화 프로세스가 자식으로 뜨고, 런처에 보낸 INT가 거기까지 안 간다
#    (실측: 20초를 줬는데 110초 녹화됨). `timeout -s INT`로 **포그라운드에서** 돌려야
#    프로세스 그룹에 신호가 전달되고 가방이 깨끗하게 닫힌다.
timeout -s INT "$DURATION" \
  ros2 bag record -s mcap -o "$DEST" --max-cache-size 20971520 "${TOPICS[@]}" \
  || true    # timeout이 보낸 INT는 정상 종료다 — 비정상 취급하지 않는다

echo
ros2 bag info "$DEST" 2>/dev/null | sed 's/^/  /'
echo
echo "재생: ros2 bag play $DEST"
