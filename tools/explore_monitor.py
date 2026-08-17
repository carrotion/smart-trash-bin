#!/usr/bin/env python3
"""자율탐색 주행을 정량 기록한다.

왜 필요한가
-----------
2026-08-17 explore 목표점 선정을 고쳤다(비용과 목표가 같은 점을 가리키도록).
이게 실제로 "제자리 회전 폭주"를 줄였는지 **숫자로** 확인해야 한다. 안 그러면
"좋아진 것 같다"로 끝나고, 다음에 또 같은 논쟁을 한다.

기록하는 것:
  - 순수 회전 비율: |v|이 거의 0인데 |w|만 있는 시간의 비율 (기존 실측 45~71%)
  - 목표 변경 횟수와 간격: 목표가 튀면 짧은 간격으로 계속 바뀐다
  - 맵 면적 증가: 실제로 탐색이 진행되는지
  - 이동 거리: 회전만 하고 안 움직이는지

사용법
------
    python3 tools/explore_monitor.py --out /tmp/run.json --interval 15
"""

import argparse
import json
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# 이 값보다 작으면 "정지"로 본다. min_turn_pwm 부근에서 실제로 안 도는 구간이 있어
# 명령 기준으로 판단한다(실제 회전 여부는 자이로로 따로 봐야 한다).
LIN_EPS = 0.01   # m/s
ANG_EPS = 0.05   # rad/s


class Monitor(Node):
    def __init__(self):
        super().__init__('explore_monitor')
        self.t0 = time.time()
        self.cmd = []           # (t, v, w)
        self.goals = []         # (t, x, y)
        self.area = []          # (t, m^2)
        self.map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            # SLAM Toolbox의 /map은 transient_local로 나간다. 안 맞추면 조용히 못 받는다.
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 20)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, self.map_qos)
        # explore가 Nav2로 보내는 목표. 액션 내부라 직접은 못 보지만, bt_navigator가
        # 받은 목표를 이 토픽으로 다시 낸다.
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)

    def _on_cmd(self, m):
        self.cmd.append((time.time(), m.linear.x, m.angular.z))

    def _on_goal(self, m):
        self.goals.append((time.time(), m.pose.position.x, m.pose.position.y))

    def _on_map(self, m):
        known = sum(1 for v in m.data if v >= 0)
        self.area.append((time.time(), known * m.info.resolution ** 2))


def summarize(mon, elapsed):
    out = {'elapsed_s': elapsed}
    c = mon.cmd
    if c:
        rot = sum(1 for _, v, w in c if abs(v) < LIN_EPS and abs(w) >= ANG_EPS)
        fwd = sum(1 for _, v, w in c if abs(v) >= LIN_EPS)
        idle = len(c) - rot - fwd
        out['cmd_n'] = len(c)
        out['pure_rotation_pct'] = 100.0 * rot / len(c)
        out['translating_pct'] = 100.0 * fwd / len(c)
        out['idle_pct'] = 100.0 * idle / len(c)
        # 명령상 이동거리(실제 이동이 아니라 의도한 이동량)
        dist = 0.0
        for i in range(1, len(c)):
            dt = c[i][0] - c[i - 1][0]
            if 0 < dt < 1.0:
                dist += abs(c[i - 1][1]) * dt
        out['commanded_distance_m'] = dist
    if mon.goals:
        g = mon.goals
        out['goal_count'] = len(g)
        gaps = [g[i][0] - g[i - 1][0] for i in range(1, len(g))]
        if gaps:
            gaps_sorted = sorted(gaps)
            out['goal_interval_median_s'] = gaps_sorted[len(gaps_sorted) // 2]
            out['goal_interval_min_s'] = gaps_sorted[0]
        # 연속한 두 목표 사이 거리 — 작으면 같은 자리에서 목표가 튀는 것
        jumps = [math.hypot(g[i][1] - g[i - 1][1], g[i][2] - g[i - 1][2])
                 for i in range(1, len(g))]
        if jumps:
            out['goal_jump_mean_m'] = sum(jumps) / len(jumps)
    if mon.area:
        out['map_area_m2'] = mon.area[-1][1]
        out['map_area_start_m2'] = mon.area[0][1]
    return out


def fmt(s):
    p = [f"{s['elapsed_s']:5.0f}초"]
    if 'pure_rotation_pct' in s:
        p.append(f"순수회전 {s['pure_rotation_pct']:5.1f}%")
        p.append(f"이동 {s['translating_pct']:5.1f}%")
        p.append(f"명령거리 {s['commanded_distance_m']:5.2f}m")
    if 'map_area_m2' in s:
        p.append(f"맵 {s['map_area_m2']:6.1f}m²")
    if 'goal_count' in s:
        p.append(f"목표 {s['goal_count']}개")
        if 'goal_interval_median_s' in s:
            p.append(f"(간격중앙 {s['goal_interval_median_s']:.1f}s)")
    return '  '.join(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/explore_run.json')
    ap.add_argument('--interval', type=float, default=15.0)
    ap.add_argument('--duration', type=float, default=1800.0)
    args = ap.parse_args()

    rclpy.init()
    mon = Monitor()
    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: stop.update(v=True))

    next_report = time.time() + args.interval
    end = time.time() + args.duration
    history = []
    while time.time() < end and not stop['v']:
        rclpy.spin_once(mon, timeout_sec=0.1)
        if time.time() >= next_report:
            next_report += args.interval
            s = summarize(mon, time.time() - mon.t0)
            history.append(s)
            print(fmt(s), flush=True)
            json.dump({'history': history}, open(args.out, 'w'),
                      ensure_ascii=False, indent=1)

    s = summarize(mon, time.time() - mon.t0)
    history.append(s)
    json.dump({'history': history, 'final': s}, open(args.out, 'w'),
              ensure_ascii=False, indent=1)
    print('\n=== 최종 ===')
    for k, v in s.items():
        print(f'  {k}: {v:.2f}' if isinstance(v, float) else f'  {k}: {v}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
