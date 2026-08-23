#!/usr/bin/env python3
"""탐색 진행률을 "완료 선언" 대신 실측 맵 커버리지(%)로 추적한다.

왜 필요한가
----------
explore_lite가 home_real에서 3번 실패하자마자 "All frontiers traversed"를
선언한 사건(2026-08-23) — 맵을 거의 안 그린 채로 "완료"라고 찍었다. 로그의
"stopping." 문자열만 보고 완료 여부를 판단하면 이런 조기 오판을 그대로
지표로 믿게 된다. /map 토픽에서 직접 known(free+occupied)/total 셀 비율을
주기적으로 재서 CSV에 남기면, 자기 신고(self-report)가 아니라 실측으로
"실제로 얼마나 탐색했는가"를 시간에 따라 비교할 수 있다.

사용법
------
    python3 tools/map_coverage_logger.py --out coverage.csv
    python3 tools/map_coverage_logger.py --out coverage.csv --interval 3 --duration 400
    python3 tools/map_coverage_logger.py --out coverage.csv --sim-time   # 가방 재생/시뮬 중

CSV 컬럼: wall_epoch, sim_time_sec, width, height, resolution,
          known_cells, free_cells, occupied_cells, total_cells, coverage_pct
"""
import argparse
import csv
import signal
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/map')
    ap.add_argument('--out', required=True, help='CSV 출력 경로')
    ap.add_argument('--interval', type=float, default=5.0,
                     help='기록 주기(초, 벽시계 기준) — 기본 5초')
    ap.add_argument('--duration', type=float, default=None,
                     help='이 시간(초) 지나면 자동 종료 — 미지정 시 SIGINT까지 계속')
    ap.add_argument('--occ-threshold', type=int, default=65,
                     help='이 값 이상이면 occupied로 침(0~100, nav2 관례와 동일)')
    ap.add_argument('--sim-time', action='store_true',
                     help='시뮬레이션/가방재생 중이면 지정 (use_sim_time=true)')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('map_coverage_logger')
    if args.sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    latest = {}

    def on_map(msg):
        latest['msg'] = msg

    node.create_subscription(OccupancyGrid, args.topic, on_map, qos)

    f = open(args.out, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(['wall_epoch', 'sim_time_sec', 'width', 'height', 'resolution',
                      'known_cells', 'free_cells', 'occupied_cells', 'total_cells',
                      'coverage_pct'])
    f.flush()

    stop = {'flag': False}

    def handle_sigint(signum, frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    start_wall = time.time()
    next_log = start_wall
    rows_written = 0

    print(f'[map_coverage_logger] {args.topic} 구독 시작 -> {args.out} '
          f'(주기 {args.interval}s, occ_threshold={args.occ_threshold})', flush=True)

    while not stop['flag']:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.time()
        if args.duration is not None and now - start_wall >= args.duration:
            break
        if now < next_log:
            continue
        next_log = now + args.interval

        msg = latest.get('msg')
        if msg is None:
            continue

        data = msg.data
        total = len(data)
        unknown = 0
        occupied = 0
        for v in data:
            if v < 0:
                unknown += 1
            elif v >= args.occ_threshold:
                occupied += 1
        known = total - unknown
        free = known - occupied
        coverage_pct = (100.0 * known / total) if total else 0.0

        sim_time_sec = (msg.header.stamp.sec
                         + msg.header.stamp.nanosec * 1e-9)

        writer.writerow([f'{now:.3f}', f'{sim_time_sec:.3f}',
                          msg.info.width, msg.info.height, msg.info.resolution,
                          known, free, occupied, total, f'{coverage_pct:.3f}'])
        f.flush()
        rows_written += 1
        print(f'  t={now - start_wall:6.1f}s  coverage={coverage_pct:5.1f}%  '
              f'known={known}/{total}', flush=True)

    f.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f'[map_coverage_logger] 종료 — {rows_written}행 기록됨: {args.out}', flush=True)


if __name__ == '__main__':
    main()
