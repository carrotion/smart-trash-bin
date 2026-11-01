#!/usr/bin/env python3
"""자기 구조물 모서리의 허상 반사 — 지우는 쐐기가 무엇을 바꾸는지 녹화로 판정한다 (2026-11-01).

사고: 주행기가 "앞 여유 0"으로 멈춘 실물 9건 중 7건에서 막은 점이 차체 좌표 거의 같은 자리 (0.11, ±0.145)였다.
정지 스캔으로 확인: 자기 구조물(8 cm)과 뒤쪽 벽 사이에 걸친 광선이 중간값(11~16 cm)을 낸다. 실제 물체가 아니다.
표준 ScanShadowsFilter는 먼 곳 노이즈까지 지워 스캔 점의 33%를 잃어 버렸다 → config/scan_self_filter.yaml의 좁은 쐐기로 지운다.
이 도구: 쐐기를 /scan_raw에 적용해 앞 여유(주행기와 같은 함수)가 어떻게 바뀌는지 본다. 진짜 물체(병 녹화)로 막힌 프레임은 그대로 막혀 있어야 한다.

    python3 tools/scan_ghost_eval.py bags/A bags/B --every 3
"""
import argparse, os, sys, types

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance

LX = 0.054
WEDGES = ((49, 53), (65, 70), (289, 292), (305, 309))   # 허상이 나오는 방향(도, 0=앞·90=왼쪽)


def wedges(r, ang):
    """config/scan_self_filter.yaml의 쐐기와 같은 범위: 라이다에서 18 cm 안, 네 좁은 각도."""
    deg = np.degrees(ang) % 360; o = r.copy()
    for a, b in WEDGES:
        o[(deg >= a) & (deg <= b) & (r <= 0.18)] = np.nan
    return o


def polygon(r, ang):
    x = LX + r * np.cos(ang); y = r * np.sin(ang); o = r.copy()
    o[(np.abs(x) <= 0.105) & (np.abs(y) <= 0.105)] = np.nan
    return o


def to_scan(m, r):
    return types.SimpleNamespace(ranges=np.nan_to_num(r, nan=float('inf')).tolist(), angle_min=m.angle_min, angle_increment=m.angle_increment, range_min=m.range_min, range_max=m.range_max)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bags', nargs='+'); ap.add_argument('--every', type=int, default=1); ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    for bag in a.bags:
        sid = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bag)) else 'sqlite3'
        rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        rd.set_filter(rosbag2_py.StorageFilter(topics=['/scan_raw']))
        S = dict(n=0, pts=0, rm=0, tight0=0, tight1=0, freed=0, keep=0); ex = []; k = 0
        while rd.has_next():
            _, data, _ = rd.read_next(); k += 1
            if k % a.every:
                continue
            m = deserialize_message(data, LaserScan); r = np.array(m.ranges, dtype=float); r[(r < m.range_min) | (r > m.range_max)] = np.nan
            ang = m.angle_min + np.arange(len(r)) * m.angle_increment
            before = polygon(r, ang); after = wedges(before, ang)
            S['n'] += 1; S['pts'] += int(np.isfinite(before).sum()); S['rm'] += int((np.isfinite(before) & ~np.isfinite(after)).sum())
            c0 = corridor_clearance(to_scan(m, before), True, 0.0, 0.05); c1 = corridor_clearance(to_scan(m, after), True, 0.0, 0.05)
            S['tight0'] += c0 < 0.30; S['tight1'] += c1 < 0.30
            if c0 < 0.30 and c1 < 0.30:
                S['keep'] += 1
            if c0 < 0.30 and c1 >= 0.30:
                S['freed'] += 1; ex.append((k, round(c0, 2), round(c1, 2)))
        print(f'{os.path.basename(bag)}: {S["n"]}장, 삭제 {S["rm"] / max(S["pts"], 1) * 100:.3f}% | 앞 여유 0.30 미만 {S["tight0"]} → {S["tight1"]}장(풀림 {S["freed"]}, 유지 {S["keep"]})', flush=True)
        for o in (ex if a.verbose else ex[:8]):
            print('   풀린 장(프레임, 전, 후):', o)
    return 0


if __name__ == '__main__':
    sys.exit(main())
