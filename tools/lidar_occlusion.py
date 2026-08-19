#!/usr/bin/env python3
"""라이다 자기 차폐를 각도별로 측정한다. **재배치할 때마다 돌릴 것.**

왜 도구로 만들었나
------------------
라이다를 옮길 때마다 같은 분석을 반복했다(상판->하판, 방향 교정, 2026-08-19 USB
이전+전방 이동). 매번 즉석 스크립트를 쓰다 보니 결과 형식이 달라 **이전과 비교가
안 됐다.** 같은 잣대로 재야 개선 여부를 말할 수 있다.

무엇을 재나
-----------
로봇 자기 구조물(케이블, 프레임, 바퀴)은 **로봇 좌표계에 고정**돼 있으므로
같은 각도에서 항상 같은 거리로 관측되고 **분산이 매우 작다.** 환경은 그렇지 않다.
이 성질로 자기 구조물을 골라낸다.

  판정: 그 각도 빈에서 (관측 빈도가 높음) AND (거리가 짧음) AND (표준편차가 작음)

⚠️ 로봇이 **정지**해 있어야 한다. 움직이면 환경도 "안 변하는 것처럼" 보이지 않아
   판정이 흐려진다. 반대로 회전시키면 자기 구조물만 남으므로 더 확실하지만,
   그건 모터 승인이 필요하다.

사용법
------
    python3 tools/lidar_occlusion.py                 # /scan 15초 관측
    python3 tools/lidar_occlusion.py --topic /scan_raw --sec 20
    python3 tools/lidar_occlusion.py --json out.json # 이력 비교용 저장
"""

import argparse
import json
import math
import sys
import time

import rclpy
from sensor_msgs.msg import LaserScan

BIN_DEG = 2.0
# 자기 구조물 판정 기준
MAX_SELF_RANGE = 0.25     # m — 이보다 멀면 구조물로 안 본다
MAX_SELF_STD = 0.02       # m — 이보다 흔들리면 환경
MIN_PRESENCE = 0.5        # 그 각도에서 관측된 스캔 비율


def collect(topic, seconds):
    rclpy.init()
    node = rclpy.create_node('lidar_occlusion')
    scans = []
    node.create_subscription(LaserScan, topic, lambda m: scans.append(m), 20)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return scans


def analyze(scans):
    bins = {}
    for m in scans:
        a = m.angle_min
        for r in m.ranges:
            deg = math.degrees(a) % 360.0
            a += m.angle_increment
            if math.isfinite(r) and r > 0:
                bins.setdefault(int(deg // BIN_DEG), []).append(r)

    n = len(scans)
    self_bins = {}
    for b, vals in bins.items():
        if len(vals) < n * MIN_PRESENCE:
            continue
        vals = sorted(vals)
        med = vals[len(vals) // 2]
        mean = sum(vals) / len(vals)
        sd = (sum((x - mean) ** 2 for x in vals) / len(vals)) ** 0.5
        if med <= MAX_SELF_RANGE and sd <= MAX_SELF_STD:
            self_bins[b] = {'median': med, 'std': sd,
                            'lo': vals[0], 'hi': vals[-1]}
    return bins, self_bins


def sectors(self_bins):
    """연속된 차폐 빈을 구간으로 묶는다(360도 경계를 감안)."""
    if not self_bins:
        return []
    keys = sorted(self_bins)
    groups = []
    cur = [keys[0]]
    for k in keys[1:]:
        if k == cur[-1] + 1:
            cur.append(k)
        else:
            groups.append(cur)
            cur = [k]
    groups.append(cur)
    # 0도와 360도에서 이어지면 합친다
    total = int(360 / BIN_DEG)
    if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == total - 1:
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    return groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--sec', type=float, default=15.0)
    ap.add_argument('--json', help='결과를 JSON으로 저장(이력 비교용)')
    args = ap.parse_args()

    scans = collect(args.topic, args.sec)
    if not scans:
        print(f'{args.topic} 수신 실패'); return 1

    bins, self_bins = analyze(scans)
    groups = sectors(self_bins)

    blocked_deg = len(self_bins) * BIN_DEG
    print(f'스캔 {len(scans)}개, 각도 빈 {BIN_DEG:.0f}도, 토픽 {args.topic}\n')
    print(f'  총 차폐 {blocked_deg:.0f}도 / 360도  ({100*blocked_deg/360:.1f}%)')

    # 정면 +-30도 차폐 (주행 방향이라 제일 중요)
    front = [b for b in self_bins
             if min((b * BIN_DEG) % 360, 360 - (b * BIN_DEG) % 360) <= 30]
    print(f'  정면 ±30도 차폐 {len(front)*BIN_DEG:.0f}도 / 60도  '
          f'({100*len(front)*BIN_DEG/60:.1f}%)')
    print(f'  차폐 구간 {len(groups)}개\n')

    if groups:
        print(f'  {"구간":>16} {"폭":>6} {"거리(중앙)":>11} {"표준편차":>9}')
        for g in groups:
            a0 = (g[0] * BIN_DEG) % 360
            a1 = ((g[-1] + 1) * BIN_DEG) % 360
            meds = [self_bins[b]['median'] for b in g]
            sds = [self_bins[b]['std'] for b in g]
            print(f'  {a0:>6.0f}~{a1:<6.0f}도 {len(g)*BIN_DEG:>5.0f}도 '
                  f'{sum(meds)/len(meds):>10.3f}m {max(sds):>8.4f}')
    else:
        print('  자기 구조물 차폐 없음')

    # 폴리곤 자기필터(차체 ±0.105)와의 관계
    inside = [b for b in self_bins if self_bins[b]['hi'] < 0.105]
    outside = [b for b in self_bins if self_bins[b]['lo'] >= 0.105]
    straddle = len(self_bins) - len(inside) - len(outside)
    print(f'\n  폴리곤 필터(차체 ±0.105m) 기준:')
    print(f'    완전히 안쪽(제거됨)  {len(inside)*BIN_DEG:>4.0f}도')
    print(f'    걸쳐 있음(일부 잔존) {straddle*BIN_DEG:>4.0f}도')
    print(f'    바깥(제거 안 됨)     {len(outside)*BIN_DEG:>4.0f}도')
    if outside:
        print('    ⚠️ 폴리곤 밖의 구조물이 있다 — 필터가 못 지운다.')
        print('       라이다가 차체 중심에서 벗어났는데 TF x/y 오프셋이 0이면 이렇게 된다.')

    if args.json:
        json.dump({'topic': args.topic, 'scans': len(scans),
                   'blocked_deg': blocked_deg,
                   'front30_deg': len(front) * BIN_DEG,
                   'sectors': [{'from': (g[0]*BIN_DEG) % 360,
                                'to': ((g[-1]+1)*BIN_DEG) % 360,
                                'width': len(g)*BIN_DEG,
                                'median_m': sum(self_bins[b]['median'] for b in g)/len(g)}
                               for g in groups]},
                  open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n  저장: {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
