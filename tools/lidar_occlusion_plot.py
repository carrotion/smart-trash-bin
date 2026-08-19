#!/usr/bin/env python3
"""라이다 차폐를 위에서 본 그림으로 그린다.

숫자표만으로는 "어디가 막혔나"가 안 잡힌다. 재배치할 때마다 그려서 이전 그림과
비교하면 무엇이 좋아지고 나빠졌는지 바로 보인다.

좌표 규약 (ROS REP-103): 0도 = 로봇 정면, +90도 = 왼쪽, 180도 = 후방.
그림은 **위에서 내려다본 것**이고 로봇 정면이 위쪽이다.

    python3 tools/lidar_occlusion_plot.py --sec 15 --out occ.png
"""

import argparse
import math
import sys
import time

from PIL import Image, ImageDraw

import rclpy
from sensor_msgs.msg import LaserScan

BIN_DEG = 2.0
MAX_SELF_RANGE = 0.25
MAX_SELF_STD = 0.02
MIN_PRESENCE = 0.5

SIZE = 900
CX = CY = SIZE // 2
# 그림 반경 0.45m를 화면 반경으로
VIEW_M = 0.45
PPM = (SIZE * 0.40) / VIEW_M          # 픽셀/미터

BG = (255, 255, 255)
GRID = (215, 215, 215)
CHASSIS = (60, 60, 60)
BLOCK = (220, 60, 60)
BLOCK_FILL = (250, 205, 205)
ODD = (235, 145, 30)
TEXT = (30, 30, 30)
FREE = (120, 190, 120)


def to_px(x, y):
    """로봇 좌표(m) -> 화면 픽셀. x=전방(화면 위), y=좌측(화면 왼쪽)."""
    return (CX - y * PPM, CY - x * PPM)


def collect(topic, seconds):
    rclpy.init()
    node = rclpy.create_node('occ_plot')
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
    env_median = {}
    for b, vals in bins.items():
        vals = sorted(vals)
        med = vals[len(vals) // 2]
        env_median[b] = med
        if len(vals) < n * MIN_PRESENCE:
            continue
        mean = sum(vals) / len(vals)
        sd = (sum((x - mean) ** 2 for x in vals) / len(vals)) ** 0.5
        if med <= MAX_SELF_RANGE and sd <= MAX_SELF_STD:
            self_bins[b] = {'median': med, 'std': sd}
    return self_bins, env_median


def draw(self_bins, env_median, out_path, lidar_x=0.0):
    img = Image.new('RGB', (SIZE, SIZE), BG)
    d = ImageDraw.Draw(img)

    # 거리 눈금 원
    for r_m in (0.1, 0.2, 0.3, 0.4):
        rp = r_m * PPM
        d.ellipse([CX - rp, CY - rp, CX + rp, CY + rp], outline=GRID)
        d.text((CX + 4, CY - rp - 14), f'{r_m:.1f}m', fill=(150, 150, 150))

    # 각도 기준선
    for a in range(0, 360, 30):
        rad = math.radians(a)
        x, y = math.cos(rad) * VIEW_M, math.sin(rad) * VIEW_M
        d.line([to_px(0, 0), to_px(x, y)], fill=GRID)
        lx, ly = math.cos(rad) * (VIEW_M * 0.97), math.sin(rad) * (VIEW_M * 0.97)
        px, py = to_px(lx, ly)
        d.text((px - 10, py - 6), f'{a}°', fill=(140, 140, 140))

    # 차폐 구간을 부채꼴로
    for b, info in self_bins.items():
        a0 = b * BIN_DEG
        a1 = a0 + BIN_DEG
        col = ODD if info['median'] > 0.15 else BLOCK
        fill = (250, 225, 190) if info['median'] > 0.15 else BLOCK_FILL
        pts = [to_px(0, 0)]
        steps = 4
        for i in range(steps + 1):
            a = math.radians(a0 + (a1 - a0) * i / steps)
            rr = min(info['median'], VIEW_M)
            pts.append(to_px(math.cos(a) * rr, math.sin(a) * rr))
        d.polygon(pts, fill=fill, outline=col)

    # 환경 관측(참고): 막히지 않은 방향이 얼마나 멀리 보이는지
    for b, med in env_median.items():
        if b in self_bins:
            continue
        a = math.radians(b * BIN_DEG + BIN_DEG / 2)
        rr = min(med, VIEW_M)
        x0, y0 = math.cos(a) * min(rr, VIEW_M * 0.98), math.sin(a) * min(rr, VIEW_M * 0.98)
        px, py = to_px(x0, y0)
        d.ellipse([px - 2, py - 2, px + 2, py + 2], fill=FREE)

    # 차체 사각형 (20x20cm). 라이다가 앞으로 lidar_x 만큼 이동했으면
    # 라이다 기준으로 차체는 뒤로 그만큼 밀린다.
    half = 0.10
    corners = [(half - lidar_x, half), (half - lidar_x, -half),
               (-half - lidar_x, -half), (-half - lidar_x, half)]
    d.polygon([to_px(x, y) for x, y in corners], outline=CHASSIS)
    # 폴리곤 자기필터 경계 (±0.105)
    pf = 0.105
    pc = [(pf - lidar_x, pf), (pf - lidar_x, -pf),
          (-pf - lidar_x, -pf), (-pf - lidar_x, pf)]
    d.polygon([to_px(x, y) for x, y in pc], outline=(80, 120, 220))

    # 차폐 구간마다 각도/거리 라벨 (어디가 뭔지 바로 읽히게)
    from itertools import groupby
    keys = sorted(self_bins)
    runs = []
    for _, g in groupby(enumerate(keys), lambda t: t[1] - t[0]):
        gg = [k for _, k in g]
        runs.append(gg)
    for g in runs:
        mid = math.radians((g[0] + (len(g) / 2.0)) * BIN_DEG)
        med = sum(self_bins[b]['median'] for b in g) / len(g)
        rr = min(max(med, 0.13), VIEW_M * 0.9) + 0.045
        px, py = to_px(math.cos(mid) * rr, math.sin(mid) * rr)
        col = ODD if med > 0.15 else BLOCK
        d.text((px - 26, py - 6),
               f'{g[0]*BIN_DEG:.0f}-{(g[-1]+1)*BIN_DEG:.0f}d {med*100:.0f}cm', fill=col)

    # 라이다 위치와 정면 화살표
    px, py = to_px(0, 0)
    d.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(40, 40, 40))
    d.polygon([to_px(0.06, 0), to_px(0.035, 0.018), to_px(0.035, -0.018)],
              fill=(40, 120, 40))

    # 범례
    y = 12
    # ⚠️ 라벨은 반드시 ASCII로. 이 시스템에는 한글 폰트가 없어서 한글을 쓰면
    #    PIL이 .notdef 네모(□□□)로 그린다 — 그림이 읽을 수 없게 된다.
    for txt, col in (('RED  = self-structure blockage (<0.15m)', BLOCK),
                     ('ORANGE = distant fixed return (>0.15m) - identify!', ODD),
                     ('BLACK box = chassis 20x20cm', CHASSIS),
                     ('BLUE box  = self-filter polygon +-0.105m', (80, 120, 220)),
                     ('GREEN dot = open-direction range', FREE),
                     ('GREEN tri = robot FRONT (0 deg)', (40, 120, 40))):
        d.rectangle([10, y, 22, y + 10], fill=col)
        d.text((28, y - 1), txt, fill=TEXT)
        y += 16
    if lidar_x:
        d.text((10, y + 4),
               f'LiDAR is {lidar_x*1000:.0f}mm FORWARD of chassis center',
               fill=TEXT)

    img.save(out_path)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--sec', type=float, default=15.0)
    ap.add_argument('--out', default='/tmp/lidar_occlusion.png')
    ap.add_argument('--lidar-x', type=float, default=0.0,
                    help='라이다가 차체 중심보다 전방으로 이동한 거리(m)')
    args = ap.parse_args()

    scans = collect(args.topic, args.sec)
    if not scans:
        print('스캔 수신 실패'); return 1
    self_bins, env_median = analyze(scans)
    p = draw(self_bins, env_median, args.out, args.lidar_x)
    print(f'스캔 {len(scans)}개 -> {p}')
    print(f'차폐 {len(self_bins)*BIN_DEG:.0f}도')
    return 0


if __name__ == '__main__':
    sys.exit(main())
