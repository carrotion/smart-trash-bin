#!/usr/bin/env python3
"""로봇을 손으로 한 바퀴 돌리며 **자기센서 보정 + 라이다 자기구조물 판별**을 동시에 한다.

왜 회전이어야 하나
------------------
정지 상태에서는 **환경이 자기 구조물인 척한다.** 벽도 소파도 안 움직이니 "같은
각도에서 항상 같은 거리"라는 조건을 똑같이 만족한다. 실제로 2026-08-19 측정에서
도크 충전 케이블과 소파를 자기 구조물로 의심했다.

회전시키면 갈린다:
  - **자기 구조물**: 로봇 좌표계에 고정 -> 각도·거리가 그대로
  - **환경**: 로봇이 돌면 로봇 기준 각도가 변함 -> 그 각도 빈의 거리가 요동

같은 회전이 자기센서 하드아이언 보정도 해준다. (Bx,By)가 그리는 원의 중심이 편향값,
반지름이 지구 자기장 수평 성분(한국 약 30µT), 잔차가 그 편향의 **안정성**이다.

⚠️ **손으로** 돌릴 것. 모터를 돌리면 모터 자체가 자기장을 만들어 보정을 오염시킨다.
⚠️ **주변이 트인 곳**에서. 좁은 곳이면 환경이 계속 가까이 있어 판별력이 떨어진다.
⚠️ **수평 유지.** 기울면 (Bx,By) 원이 타원으로 찌그러진다.

사용법
------
    python3 tools/hand_rotation_check.py --sec 60
    (기록이 시작되면 제자리에서 천천히 2바퀴)
"""

import argparse
import json
import math
import sys
import time

import rclpy
from sensor_msgs.msg import Imu, LaserScan, MagneticField

BIN_DEG = 2.0


def fit_circle(pts):
    """Kasa 대수 원 적합. 반환 (cx, cy, r). 반복 최적화가 없어 안정적이다."""
    n = len(pts)
    sx = sy = sxx = syy = sxy = sz = szx = szy = 0.0
    for x, y in pts:
        z = x * x + y * y
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y
        sz += z; szx += z * x; szy += z * y
    a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
    b = [-szx, -szy, -sz]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(a[r][i]))
        if abs(a[p][i]) < 1e-12:
            return None
        a[i], a[p] = a[p], a[i]
        b[i], b[p] = b[p], b[i]
        for r in range(i + 1, 3):
            f = a[r][i] / a[i][i]
            for c in range(i, 3):
                a[r][c] -= f * a[i][c]
            b[r] -= f * b[i]
    sol = [0.0] * 3
    for i in (2, 1, 0):
        sol[i] = (b[i] - sum(a[i][c] * sol[c] for c in range(i + 1, 3))) / a[i][i]
    d, e, f = sol
    cx, cy = -d / 2.0, -e / 2.0
    disc = cx * cx + cy * cy - f
    return (cx, cy, math.sqrt(disc)) if disc > 0 else None


def record(seconds):
    rclpy.init()
    node = rclpy.create_node('hand_rotation_check')
    mag, scans, gyro = [], [], []
    node.create_subscription(MagneticField, '/imu/mag',
                             lambda m: mag.append((m.magnetic_field.x * 1e6,
                                                   m.magnetic_field.y * 1e6,
                                                   m.magnetic_field.z * 1e6)), 50)
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), 30)
    node.create_subscription(Imu, '/imu/data',
                             lambda m: gyro.append((time.time(),
                                                    m.angular_velocity.z)), 50)
    end = time.time() + seconds
    last = 0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        left = int(end - time.time())
        if left != last and left % 10 == 0:
            print(f'  ... {left}초 남음  (자기 {len(mag)}, 스캔 {len(scans)})',
                  flush=True)
            last = left
    node.destroy_node()
    rclpy.shutdown()
    return mag, scans, gyro


def report_rotation(gyro):
    """자이로 적분으로 실제로 몇 바퀴 돌았는지."""
    if len(gyro) < 2:
        return None
    total = 0.0
    for i in range(1, len(gyro)):
        dt = gyro[i][0] - gyro[i - 1][0]
        if 0 < dt < 0.2:
            total += gyro[i][1] * dt
    return math.degrees(total)


def report_mag(mag):
    print('\n=== 자기센서 하드아이언 보정 ===')
    if len(mag) < 100:
        print(f'  샘플 부족({len(mag)})'); return None
    pts = [(m[0], m[1]) for m in mag]
    fit = fit_circle(pts)
    if not fit:
        print('  원 적합 실패 — 회전이 부족하다'); return None
    cx, cy, r = fit
    buckets = {int((math.degrees(math.atan2(y - cy, x - cx)) % 360) // 45)
               for x, y in pts}
    cover = len(buckets) / 8.0
    resid = [abs(math.hypot(x - cx, y - cy) - r) for x, y in pts]
    rms = math.sqrt(sum(v * v for v in resid) / len(resid))

    print(f'  샘플 {len(mag)}개, 회전 커버리지 {cover*100:.0f}% ({len(buckets)}/8)')
    print(f'  하드아이언 편향   ox={cx:+7.2f}  oy={cy:+7.2f} µT')
    print(f'  지구 자기장 수평  r ={r:7.2f} µT   (한국 기준 약 30)')
    print(f'  잔차 RMS         {rms:6.2f} µT  = 반지름의 {100*rms/max(r,1e-9):.1f}%')
    print(f'  Z 평균           {sum(m[2] for m in mag)/len(mag):+7.2f} µT')
    if cover < 0.75:
        print('  ⚠️ 커버리지 부족 — 한 바퀴를 다 못 돌면 중심 추정이 크게 틀린다')
        return None
    print()
    if rms < 0.15 * r:
        print('  ✅ 편향이 안정적이다. 크기와 무관하게 보정으로 제거된다.')
        print(f'     적용값:  mag_hard_iron_x: {cx:.2f}   mag_hard_iron_y: {cy:.2f}')
    else:
        print('  ⚠️ 잔차가 크다 — 편향이 일정하지 않거나 회전 중 기울었다.')
        print('     이 경우 상수 보정으로는 못 잡는다(소프트아이언/불안정 자기원).')
    return {'ox': cx, 'oy': cy, 'r': r, 'rms': rms, 'coverage': cover}


def report_scan(scans):
    print('\n=== 라이다 자기 구조물 판별 (회전 중) ===')
    if len(scans) < 30:
        print(f'  스캔 부족({len(scans)})'); return None
    bins = {}
    for m in scans:
        a = m.angle_min
        for rg in m.ranges:
            deg = math.degrees(a) % 360.0
            a += m.angle_increment
            if math.isfinite(rg) and rg > 0:
                bins.setdefault(int(deg // BIN_DEG), []).append(rg)

    # ⚠️ 판정은 **근거리 샘플만** 보고 한다.
    # 전체 표본의 표준편차를 쓰면 **가장자리 빈에서 오답**이 난다: 구조물 경계에
    # 걸친 빈은 어떤 샘플은 구조물(0.08m), 어떤 샘플은 그 옆을 지나 환경(1~3m)에
    # 맞아서 표준편차가 폭증한다. 실제로 2026-08-19 측정에서 같은 0.080m 구간 안에
    # 56도=자기구조물(std 0.0005), 58도=환경(std 0.168)이 섞여 나왔다.
    #
    # 자기 구조물의 진짜 특징은 "**근거리 반사가 항상 같은 거리에 있다**"이다.
    # 그래서 (a) 근거리 샘플의 비율과 (b) 그 샘플들만의 표준편차로 판정한다.
    NEAR = 0.15
    n = len(scans)
    rows = []
    for b, vals in sorted(bins.items()):
        near = [v for v in vals if v <= NEAR]
        if len(near) < n * 0.2:          # 근거리 반사가 드물면 구조물이 아니다
            continue
        near_s = sorted(near)
        med = near_s[len(near_s) // 2]
        mean = sum(near) / len(near)
        sd = (sum((x - mean) ** 2 for x in near) / len(near)) ** 0.5
        rows.append((b, med, sd, len(near) / n))

    if not rows:
        print('  0.30m 이내 지속 반사 없음')
        return None
    print(f'  스캔 {len(scans)}개\n')
    print(f'  {"각도":>7} {"거리(중앙)":>11} {"표준편차":>10} {"근거리비율":>10}  판정')
    self_deg = 0
    for b, med, sd, frac in rows:
        # 회전 중에도 근거리 반사 거리가 일정하면 로봇에 붙어 있는 것.
        # 비율이 낮으면 가장자리(부분 차폐)로 본다 — 그래도 구조물이다.
        if sd <= 0.01:
            verdict = '자기 구조물' if frac > 0.8 else '자기 구조물 (가장자리)'
            self_deg += BIN_DEG
        else:
            verdict = '환경'
        print(f'  {b*BIN_DEG:>5.0f}도 {med:>10.3f}m {sd:>9.4f} {frac*100:>9.0f}%  {verdict}')
    print(f'\n  자기 구조물 총 {self_deg:.0f}도')
    return {'self_deg': self_deg}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=60.0)
    ap.add_argument('--json', help='결과 저장 경로')
    args = ap.parse_args()

    print(f'기록 시작 — {args.sec:.0f}초. 지금부터 제자리에서 천천히 2바퀴 돌려주세요.')
    mag, scans, gyro = record(args.sec)
    if not mag and not scans:
        print('수신 실패 — imu/lidar 서비스를 확인할 것'); return 1

    deg = report_rotation(gyro)
    if deg is not None:
        print(f'\n자이로 적분 회전량: {deg:+.0f}도  ({abs(deg)/360:.1f}바퀴)')
        if abs(deg) < 300:
            print('⚠️ 한 바퀴에 못 미친다 — 보정 신뢰도가 떨어진다')

    m = report_mag(mag)
    s = report_scan(scans)
    if args.json:
        json.dump({'rotation_deg': deg, 'mag': m, 'scan': s},
                  open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n저장: {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
