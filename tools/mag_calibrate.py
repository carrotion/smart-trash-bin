#!/usr/bin/env python3
"""지자기센서(AK09916) 하드아이언 보정값 측정.

원리
----
지구 자기장은 크기가 일정하므로, 센서를 수평면에서 한 바퀴 돌리면 (Bx, By)는
**원점을 중심으로 하는 원**을 그려야 한다. 실제로는 로봇 자신의 철재/자석 때문에
일정한 벡터가 더해져서 원의 중심이 (ox, oy)만큼 밀려난다 — 이게 하드아이언 편향이고,
빼주면 방위 오차가 사라진다.

⚠️ 반드시 **손으로** 돌릴 것. 모터를 돌리면 모터 자체가 52~69µT를 만들어내서
   측정을 오염시킨다(2026-08-17 실측). 손 회전은 승인도 필요 없다.

사용법
------
    python3 tools/mag_calibrate.py --out /tmp/magcal.json      # 기록 시작(백그라운드)
    (로봇을 제자리에서 천천히 2바퀴 돌린다)
    python3 tools/mag_calibrate.py --analyze /tmp/magcal.json  # 결과 분석

원 적합은 대수적 최소제곱(Kasa 방법)을 쓴다: 원 x²+y²+Dx+Ey+F=0 를 선형계로 풀면
중심은 (-D/2, -E/2), 반지름은 sqrt(중심² - F). 반복 최적화가 필요 없어 안정적이다.
"""

import argparse
import json
import math
import os
import signal
import sys
import time


def record(out_path, duration):
    import rclpy
    from sensor_msgs.msg import MagneticField

    samples = []
    rclpy.init()
    node = rclpy.create_node('mag_calibrate')

    def cb(m):
        samples.append([
            m.magnetic_field.x * 1e6,
            m.magnetic_field.y * 1e6,
            m.magnetic_field.z * 1e6,
        ])

    node.create_subscription(MagneticField, '/imu/mag', cb, 50)

    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *a: stop.update(v=True))

    end = time.time() + duration
    while time.time() < end and not stop['v']:
        rclpy.spin_once(node, timeout_sec=0.1)
        # 진행 상황을 파일로 계속 흘려둬서, 도중에도 몇 개 모였는지 볼 수 있게 한다.
        if len(samples) % 100 == 0:
            _dump(out_path, samples)

    _dump(out_path, samples)
    node.destroy_node()
    rclpy.shutdown()


def _dump(path, samples):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump({'samples': samples}, f)
    os.replace(tmp, path)


def fit_circle(pts):
    """Kasa 대수 원 적합. 반환 (cx, cy, r)."""
    n = len(pts)
    sx = sy = sxx = syy = sxy = sz = szx = szy = 0.0
    for x, y in pts:
        z = x * x + y * y
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y
        sz += z; szx += z * x; szy += z * y
    # 정규방정식 A·[D,E,F]^T = b
    a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
    b = [-szx, -szy, -sz]
    # 3x3 가우스 소거
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
        s = b[i] - sum(a[i][c] * sol[c] for c in range(i + 1, 3))
        sol[i] = s / a[i][i]
    d, e, f = sol
    cx, cy = -d / 2.0, -e / 2.0
    disc = cx * cx + cy * cy - f
    if disc <= 0:
        return None
    return cx, cy, math.sqrt(disc)


def analyze(path):
    data = json.load(open(path))
    s = data['samples']
    if len(s) < 50:
        print(f"샘플이 {len(s)}개뿐이다 — 최소 50개 필요. 기록이 제대로 안 됐다.")
        return 1

    pts = [(p[0], p[1]) for p in s]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in s]

    # 한 바퀴를 실제로 돌았는지 확인: 각도 커버리지를 45도 8구간으로 본다.
    fit = fit_circle(pts)
    if fit is None:
        print("원 적합 실패 — 데이터가 한 방향으로만 몰려 있다.")
        return 1
    cx, cy, r = fit

    buckets = set()
    for x, y in pts:
        ang = math.degrees(math.atan2(y - cy, x - cx)) % 360
        buckets.add(int(ang // 45))
    cover = len(buckets) / 8.0

    resid = [abs(math.hypot(x - cx, y - cy) - r) for x, y in pts]
    rms = math.sqrt(sum(v * v for v in resid) / len(resid))

    print(f"샘플 {len(s)}개, 회전 커버리지 {cover*100:.0f}% ({len(buckets)}/8 구간)")
    print()
    print(f"  원 중심(하드아이언 편향)  ox = {cx:+7.2f} µT,  oy = {cy:+7.2f} µT")
    print(f"  원 반지름(수평 지구자기장) r  = {r:7.2f} µT   (한국 기준 약 30)")
    print(f"  적합 잔차 RMS             {rms:.2f} µT")
    print(f"  보정 전 수평 세기 범위    {min(map(abs,xs)):.1f}~{max(map(abs,xs)):.1f}(X) "
          f"{min(map(abs,ys)):.1f}~{max(map(abs,ys)):.1f}(Y)")
    print(f"  Z 평균                    {sum(zs)/len(zs):+.2f} µT")
    print()

    if cover < 0.75:
        print("⚠️ 커버리지가 부족하다. 한 바퀴를 다 못 돌았으면 중심 추정이 크게 틀린다.")
        print("   다시 기록해서 최소 6/8 구간을 채울 것.")
        return 1
    if rms > 0.15 * r:
        print("⚠️ 잔차가 크다 — 회전 중 기울었거나(수평 유지 필요) 주변 철재 영향.")

    print("적용할 값 (imu_publisher_node.py 또는 설정에 넣을 것):")
    print(f"    mag_hard_iron_x: {cx:.2f}")
    print(f"    mag_hard_iron_y: {cy:.2f}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', help='기록 모드: 샘플을 저장할 경로')
    ap.add_argument('--duration', type=float, default=180.0, help='기록 최대 시간(초)')
    ap.add_argument('--analyze', help='분석 모드: 기록 파일 경로')
    args = ap.parse_args()

    if args.analyze:
        return analyze(args.analyze)
    if args.out:
        record(args.out, args.duration)
        return 0
    ap.print_help()
    return 1


if __name__ == '__main__':
    sys.exit(main())
