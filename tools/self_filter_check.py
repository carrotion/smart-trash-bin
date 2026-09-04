#!/usr/bin/env python3
"""scan_self_filter 근접 사각지대 가설 검증 도구.

배경: Layer 2 자율탐색 중 2회 실제 충돌(식탁/의자 다리 — 둘 다 얇은 구조물)
발생 후, `scan_self_filter`(±0.105m 정사각형 폴리곤)가 물체가 그 경계에
가까워지면 진짜 장애물을 자기 구조물로 오인해서 지우는 게 아닌가 하는 가설이
나왔으나 당시 검증을 못 하고 넘어감(2026-08-29).

방법: `/scan_raw`(필터 전)와 `/scan`(필터 후)에서 **header.stamp가 정확히
같은 쌍만** 비교한다(2026-09-04 수정 — 처음엔 "각자 가장 최근 도착분"끼리
비교했는데, 라이다 회전마다 점 개수가 살짝 흔들려 인덱스가 어긋난 서로 다른
스캔을 비교하는 타이밍 버그가 있었다. 27~29cm짜리 다수의 가짜 "이상 사례"가
전부 이 버그였음 — 실제 폴리곤은 최대 14.8cm(대각선)까지밖에 안 커서 그렇게
먼 거리가 지워지는 건 물리적으로 필터 자체의 버그일 수 없다).

같은 각도(인덱스)에서 raw는 유효한 거리를 보고하는데 filt는 무효
(inf/range_max)로 나오면 그 지점을 **`base_link` 좌표로 변환**해서 실제
정사각형 폴리곤(±0.105m, `config/scan_self_filter.yaml`) 안쪽인지 밖인지
기하로 직접 판정한다 — 폴리곤이 원이 아니라 사각형이라 각도별 경계 거리가
다르므로(축 방향 10.5cm, 대각선 방향 최대 14.8cm) 단순 반지름 임계값은 틀린다.
라이다 원점이 base_link에서 앞으로 5.4cm 떨어져 있는 것도 반영한다.

폴리곤 **밖인데도** 지워지면 그게 가설이 말하는 진짜 버그.

사용법:
    python3 tools/self_filter_check.py [초, 기본 60]
"""
import rclpy, math, time, sys, csv, os
from sensor_msgs.msg import LaserScan

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
REPORT_INTERVAL = 1.0

# config/scan_self_filter.yaml과 반드시 일치시킬 것
POLY_HALF = 0.105
# base_link -> base_laser 정적 변환의 x축 오프셋 (회전은 항등, y/z는 이 2D
# 판정에 무관). mapping.launch.py 기동 로그에서 실측 확인: (0.054, 0, 0.085)
LASER_DX = 0.054


def in_polygon_base_link(angle_rad, r):
    x_laser = r * math.cos(angle_rad)
    y_laser = r * math.sin(angle_rad)
    x_base = x_laser + LASER_DX
    y_base = y_laser
    return abs(x_base) <= POLY_HALF and abs(y_base) <= POLY_HALF, x_base, y_base


def main():
    rclpy.init()
    n = rclpy.create_node('self_filter_check')
    state = {}
    n.create_subscription(LaserScan, '/scan_raw', lambda m: state.update(raw=m), 5)
    n.create_subscription(LaserScan, '/scan', lambda m: state.update(filt=m), 5)

    t0 = time.time()
    while ('raw' not in state or 'filt' not in state) and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'raw' not in state or 'filt' not in state:
        print('구독 실패 — /scan_raw 또는 /scan이 안 보임. mapping.launch.py 떠있는지 확인.')
        return 1

    print(f'검증 시작 — {DURATION:.0f}초. 지금부터 로봇 정면 쪽에서 얇은 물체를 '
          f'천천히 가까이 가져가면 됨. (진짜 폴리곤 ±{POLY_HALF}m 밖인데 지워지면 문제)')

    fpath = 'docs/calibration/self_filter_check.csv'
    os.makedirs(os.path.dirname(fpath), exist_ok=True)
    newfile = not os.path.exists(fpath)
    f = open(fpath, 'a', newline='')
    w = csv.writer(f)
    if newfile:
        w.writerow(['t', 'angle_deg', 'raw_range_m', 'x_base', 'y_base'])

    def stamp_key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    t_end = time.time() + DURATION
    last_report = 0.0
    last_compared_stamp = None
    suspicious_total = 0
    self_polygon_total = 0
    matched_pairs = 0
    stamp_mismatches = 0

    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        now = time.time()
        raw, filt = state.get('raw'), state.get('filt')
        if raw is None or filt is None:
            continue

        raw_key, filt_key = stamp_key(raw), stamp_key(filt)
        if raw_key != filt_key:
            stamp_mismatches += 1
            continue  # 같은 스캔이 아님 — 비교 스킵(타이밍 버그 재발 방지)
        if raw_key == last_compared_stamp:
            continue  # 이미 비교한 쌍
        last_compared_stamp = raw_key
        matched_pairs += 1

        if now - last_report < REPORT_INTERVAL:
            continue
        last_report = now

        n_bins = min(len(raw.ranges), len(filt.ranges))
        hits = []
        for i in range(n_bins):
            r = raw.ranges[i]
            if not (raw.range_min <= r <= raw.range_max):
                continue
            fr = filt.ranges[i]
            filtered_out = not (filt.range_min <= fr <= filt.range_max)
            if not filtered_out:
                continue
            ang = raw.angle_min + i * raw.angle_increment
            inside, xb, yb = in_polygon_base_link(ang, r)
            if inside:
                self_polygon_total += 1
                continue  # 정상 — 진짜 폴리곤 안쪽
            hits.append((math.degrees(ang) % 360, r, xb, yb))
            suspicious_total += 1
            w.writerow([f'{now - t0:.2f}', f'{math.degrees(ang)%360:.1f}',
                        f'{r:.4f}', f'{xb:.4f}', f'{yb:.4f}'])

        if hits:
            hits.sort()
            desc = ', '.join(f'{a:.0f}°@{r*100:.1f}cm(x={xb:.3f},y={yb:.3f})'
                              for a, r, xb, yb in hits[:5])
            more = f' 외 {len(hits)-5}개' if len(hits) > 5 else ''
            print(f'[{now-t0:5.1f}s] ⚠️ 진짜 폴리곤 밖인데 지워짐: {desc}{more}')
        f.flush()

    f.close()
    print(f'\n=== 종료: 매칭된 스캔쌍 {matched_pairs}개(스탬프 불일치로 스킵 '
          f'{stamp_mismatches}회), 진짜 폴리곤 밖 오삭제 {suspicious_total}건, '
          f'정상(폴리곤 안쪽) 필터링 {self_polygon_total}건 ===')
    print(f'원본 로그: {fpath}')
    if suspicious_total > 0:
        print('⚠️ 가설 지지 — 폴리곤 밖(x_base/y_base로 확인 가능)인데 지워진 사례가 실재함.')
    else:
        print('이번 시행에서는 폴리곤 밖 오탐 없음 — 가설 반증 방향(접근 각도/속도를 '
              '바꿔 추가 시행 권장).')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
