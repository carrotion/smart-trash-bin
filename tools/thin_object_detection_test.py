#!/usr/bin/env python3
"""가는 다리류(의자/식탁 다리) 라이다 원시 감지율 vs 거리 실측 도구.

배경: 2026-08-29 Layer 2 자율탐색 중 2회 충돌(둘 다 가늘고 낮은 다리류) —
당시 유력 후보였던 scan_self_filter 근접 사각지대 가설은 2026-09-04
검증에서 미지지로 결론남(`docs/self_filter_check.csv`, CLAUDE.md 참고).
다음 후보: LD06 각분해능(실측 0.79°/454점 360도) 자체의 물리적 한계 —
거리 1m에서 인접 광선 간격이 약 1.4cm로, 가는 다리(2~3cm)와 비슷한
수준이라 한 바퀴 스캔에서 아예 못 맞히는 회전이 생길 수 있다는 가설.

방법: `/scan_raw`(필터 영향 배제, 순수 센서 성능만 봄)에서 지정한 전방
각도창 안의 최소 유효거리를 매 스캔마다 기록한다. 사용자가 얇은 물체를
로봇 정면 가까운 곳(~10cm)에서 먼 곳(~2m)까지 천천히 직선으로 멀어지며
드는 동안 돌리면, 거리별로 "그 스캔에서 감지 성공/실패"가 시계열로 남는다.
연속 실패(dropout) 구간의 길이와 그때의 대략적 거리(양옆 성공 프레임의
거리로 보간)를 분석해서 거리별 감지율 곡선을 뽑는다.

⚠️ 정지 상태 도크 위에서도 가능 — 로봇이 움직일 필요 없음, 물체만 움직이면 됨.

사용법:
    python3 tools/thin_object_detection_test.py [초, 기본 90]
"""
import rclpy, math, time, sys, csv, os
from sensor_msgs.msg import LaserScan

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
# 기본 /scan_raw(필터 전, mapping.launch.py 뜬 상태 가정). 상시 서비스
# smart-trash-lidar.service만 떠있을 땐 그게 이미 원시 데이터를 'scan'
# 이름으로 내보내므로(스스로 필터 안 함) 두 번째 인자로 'scan' 지정 가능.
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/scan_raw'
# 전방 창 — 자기 구조물(정면 근처는 대체로 뚫려있음, CLAUDE.md 08-19 실측
# "정면 ±30° 차폐 0%") 밖에서 물체를 든다는 전제. 자기 구조물 대역
# (0.078~0.084m 부근)은 최소값 탐색에서 제외한다.
FRONT_HALF_DEG = 20
SELF_BAND = (0.075, 0.090)


def main():
    rclpy.init()
    n = rclpy.create_node('thin_object_detection_test')
    state = {}
    n.create_subscription(LaserScan, TOPIC, lambda m: state.update(raw=m), 5)

    t0 = time.time()
    while 'raw' not in state and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'raw' not in state:
        print(f'구독 실패 — {TOPIC}이 안 보임. 라이다 서비스/런치 확인.')
        return 1

    print(f'검증 시작 — {DURATION:.0f}초. 얇은 물체를 로봇 정면(±{FRONT_HALF_DEG}도) '
          f'가까운 곳(~10cm)에서 천천히 직선으로 멀어지며 2m 정도까지 들어줘. '
          f'최대한 일정한 속도로, 급하지 않게.')

    fpath = 'docs/calibration/thin_object_detection.csv'
    os.makedirs(os.path.dirname(fpath), exist_ok=True)
    newfile = not os.path.exists(fpath)
    f = open(fpath, 'a', newline='')
    w = csv.writer(f)
    if newfile:
        w.writerow(['t', 'min_range_m', 'hit'])

    t_end = time.time() + DURATION
    last_stamp = None
    samples = []  # (t, min_range or None)

    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        raw = state.get('raw')
        if raw is None:
            continue
        key = (raw.header.stamp.sec, raw.header.stamp.nanosec)
        if key == last_stamp:
            continue
        last_stamp = key
        now = time.time()

        best = None
        for i, r in enumerate(raw.ranges):
            if not (raw.range_min <= r <= raw.range_max):
                continue
            if SELF_BAND[0] <= r <= SELF_BAND[1]:
                continue  # 자기 구조물 — 대상에서 제외
            ang = math.degrees(raw.angle_min + i * raw.angle_increment)
            ang = ((ang + 180) % 360) - 180  # -180~180으로 정규화, 0=정면
            if abs(ang) <= FRONT_HALF_DEG:
                if best is None or r < best:
                    best = r

        samples.append((now - t0, best))
        w.writerow([f'{now - t0:.2f}', f'{best:.4f}' if best is not None else '', int(best is not None)])
        f.flush()

    f.close()

    # 분석: dropout(연속 미검출) 구간을 찾아 양옆 성공 프레임 거리로 보간
    print('\n=== 분석 ===')
    hit_count = sum(1 for _, r in samples if r is not None)
    print(f'전체 프레임 {len(samples)}개, 감지 {hit_count}개 '
          f'({100*hit_count/max(len(samples),1):.1f}%), 미검출 {len(samples)-hit_count}개')

    dropouts = []
    i = 0
    while i < len(samples):
        if samples[i][1] is None:
            j = i
            while j < len(samples) and samples[j][1] is None:
                j += 1
            prev_r = samples[i-1][1] if i > 0 else None
            next_r = samples[j][1] if j < len(samples) else None
            approx = None
            if prev_r is not None and next_r is not None:
                approx = (prev_r + next_r) / 2
            elif prev_r is not None:
                approx = prev_r
            elif next_r is not None:
                approx = next_r
            dropouts.append((samples[i][0], samples[j-1][0], j - i, approx))
            i = j
        else:
            i += 1

    if dropouts:
        print(f'\n연속 미검출 구간 {len(dropouts)}개 (프레임수 1개=그 순간만 놓침, '
              f'여러개=한동안 완전히 못 봄):')
        for t_start, t_end_d, n_frames, approx_r in dropouts:
            dur = t_end_d - t_start
            approx_s = f'{approx_r*100:.0f}cm' if approx_r else '?'
            flag = ' ⚠️ 0.3초 이상 연속 미검출' if dur >= 0.3 else ''
            print(f'  t={t_start:.1f}~{t_end_d:.1f}s ({n_frames}프레임, {dur:.2f}초) '
                  f'대략 거리 {approx_s}{flag}')
    else:
        print('\n미검출 구간 없음 — 이번 시행에서는 계속 감지됨.')

    print(f'\n원본 로그: {fpath}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
