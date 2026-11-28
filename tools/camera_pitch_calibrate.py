#!/usr/bin/env python3
"""카메라 기울기 다시 재기 — 라이다로 잰 물체 거리 + 검출 상자 아래 끝 줄 → 광축이 위로 든 각 (2026-11-28).

언제: 카메라 각도를 바꾼 뒤(11-28 사용자가 위로 올림). 높이 h·초점 f는 그대로 두고 기울기 하나만 다시 푼다.
쓰는 법: 로봇을 **바닥**에 두고(도크 위는 높이가 달라 값이 틀린다) 정면 1~2 m에 검출되는 물체(병·의자·가방 등)를 세운다.
         물체 앞에 다른 것이 없게. 스택 없이도 된다(라이다 서비스의 /scan을 읽는다).

    python3 tools/camera_pitch_calibrate.py               # 15장, 아무 물체
    python3 tools/camera_pitch_calibrate.py --name bottle --frames 20 --save

계산: 상자 좌우 끝을 카메라 방위로 바꿔(카메라는 base_link 가운데, 라이다는 5.4 cm 앞) 그 안의 라이다 점 중 가장 가까운 무리를
      물체 앞면으로 본다 → 수평 거리 d. 상자 아래 끝 줄 v에서 위 기울기 = atan((v − cy)/f) − atan(h/d).
      물체가 바닥에 닿는 곳이 앞면과 같은 거리라고 본다(세워진 물체). 장마다 값을 내고 중앙값·흩어짐을 보인다.
⚠️ 기울기를 많이 올리면 바닥이 화면 아래 끝에만 남아 가까운 물체는 아래 끝이 잘린다(상자 아래 끝이 479 근처면 버린다).
"""
import argparse, json, math, os, sys, time, urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_detect as C

API = 'http://127.0.0.1:8767'
LX = 0.054


def scan_points():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    if not rclpy.ok():
        rclpy.init()
    n = rclpy.create_node('camera_pitch_calibrate'); got = []
    n.create_subscription(LaserScan, '/scan', lambda m: got.append(m), qos_profile_sensor_data)
    t = time.time()
    while not got and time.time() - t < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    if not got:
        return None
    m = got[-1]; r = np.asarray(m.ranges, float); a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r > 0.12)
    return np.c_[LX + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]          # base_link 기준(앞 +x, 왼쪽 +y)


def one(q, pts, g, width=640, height=480):
    """검출 하나 → (위 기울기°, d, v) 또는 (None, 이유)."""
    if q['y2'] >= height - 2:
        return None, '아래 끝이 화면 끝에 잘림'
    f, cx = g['f'], g['cx']
    # 카메라 화면 u → 방위(왼쪽 +). 상자 좌우 20%씩 안쪽만(가장자리는 배경이 섞임)
    w = q['x2'] - q['x1']; u1, u2 = q['x1'] + 0.2 * w, q['x2'] - 0.2 * w
    b_left, b_right = math.atan((cx - u1) / f), math.atan((cx - u2) / f)
    brg = np.arctan2(pts[:, 1], pts[:, 0])
    sel = pts[(brg <= b_left) & (brg >= b_right) & (pts[:, 0] > 0.15)]
    if len(sel) < 2:
        return None, f'상자 방위 안 라이다 점 {len(sel)}개'
    d_all = np.hypot(sel[:, 0], sel[:, 1]); d0 = d_all.min()
    d = float(np.median(d_all[d_all <= d0 + 0.05]))
    pitch = math.degrees(math.atan((q['y2'] - g['cy']) / f) - math.atan(g['h'] / d))
    return (pitch, d, q['y2']), ''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name'); ap.add_argument('--frames', type=int, default=15); ap.add_argument('--min-score', type=float, default=0.5)
    ap.add_argument('--save', action='store_true', help='docs/calibration/camera_geometry_<날짜>.json으로 남김')
    a = ap.parse_args()
    g = dict(C.GEOM)
    out = []; why = {}; t_last = 0
    for k in range(a.frames * 4):
        if len(out) >= a.frames:
            break
        try:
            d = json.load(urllib.request.urlopen(API + '/api/detect', timeout=3))      # 카메라를 켠다
        except Exception as e:
            print('검출 응답 없음:', e); time.sleep(1); continue
        if not d.get('on') or d.get('t', 0) <= t_last or d.get('age_s', 9) > 1.5:
            time.sleep(0.4); continue
        t_last = d['t']
        dets = [q for q in d.get('dets', []) if q.get('score', 0) >= a.min_score and (not a.name or q.get('name') == a.name)]
        if not dets:
            why['검출 없음'] = why.get('검출 없음', 0) + 1; time.sleep(0.3); continue
        q = max(dets, key=lambda q: (q['x2'] - q['x1']) * (q['y2'] - q['y1']))        # 가장 큰 것 하나
        pts = scan_points()
        if pts is None:
            print('❌ /scan이 안 온다'); return 2
        r, msg = one(q, pts, g)
        if r is None:
            why[msg.split(' ')[0] + '…'] = why.get(msg.split(' ')[0] + '…', 0) + 1; continue
        out.append(dict(name=q['name'], score=round(q['score'], 2), d_m=round(r[1], 3), row=round(r[2], 1), pitch_up_deg=round(r[0], 2)))
        print(f'{len(out):2d}. {q["name"]} {q["score"]:.2f}  거리 {r[1]:.2f} m  아래 끝 {r[2]:.0f}줄  → 위 {r[0]:+.2f}°', flush=True)
        time.sleep(0.3)
    if why:
        print('버린 장:', why)
    if len(out) < 3:
        print('❌ 쓸 만한 장이 3장 미만 — 물체를 1~2 m 정면에 두고(아래 끝이 화면 안에 들게) 다시'); return 1
    p = np.array([o['pitch_up_deg'] for o in out]); med = float(np.median(p))
    print(f'\n위 기울기 중앙값 {med:+.2f}° (흩어짐 표준편차 {p.std():.2f}°, {len(out)}장) — 11-25는 +9.5°')
    vis = math.degrees(math.atan((479 - g['cy']) / g['f'])) - med
    print(f'이 기울기에서 화면 맨 아래 줄 = 바닥 {g["h"] / math.tan(math.radians(vis)):.2f} m' if vis > 0.5 else '이 기울기에서는 화면에 바닥이 거의 안 보인다')
    print(f"camera_detect.GEOM['pitch_up_deg'] = {med:.2f}")
    if a.save:
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs', 'calibration', f'camera_geometry_{time.strftime("%Y%m%d")}.json')
        json.dump(dict(date=time.strftime('%Y-%m-%d %H:%M'), camera='logitech C270 640x480', height_m=g['h'], f_px=g['f'], cy=g['cy'],
                       pitch_up_deg=round(med, 2), pitch_std_deg=round(float(p.std()), 2), points=out,
                       note='위 기울기 = atan((v − cy)/f) − atan(h/d). d는 라이다로 잰 물체 앞면까지 수평 거리(카메라 = base_link 가운데)'),
                  open(path, 'w'), ensure_ascii=False, indent=1)
        print('저장:', path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
