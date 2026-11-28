#!/usr/bin/env python3
"""카메라 렌즈 값(초점 fx·fy, 중심 cx·cy, 왜곡) 재기 — 인쇄한 체스판으로 OpenCV 표준 보정 (2026-11-28).

왜: 11-28 키 170 cm 사람으로 확인하니 f=644 px로는 발끝과 머리 끝이 서로 다른 기울기를 냈다(16.5° vs 10.5°).
    라이다 모서리 방위와 화면 모서리를 맞추면 f≈820이었다 → 짐작 대신 표준 방법으로 잰다(cv2.calibrateCamera).
판: 10×7칸(내부 모서리 9×6), 칸 25.4 mm, A4에 100%로 인쇄. 인쇄 뒤 칸 하나를 자로 재서 다르면 --square로 준다.

쓰는 법: 판을 평평한 판지 등에 붙이고, 카메라 앞 0.4~1.5 m에서 **여러 자리·여러 기울기**로 천천히 비춘다
         (화면 가운데·네 구석·가까이·멀리·위아래 좌우로 30~45° 기울여서). 같은 모습은 알아서 건너뛴다.

    python3 tools/camera_intrinsics_calibrate.py               # 20장 모이면 계산
    python3 tools/camera_intrinsics_calibrate.py --save        # docs/calibration/camera_intrinsics_<날짜>.json

사진은 저장소 밖 ~/ar_scan/calib_<시각>/에만 둔다(사진은 커밋하지 않는다).
"""
import argparse, json, math, os, sys, time, urllib.request

import cv2
import numpy as np

API = 'http://127.0.0.1:8767'
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def grab():
    b = urllib.request.urlopen(API + '/camera.jpg?raw=1', timeout=5).read()
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cols', type=int, default=9); ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square', type=float, default=25.4, help='칸 한 변(mm)')
    ap.add_argument('--views', type=int, default=20); ap.add_argument('--timeout', type=float, default=300)
    ap.add_argument('--save', action='store_true')
    a = ap.parse_args()
    pat = (a.cols, a.rows)
    obj = np.zeros((a.cols * a.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square / 1000.0
    out_dir = os.path.expanduser(time.strftime('~/ar_scan/calib_%Y%m%d_%H%M%S')); os.makedirs(out_dir, exist_ok=True)
    views, kept, size = [], [], None
    t0 = time.time(); t_wake = 0; n_try = 0
    print(f'체스판 {a.cols}×{a.rows} 모서리, 칸 {a.square} mm — 카메라 앞에서 천천히 움직여 주세요', flush=True)
    while len(views) < a.views and time.time() - t0 < a.timeout:
        if time.time() - t_wake > 5:                      # 카메라는 검출 요청이 있어야 켜져 있다
            try:
                urllib.request.urlopen(API + '/api/detect', timeout=3).read()
            except Exception:
                pass
            t_wake = time.time()
        try:
            img = grab()
        except Exception as e:
            print('화면 못 받음:', e); time.sleep(1); continue
        if img is None:
            time.sleep(0.3); continue
        n_try += 1
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); size = g.shape[::-1]
        ok, c = cv2.findChessboardCorners(g, pat, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if not ok:
            time.sleep(0.2); continue
        c = cv2.cornerSubPix(g, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
        p = c.reshape(-1, 2)
        ctr = p.mean(0); area = cv2.contourArea(cv2.convexHull(p).astype(np.float32))
        v1 = p[a.cols - 1] - p[0]; v2 = p[-a.cols] - p[0]
        shape = (math.atan2(v1[1], v1[0]), np.linalg.norm(v1) / max(np.linalg.norm(v2), 1))  # 돌림·찌그러짐(기울임)
        new = all(np.linalg.norm(ctr - k[0]) > 40 or abs(area / k[1] - 1) > 0.25
                  or abs(shape[0] - k[2][0]) > 0.15 or abs(shape[1] / k[2][1] - 1) > 0.15 for k in kept)
        if not new:
            time.sleep(0.2); continue
        kept.append((ctr, area, shape)); views.append(c)
        cv2.imwrite(os.path.join(out_dir, f'{len(views):02d}.jpg'), img)
        print(f'{len(views):2d}/{a.views}  가운데 ({ctr[0]:.0f},{ctr[1]:.0f})  넓이 {area / (size[0] * size[1]) * 100:.0f}%', flush=True)
        time.sleep(0.5)
    if len(views) < 8:
        print(f'❌ 쓸 만한 장 {len(views)}장(8장 미만, 시도 {n_try}장) — 판이 화면에 다 들어오게, 밝은 데서 다시'); return 1
    rms, K, dist, rv, tv = cv2.calibrateCamera([obj] * len(views), views, size, None, None,
                                               flags=cv2.CALIB_FIX_K3)   # 웹캠·20장이면 k3는 흔들리기만 한다(가짜 화면 시험에서 −3.7)
    errs = []
    for o_, c_, r_, t_ in zip([obj] * len(views), views, rv, tv):
        pr, _ = cv2.projectPoints(o_, r_, t_, K, dist); errs.append(float(np.sqrt(((pr - c_) ** 2).sum(-1).mean())))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = 2 * math.degrees(math.atan(size[0] / 2 / fx)); vfov = 2 * math.degrees(math.atan(size[1] / 2 / fy))
    print(f'\n{len(views)}장, 재투영 오차 RMS {rms:.3f} px (장마다 최대 {max(errs):.3f})')
    print(f'fx {fx:.1f}  fy {fy:.1f}  cx {cx:.1f}  cy {cy:.1f}  화각 {hfov:.1f}×{vfov:.1f}°   (그동안 쓴 값: f 644, 320/240, 52.8×40.9°)')
    print('왜곡 k1 k2 p1 p2 k3 =', ' '.join(f'{x:+.4f}' for x in dist.ravel()[:5]))
    # 오늘 잰 사람 발끝 점(라이다 거리, 상자 아래 끝 줄)으로 기울기 다시 풀기 — 왜곡은 무시(화면 가운데 근처)
    pf = os.path.join(REPO, 'docs', 'calibration', 'camera_geometry_20261128.json')
    pitch = None
    if os.path.exists(pf):
        P = json.load(open(pf)); h = P.get('height_m', 0.14)
        ps = [math.degrees(math.atan((q['row'] - cy) / fy) - math.atan(h / q['d_m'])) for q in P['points']]
        pitch = float(np.median(ps))
        print(f'이 렌즈 값으로 11-28 발끝 점 {len(ps)}개를 다시 풀면 위 기울기 {pitch:+.2f}° (흩어짐 {np.std(ps):.2f}°)')
    if a.save:
        path = os.path.join(REPO, 'docs', 'calibration', f'camera_intrinsics_{time.strftime("%Y%m%d")}.json')
        json.dump(dict(date=time.strftime('%Y-%m-%d %H:%M'), camera='logitech C270 640x480 (/camera.jpg?raw=1)', board=f'{a.cols}x{a.rows} inner, {a.square} mm',
                       views=len(views), rms_px=round(rms, 3), fx=round(fx, 1), fy=round(fy, 1), cx=round(cx, 1), cy=round(cy, 1),
                       hfov_deg=round(hfov, 1), vfov_deg=round(vfov, 1), dist=[round(float(x), 5) for x in dist.ravel()],
                       pitch_up_deg_from_20261128_feet=None if pitch is None else round(pitch, 2)),
                  open(path, 'w'), ensure_ascii=False, indent=1)
        print('저장:', path)
    print('사진:', out_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
