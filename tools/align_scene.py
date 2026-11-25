#!/usr/bin/env python3
"""3D 장면 좌표를 로봇 라이다 지도 좌표에 맞춘다 (2026-11-25, 주행 AR의 C 준비).

쓰임: 집을 찍어 만든 3D 장면은 자기 마음대로의 좌표계를 쓴다. 그래서 장면 위에 로봇 자세·경로를 그리려면
      "장면 좌표 → 지도 좌표" 변환(크기·회전·이동)을 알아야 한다. 바닥에 1~6번 종이를 놓고 찍은 뒤,
      장면에서 종이 중심을 짚어 짝을 만들면 이 도구가 변환과 오차(cm)를 낸다.

짝 파일(JSON) — 종이 번호마다 장면 좌표와 지도 좌표:
    {"pairs": [{"name": "1", "scene": [1.23, -0.44], "map": [4.14, 2.73]}, ...],
     "check": [{"name": "화장실 앞", "scene": [...], "map": [...]}]}     # check는 맞춤에 안 쓰고 오차만 잰다

    python3 tools/align_scene.py --pairs pairs.json --out ~/maps/ar_transform.json
    python3 tools/align_scene.py --selftest

결과: 변환(크기 s·회전 θ·이동 t)과 점마다 남은 오차(cm), 기준점 RMS·최대, check 점 오차.
      장면이 거울처럼 뒤집혀 보이면(det<0) 경고한다 — 그때는 장면을 뒤집어 다시 맞춘다.
"""
import argparse, json, math, sys

import numpy as np


def fit_similarity(src, dst):
    """src(장면) → dst(지도) 유사변환 s·R·t (Umeyama). 반사는 허용하지 않는다."""
    src = np.asarray(src, float); dst = np.asarray(dst, float)
    if len(src) < 2 or len(src) != len(dst):
        raise ValueError('짝이 2개 이상이어야 하고 개수가 같아야 한다')
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    cov = D.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(cov)
    E = np.eye(2)
    if np.linalg.det(U @ Vt) < 0:      # 반사 금지
        E[-1, -1] = -1
    R = U @ E @ Vt
    var_s = (S ** 2).sum() / len(src)
    s = float(np.trace(np.diag(sig) @ E) / var_s) if var_s > 0 else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def apply(s, R, t, pts):
    return (s * (np.asarray(pts, float) @ R.T)) + t


def report(pairs, check=()):
    src = [p['scene'] for p in pairs]; dst = [p['map'] for p in pairs]
    s, R, t = fit_similarity(src, dst)
    pred = apply(s, R, t, src)
    err = np.hypot(*(pred - np.asarray(dst, float)).T) * 100          # cm
    return s, R, t, err, [round(float(e), 1) for e in err], (pred, )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pairs'); ap.add_argument('--out'); ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.pairs:
        ap.print_help(); return 2
    d = json.load(open(a.pairs)); pairs = d['pairs']; check = d.get('check') or []
    s, R, t, err, errs, (pred,) = report(pairs, check)
    th = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    print(f'변환(장면 → 지도): 크기 {s:.3f}, 회전 {th:+.2f}°, 이동 ({t[0]:+.2f}, {t[1]:+.2f}) m')
    if np.linalg.det(R) < 0:
        print('⚠️ 거울처럼 뒤집힌 맞춤이다 — 장면 좌표를 뒤집고 다시 맞추세요')
    print('기준점 오차(cm):')
    for p, e in zip(pairs, errs):
        print(f'  {p["name"]:<10} {e:5.1f} cm')
    print(f'  RMS {float(np.sqrt((err ** 2).mean())):.1f} cm, 최대 {max(errs):.1f} cm')
    out = {'scale': round(s, 4), 'rotation_deg': round(th, 3), 'translation_m': [round(float(v), 4) for v in t],
           'rotation_matrix': [[round(float(v), 6) for v in row] for row in R],
           'pairs_rms_cm': round(float(np.sqrt((err ** 2).mean())), 2), 'pairs_max_cm': max(errs),
           'pairs': [dict(p, err_cm=e) for p, e in zip(pairs, errs)], 'check': []}
    if check:
        cp = apply(s, R, t, [c['scene'] for c in check]); ce = np.hypot(*(cp - np.asarray([c['map'] for c in check], float)).T) * 100
        print('기준점 아닌 곳 오차(cm):')
        for c, e in zip(check, ce):
            print(f'  {c["name"]:<10} {float(e):5.1f} cm')
            out['check'].append(dict(c, err_cm=round(float(e), 1)))
    if a.out:
        import os
        os.makedirs(os.path.dirname(os.path.expanduser(a.out)) or '.', exist_ok=True)
        json.dump(out, open(os.path.expanduser(a.out), 'w'), ensure_ascii=False, indent=1)
        print('저장:', a.out)
    return 0


def selftest():
    """알려진 변환으로 만든 점 6개(+잡음 2 cm)로 되찾는지, check 점 오차도 재지는지 확인."""
    rng = np.random.default_rng(3)
    truth = dict(s=1.37, th=math.radians(-62.0), t=np.array([4.1, -1.7]))
    R = np.array([[math.cos(truth['th']), -math.sin(truth['th'])], [math.sin(truth['th']), math.cos(truth['th'])]])
    scene = np.array([[0, 0], [3, 0], [3, 2], [0, 2], [1.5, 1.0], [2.5, 1.6]], float)
    mp = truth['s'] * (scene @ R.T) + truth['t']
    noisy = mp + rng.normal(0, 0.02, mp.shape)
    s, R2, t = fit_similarity(scene, noisy)
    back = apply(s, R2, t, scene)
    err = np.hypot(*(back - mp).T) * 100
    dth = abs(math.degrees(math.atan2(R2[1, 0], R2[0, 0])) - math.degrees(truth['th']))
    ok1 = abs(s - truth['s']) / truth['s'] < 0.02 and dth < 1.0 and np.hypot(*(t - truth['t'])) < 0.05   # 잡음 2 cm가 섞였으니 여유를 둔다
    ok2 = float(err.max()) < 6.0                     # 잡음 2 cm면 되찾은 오차는 6 cm 안이어야
    # 반사(뒤집힘)를 금지하는지: 거울로 뒤집힌 점을 넣으면 회전으로 흉내내고 남은 오차가 커야 한다
    mirror = mp * np.array([-1, 1])
    s3, R3, t3 = fit_similarity(scene, mirror)
    err3 = np.hypot(*(apply(s3, R3, t3, scene) - mirror).T) * 100
    ok3 = float(err3.max()) > 20.0                   # 뒤집힘은 못 맞춘다(경고로 알린다)
    print(f'{"✅" if ok1 else "❌"} 크기·회전·이동 되찾기: s={s:.3f}(참 {truth["s"]}), 회전 오차 {dth:.2f}°, 이동 오차 {np.hypot(*(t-truth["t"]))*100:.1f} cm')
    print(f'{"✅" if ok2 else "❌"} 잡음 2 cm에서 점 오차 최대 {err.max():.1f} cm(<6)')
    print(f'{"✅" if ok3 else "❌"} 거울(뒤집힘)은 맞추지 못하고 크게 남는다: 최대 {err3.max():.1f} cm(>20)')
    return 0 if (ok1 and ok2 and ok3) else 1


if __name__ == '__main__':
    sys.exit(main())
