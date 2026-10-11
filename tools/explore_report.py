#!/usr/bin/env python3
"""탐색 한 판의 결과를 숫자로 모은다 — 주행기 서버 기록(jsonl) + 넓이 기록(coverage.csv) + 자세·구역 기록(watch.csv) + 저장된 지도.

다음 탐색과 같은 잣대로 비교하려고 만든다(10-11 첫 실물 탐색). 눈으로 "좋아졌다"가 아니라 같은 항목을 다시 잰다.

    python3 tools/explore_report.py --server docs/calibration/step_nav_server_YYYYMMDD_HHMMSS.jsonl \
        [--coverage logs/…/coverage.csv] [--watch logs/…/watch.csv] [--pgm ~/maps/이름.pgm] [--battery logs/battery/YYYYMMDD.csv]
"""
import argparse, collections, csv, json, math, os, statistics, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def pct(a, b):
    return f'{100 * a / b:.0f}%' if b else '–'


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float('nan')


def group(msg):
    for key in ('취소', '자리가 스캔과 안 맞음', '빈 경로', '미확인 물체', '앞 여유 부족', '회전 실패', '회전 반경 막힘', '직진 실패', '시간 초과', '금지 구역', '진동', 'TF'):
        if key in msg:
            return key
    return msg[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--server', required=True); ap.add_argument('--coverage'); ap.add_argument('--watch'); ap.add_argument('--pgm'); ap.add_argument('--battery')
    a = ap.parse_args()
    R = [json.loads(l) for l in open(a.server)]
    t0 = min(e['t'] for r in R for e in r['log']) if any(r['log'] for r in R) else R[0]['t']; t1 = R[-1]['t']
    ok = [r for r in R if r['result']['ok']]; bad = [r for r in R if not r['result']['ok']]
    print(f'== 목표 ({os.path.basename(a.server)})')
    print(f'  기간 {(t1 - t0) / 60:.1f}분, 목표 {len(R)}개: 도착 {len(ok)} ({pct(len(ok), len(R))}), 실패 {len(bad)}')
    c = collections.Counter(group(r['result']['msg']) for r in bad); spent = collections.defaultdict(float)
    for r in bad:
        spent[group(r['result']['msg'])] += r['result']['secs']
    for k, n in c.most_common():
        print(f'    실패 {k}: {n}건, 쓴 시간 {spent[k]:.0f}초')
    if ok:
        e = [r['result']['final_err_m'] * 100 for r in ok]; d = [math.hypot(r['goal'][0] - r['start'][0], r['goal'][1] - r['start'][1]) for r in ok]
        print(f'  도착 오차 cm: 중앙 {statistics.median(e):.1f}, 최대 {max(e):.1f} | 도착 목표의 직선 거리 중앙 {statistics.median(d):.1f} m, 걸린 시간 중앙 {statistics.median([r["result"]["secs"] for r in ok]):.0f}초')
    tot_goal = sum(r['result']['secs'] for r in R)
    print(f'  목표 수행에 쓴 시간 {tot_goal / 60:.1f}분 (도착한 목표 {sum(r["result"]["secs"] for r in ok) / 60:.1f}분, 실패한 목표 {sum(r["result"]["secs"] for r in bad) / 60:.1f}분), 목표 사이 대기 {(t1 - t0 - tot_goal) / 60:.1f}분')

    L = [e for r in R for e in r['log']]
    turns = [e for e in L if e['kind'] in ('turn', 'final_turn')]; drives = [e for e in L if e['kind'] in ('drive', 'back', 'retreat', 'rot_clear', 'final_fix') and e.get('secs')]
    print('== 기본 동작')
    if turns:
        ang = [abs(e['arg']) for e in turns]; err = [abs(e['err_deg']) for e in turns if e.get('err_deg') is not None]; tok = sum(1 for e in turns if e.get('ok'))
        print(f'  회전 {len(turns)}회(성공 {tok}): 각도 중앙 {statistics.median(ang):.0f}°, 90° 넘는 회전 {sum(x > 90 for x in ang)}회, 150° 넘는 회전 {sum(x > 150 for x in ang)}회, '
              f'합 {sum(ang):.0f}° | 각도 오차 중앙 {statistics.median(err):.1f}°, 95% {q(err, 0.95):.1f}° | 시간 합 {sum(e.get("secs", 0) for e in turns) / 60:.1f}분')
    if drives:
        dist = [abs(e['arg']) for e in drives if isinstance(e['arg'], (int, float))]; err = [abs(e['err_m']) * 100 for e in drives if e.get('err_m') is not None]; dok = sum(1 for e in drives if e.get('ok'))
        print(f'  직진 {len(drives)}회(성공 {dok}): 걸음 중앙 {statistics.median(dist) * 100:.0f} cm, 명령 거리 합 {sum(dist):.1f} m | 거리 오차 중앙 {statistics.median(err):.1f} cm, 95% {q(err, 0.95):.1f} cm '
              f'| 시간 합 {sum(e.get("secs", 0) for e in drives) / 60:.1f}분')
    pcs = [e for e in L if e['kind'] == 'pose_check' and e.get('ratio') is not None]      # 출발 전 자리 검사(10-11~, tools/pose_check.py)
    if pcs:
        rr = [e['ratio'] * 100 for e in pcs]
        print(f'  자리 검사 {len(pcs)}회: 아는 칸의 점 중 벽 위 비율 최소 {min(rr):.0f}%, 5% 분위 {q(rr, 0.05):.0f}%, 중앙 {statistics.median(rr):.0f}% | 기준(80%) 아래 {sum(not e["ok"] for e in pcs)}회, '
              f'그 때문에 거절된 목표 {sum("자리가 스캔과 안 맞음" in r["result"]["msg"] for r in R)}개')
    kinds = collections.Counter(e['kind'] for e in L)
    print('  그 밖의 일: ' + ', '.join(f'{k} {kinds[k]}' for k in ('plan_fail', 'plan_shift', 'retreat', 'rot_near', 'rot_clear', 'near_unknown', 'blocked', 'lookahead', 'shorten', 'near_accept') if kinds[k]))

    if a.coverage:
        rows = [r for r in csv.DictReader(open(a.coverage))]; res = float(rows[0]['resolution']) ** 2
        T = [(float(r['wall_epoch']), int(r['known_cells']) * res) for r in rows]; tt0 = T[0][0]
        print('== 지도 넓이(아는 칸)')
        print('  ' + ', '.join(f'{m}분 {min(T, key=lambda x: abs(x[0] - tt0 - 60 * m))[1]:.1f} m²' for m in (2, 5, 10, 15, 20, 30, 40, 50) if tt0 + 60 * m <= T[-1][0] + 30))
        fin = T[-1][1]
        for frac in (0.5, 0.8, 0.9):
            hit = next((t for t, v in T if v >= frac * fin), None)
            print(f'    마지막 넓이({fin:.1f} m²)의 {int(frac * 100)}%에 도달: {(hit - tt0) / 60:.1f}분')
        best = 0.0; last_t, last_v = T[0]
        for t, v in T[1:]:
            if v - last_v >= 0.5:
                best = max(best, t - last_t); last_t, last_v = t, v
        print(f'    넓이가 0.5 m²도 안 늘어난 가장 긴 구간 {max(best, T[-1][0] - last_t) / 60:.1f}분')
    if a.watch:
        n = 0; dist = 0.0; mn = 9e9; cross = 0; last = None
        for r in csv.DictReader(open(a.watch)):
            x, y = float(r['x']), float(r['y']); g = float(r['body_gap']); n += 1; mn = min(mn, g); cross += g < -0.005
            if last:
                d = math.hypot(x - last[0], y - last[1]); dist += d
            last = (x, y)
        print(f'== 자세·금지 구역 기록: 표본 {n}개, 이동 {dist:.1f} m, 차체-구역 선 최소 {mn * 100:+.1f} cm, 선을 넘은 표본 {cross}')
        if a.coverage:
            print(f'    이동 1 m당 지도 {fin / dist:.2f} m²')
    if a.pgm:
        import map_quality
        occ, w, h = map_quality.from_pgm(a.pgm); raw = open(a.pgm, 'rb').read().split(None, 4)[4][-w * h:]
        known = sum(1 for v in raw if v != 205); ang = map_quality.wall_angles(occ); hh = collections.Counter(int(x // 5) for x in ang); pk = hh.most_common(1)[0][0]
        al = sum(nn for b, nn in hh.items() if min(abs(b - pk), 18 - abs(b - pk)) <= 2) / len(ang)
        print(f'== 저장된 지도 {os.path.basename(a.pgm)}: 아는 넓이 {known * 0.0025:.1f} m², 벽 칸 {len(occ)}개, 벽 방향 정렬도 {al * 100:.1f}% (지배 방향 ±10° 안, 높을수록 좋다)')
    if a.battery:
        B = [(float(r['epoch']), float(r['v'])) for r in csv.DictReader(open(a.battery)) if t0 - 60 <= float(r['epoch']) <= t1 + 60]
        if B:
            v0 = statistics.mean(v for _, v in B[:20]); v1 = statistics.mean(v for _, v in B[-20:])
            print(f'== 배터리: 시작 {v0:.2f} V → 끝 {v1:.2f} V ({(v0 - v1) / ((B[-1][0] - B[0][0]) / 3600):.2f} V/시간), 최저 {min(v for _, v in B):.2f} V')


if __name__ == '__main__':
    main()
