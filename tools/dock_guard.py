"""바닥 시험이 도크 위에서 돌고 있지 않은지 확인하는 검사.

⚠️ 2026-09-26에 두 번이나 바닥 전용 시험을 도크 위에서 돌렸다(역회전 시험, 출발 지연 시험).
사용자가 로봇을 옮긴 뒤 위치를 다시 묻지 않은 탓이다. 도크 위에선 바퀴가 공중에서 헛돌아
엔코더는 움직이는데 로봇은 그대로다 → 결과가 조용히 무의미해진다.

검사: 첫 동작 전후 라이다 스캔을 각도 칸으로 맞춰 거리 변화의 중앙값을 본다.
엔코더로 min_enc_m 이상 움직였는데 스캔 변화가 max_scan_m 미만이면 도크(또는 바퀴 헛돎)로 판정.
실측(09-26): 도크 정지 스캔끼리 최대 0.1 cm, 녹화 직진 208쌍(0.09~0.59 m)은 최소 1.5 cm·중앙 32.7 cm
→ 기준 1 cm(2 cm로 두면 직진 2건을 도크로 오판).

    from dock_guard import scan_change, looks_on_dock
    s0 = S['scan']; ...첫 동작...; s1 = S['scan']
    bad, msg = looks_on_dock(enc_dist_m, s0, s1)
    if bad: print(msg); 중단
"""
import math

BINS = 360


def _binned(scan):
    out = [None] * BINS
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min <= r <= scan.range_max) or r < 0.15:
            continue
        a = (scan.angle_min + i * scan.angle_increment) % (2 * math.pi)
        k = int(a / (2 * math.pi) * BINS) % BINS
        if out[k] is None or r < out[k]:
            out[k] = r
    return out


def scan_change(scan_a, scan_b):
    """두 스캔의 같은 각도 칸 거리 차이 |Δr|의 중앙값(m). 공통 칸이 적으면 None."""
    a, b = _binned(scan_a), _binned(scan_b)
    d = sorted(abs(x - y) for x, y in zip(a, b) if x is not None and y is not None)
    if len(d) < 30:
        return None
    return d[len(d) // 2]


def looks_on_dock(enc_dist_m, scan_a, scan_b, min_enc_m=0.08, max_scan_m=0.01):
    """(도크 의심 여부, 설명). 엔코더 이동이 작으면 판정 보류(False)."""
    ch = scan_change(scan_a, scan_b)
    if ch is None:
        return False, '스캔 비교 불가(공통 각도 부족) — 판정 보류'
    if abs(enc_dist_m) >= min_enc_m and ch < max_scan_m:
        return True, (f'❌ 도크 의심: 엔코더 {enc_dist_m * 100:.1f} cm 이동인데 라이다 변화 {ch * 100:.1f} cm '
                      f'— 바퀴가 헛돌고 있다. 로봇 위치(도크/바닥)를 사용자에게 확인할 것')
    return False, f'이동 확인: 엔코더 {enc_dist_m * 100:.1f} cm, 라이다 변화 {ch * 100:.1f} cm'
