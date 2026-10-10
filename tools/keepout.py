#!/usr/bin/env python3
"""금지 구역(소파·침대 밑처럼 가면 안 되는 곳) — 저장 형식과 기하 계산. ROS 없이 쓴다(2026-10-10, todo 11a).

왜: 라이다는 높이 8.5 cm 한 겹만 본다. 소파 밑은 다리 사이가 빈 공간으로 보이지만 로봇 위쪽(파이·배터리)은 걸린다
(08-21, 09-27 두 번 들어가려 함). 사람이 지도 위에 "여기는 가지 마"를 그려 준다 — 로봇청소기의 금지 구역과 같은 방식.

두 겹으로 막는다:
  1. 경로 계획: keepout_server.py가 구역을 격자(mask)로 내고 Nav2 전역 코스트맵의 KeepoutFilter가 받는다 → planner가 돌아간다
  2. 주행기: 구역 **경계를 가상의 벽**(스캔 점)으로 더한다(step_navigator._scan) → 앞 여유·회전 반경·떨어지기가 실제 벽과 똑같이 동작
구역 선은 "벽"이다: 차체가 선을 넘지 않는다(중심은 선에서 차체 반폭 0.11 m 이상).

⚠️ 좌표는 **지금 SLAM 지도 좌표**다. SLAM은 켤 때마다 로봇이 놓인 자리를 원점으로 새 지도를 그리므로, 스택을 다시 켜면
   지난 구역은 엉뚱한 자리를 가리킨다 → 서버가 켜질 때 지난 구역을 prev로 치워 두고 비운다(같은 자리·같은 방향에서
   출발했을 때만 사용자가 다시 불러 쓴다). 저장된 지도에서 위치를 잡는 기능이 생기면 지도 이름별로 묶는다.

파일(~/maps/keepout_zones.json): {"version": n, "updated": t, "zones": [{"id": 1, "name": "소파", "poly": [[x, y], ...]}], "prev": [...]}
"""
import json, math, os, time

FILE = os.environ.get('KEEPOUT_FILE') or os.path.expanduser('~/maps/keepout_zones.json')     # 시뮬·시험은 KEEPOUT_FILE로 다른 파일을 쓴다
MAX_ZONES, MAX_PTS, LIMIT, MIN_AREA = 30, 12, 19.0, 0.01
INSCRIBED, RADIUS, CSF = 0.11, 0.35, 2.0      # 차체 반폭, 전역 팽창 반경·기울기(nav2_params.yaml global_costmap과 같은 값)


def load(path=FILE):
    try:
        d = json.load(open(path))
    except (OSError, ValueError):
        d = {}
    return dict(version=int(d.get('version', 0)), updated=d.get('updated', 0.0), zones=d.get('zones') or [], prev=d.get('prev') or [])


def save(d, path=FILE):
    """임시 파일에 쓰고 바꿔치기(읽는 쪽이 반쯤 쓴 파일을 보지 않게). version을 올린다."""
    d = dict(d, version=int(d.get('version', 0)) + 1, updated=time.time())
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(d, f, ensure_ascii=False)
    os.replace(tmp, path)
    return d


def area(poly):
    return abs(sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))) / 2


def clean_zones(zones):
    """밖(화면)에서 온 구역 목록을 검사해 깨끗한 목록으로. 문제가 있으면 ValueError(사람이 읽을 이유)."""
    if not isinstance(zones, list) or len(zones) > MAX_ZONES:
        raise ValueError(f'구역은 {MAX_ZONES}개까지')
    out = []
    for k, z in enumerate(zones):
        poly = z.get('poly') if isinstance(z, dict) else None
        if not isinstance(poly, list) or not (3 <= len(poly) <= MAX_PTS):
            raise ValueError(f'구역 {k + 1}: 꼭짓점은 3~{MAX_PTS}개')
        try:
            poly = [(round(float(p[0]), 3), round(float(p[1]), 3)) for p in poly]
        except (TypeError, ValueError, IndexError):
            raise ValueError(f'구역 {k + 1}: 좌표가 숫자가 아님')
        if any(not (math.isfinite(x) and math.isfinite(y)) or abs(x) > LIMIT or abs(y) > LIMIT for x, y in poly):
            raise ValueError(f'구역 {k + 1}: 좌표가 지도 범위(±{LIMIT:.0f} m) 밖')
        if area(poly) < MIN_AREA:
            raise ValueError(f'구역 {k + 1}: 너무 작음(10 × 10 cm 이상)')
        out.append(dict(id=int(z.get('id', k + 1)), name=str(z.get('name', ''))[:20], poly=[list(p) for p in poly]))
    return out


def inside(x, y, poly):
    n = len(poly); c = False
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def dist(x, y, poly):
    """점에서 구역까지 거리(m). 구역 안이면 0."""
    if inside(x, y, poly):
        return 0.0
    best = float('inf'); n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]; dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy))) if dx or dy else 0.0
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return best


def nearest(x, y, zones):
    """(가장 가까운 구역까지 거리, 그 구역) — 구역이 없으면 (inf, None)."""
    best = (float('inf'), None)
    for z in zones:
        d = dist(x, y, z['poly'])
        if d < best[0]:
            best = (d, z)
    return best


def boundary_points(zones, step=0.03):
    """구역 경계를 step 간격으로 찍은 점들(지도 좌표) — 주행기가 가상의 벽으로 쓴다."""
    pts = []
    for z in zones:
        poly = z['poly']; n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]; L = math.hypot(x2 - x1, y2 - y1); k = max(1, int(math.ceil(L / step)))
            pts.extend((x1 + (x2 - x1) * j / k, y1 + (y2 - y1) * j / k) for j in range(k))
    return pts


def build_mask(zones, res=0.05, half=20.0, inscribed=INSCRIBED, radius=RADIUS, csf=CSF):
    """KeepoutFilter용 격자. 원점 (−half, −half), 한 변 2·half. 값: 구역 안과 차체 반폭(inscribed) 안은 100(통행 불가),
    그 밖 radius까지는 Nav2 팽창 층과 같은 식의 비용(벽 옆을 피하듯 구역 옆도 덜 선호), 나머지 0.
    ⚠️ 크기를 고정으로 크게 잡는 이유: KeepoutFilter는 새 격자가 오면 격자의 칸 수만큼 코스트맵을 다시 칠한다 — 구역을 지웠을 때
       예전 자국이 남지 않으려면 격자가 코스트맵 전체를 덮어야 한다. 반환 (numpy int8 2차원[y][x], 원점 x, 원점 y)."""
    import numpy as np
    n = int(round(2 * half / res)); m = np.zeros((n, n), np.int8)
    for z in zones:
        P = np.array(z['poly'], float)
        i0, j0 = (int((P[:, k].min() - radius - res + half) / res) for k in (0, 1))
        i1, j1 = (int((P[:, k].max() + radius + res + half) / res) + 1 for k in (0, 1))
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, n), min(j1, n)
        if i1 <= i0 or j1 <= j0:
            continue
        X, Y = np.meshgrid(-half + (np.arange(i0, i1) + 0.5) * res, -half + (np.arange(j0, j1) + 0.5) * res)
        ins = np.zeros(X.shape, bool); d = np.full(X.shape, np.inf)
        for a in range(len(P)):
            (x1, y1), (x2, y2) = P[a], P[(a + 1) % len(P)]; dx, dy = x2 - x1, y2 - y1
            with np.errstate(divide='ignore', invalid='ignore'):
                ins ^= ((y1 > Y) != (y2 > Y)) & (X < dx * (Y - y1) / (dy if dy else 1e-12) + x1)
            t = np.clip(((X - x1) * dx + (Y - y1) * dy) / max(dx * dx + dy * dy, 1e-12), 0, 1)
            d = np.minimum(d, np.hypot(X - x1 - t * dx, Y - y1 - t * dy))
        d[ins] = 0.0
        v = np.where(d <= inscribed, 100, np.where(d <= radius, np.clip(np.round(252 * np.exp(-csf * (d - inscribed)) / 2.54), 1, 99), 0)).astype(np.int8)
        m[j0:j1, i0:i1] = np.maximum(m[j0:j1, i0:i1], v)
    return m, -half, -half
