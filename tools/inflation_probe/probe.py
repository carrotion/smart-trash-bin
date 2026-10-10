"""전역 코스트맵 팽창 반경 판정(run.sh가 부름). 도메인 42 planner로 집 지도에서 무작위 쌍 경로 → 성공·길이·장애물과의 최소 거리. python3 probe.py 0.20"""
import os, sys, math, random, json
import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
R = sys.argv[1]
im = np.array(Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../robot/sim_worlds/bitmaps/our_home.png')))[::-1, :]          # 행 0 = 아래(y 최소)
occ = im < 128; res = 0.05; ox, oy = -7.45, -4.925
H, W = occ.shape
oj, oi = np.nonzero(occ); O = np.stack([oi, oj], 1).astype(np.float32)
dist = np.zeros((H, W), np.float32)
fj, fi = np.nonzero(~occ); F = np.stack([fi, fj], 1).astype(np.float32)
for k in range(0, len(F), 2000):
    d = np.sqrt(((F[k:k + 2000, None, :] - O[None, :, :]) ** 2).sum(-1)).min(1)
    dist[fj[k:k + 2000], fi[k:k + 2000]] = d * res
def clr(x, y):
    i, j = int((x - ox) / res), int((y - oy) / res)
    return dist[j, i] if 0 <= i < W and 0 <= j < H else 0.0
# 집 안(외곽 바깥 제외): 가장 큰 연결 자유 영역만
from collections import deque
ok_ = dist > 0.16; seen = np.zeros_like(ok_); best = []
for sj, si in zip(*np.nonzero(ok_)):
    if seen[sj, si]: continue
    comp = []; q = deque([(sj, si)]); seen[sj, si] = True
    while q:
        j, i = q.popleft(); comp.append((j, i))
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = j + dj, i + di
            if 0 <= a < H and 0 <= b < W and ok_[a, b] and not seen[a, b]:
                seen[a, b] = True; q.append((a, b))
    if len(comp) > len(best): best = comp
cand = np.array(best)
print('집 안 후보 칸', len(cand), flush=True)
random.seed(7); pairs = []
while len(pairs) < 60:
    a, b = cand[random.randrange(len(cand))], cand[random.randrange(len(cand))]
    pa = (ox + (a[1] + .5) * res, oy + (a[0] + .5) * res); pb = (ox + (b[1] + .5) * res, oy + (b[0] + .5) * res)
    if math.dist(pa, pb) >= 2.0:
        pairs.append((pa, pb))
rclpy.init(); n = rclpy.create_node('probe'); ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
ac.wait_for_server(timeout_sec=20)
def ps(p):
    m = PoseStamped(); m.header.frame_id = 'map'; m.pose.position.x, m.pose.position.y = p; m.pose.orientation.w = 1.0; return m
out = []
for pa, pb in pairs:
    g = ComputePathToPose.Goal(); g.start = ps(pa); g.goal = ps(pb); g.use_start = True
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    h = f.result(); r = h.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=20)
    path = [(q.pose.position.x, q.pose.position.y) for q in r.result().result.path.poses] if r.done() else []
    if len(path) < 2:
        out.append(dict(ok=False)); continue
    L = sum(math.dist(path[k], path[k + 1]) for k in range(len(path) - 1))
    c = [clr(*q) for q in path[3:-3]] or [clr(*q) for q in path]
    out.append(dict(ok=True, len=L, straight=math.dist(pa, pb), min_clr=min(c), frac_lt021=sum(v < 0.21 for v in c) / len(c),
                    frac_lt025=sum(v < 0.25 for v in c) / len(c)))
json.dump(dict(R=R, pairs=pairs, out=out), open(f'/tmp/inflation_probe_{R}.json', 'w'), default=float)
ok = [o for o in out if o['ok']]
print(f'팽창 {R}: 성공 {len(ok)}/{len(out)}, 길이 평균 {np.mean([o["len"] for o in ok]):.2f} m, '
      f'최소 여유 중앙 {np.median([o["min_clr"] for o in ok]):.3f} m, 경로 중 여유<0.21 m 비율 평균 {np.mean([o["frac_lt021"] for o in ok]):.1%}, '
      f'<0.25 평균 {np.mean([o["frac_lt025"] for o in ok]):.1%}')
