#!/usr/bin/env python3
"""계획 실패 스냅샷 재생 — 주행기 서버가 '빈 경로' 때 떠 둔 SLAM 지도·출발·목표(docs/calibration/planfail_*/)로
같은 상황을 다시 만들어 planner 설정을 비교한다(로봇·Stage 없이, 스냅샷 하나에 수 초).

도메인 43에 map_server(스냅샷 지도) + planner_server(전역 코스트맵: 정적 지도 + 팽창 0.35, 실물 설정과 같은 값)를 planner
종류마다 띄우고, 스냅샷의 출발 → 목표로 ComputePathToPose를 부른다. 출발점 주변 0.1~0.2 m로 옮긴 계획(주행기의 복구)도 같이 본다.

    python3 tools/planfail_replay.py docs/calibration/planfail_20261010_*          # 모든 planner
    python3 tools/planfail_replay.py <폴더들> --planners navfn smac2d

planner: navfn(지금 설정, 다익스트라) / navfn_astar / navfn_tol(목표 허용 0.3 → 0.6) / smac2d / theta
⚠️ 스냅샷엔 실시간 라이다 층(obstacle_layer)이 없다 — SLAM 지도에 아직 안 들어간 장애물은 빠진다.
"""
import argparse, glob, json, math, os, signal, subprocess, sys, time

os.environ['ROS_DOMAIN_ID'] = '43'
HERE = os.path.dirname(os.path.abspath(__file__))
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

PLANNERS = {
    'navfn': 'plugin: "nav2_navfn_planner::NavfnPlanner"\n      tolerance: 0.3\n      use_astar: false\n      allow_unknown: true',
    'navfn_astar': 'plugin: "nav2_navfn_planner::NavfnPlanner"\n      tolerance: 0.3\n      use_astar: true\n      allow_unknown: true',
    'navfn_tol': 'plugin: "nav2_navfn_planner::NavfnPlanner"\n      tolerance: 0.6\n      use_astar: false\n      allow_unknown: true',
    'smac2d': 'plugin: "nav2_smac_planner::SmacPlanner2D"\n      tolerance: 0.3\n      allow_unknown: true\n      max_planning_time: 2.0\n      cost_travel_multiplier: 2.0',
    'theta': 'plugin: "nav2_theta_star_planner::ThetaStarPlanner"\n      how_many_corners: 8\n      allow_unknown: true',
}
TEMPLATE = """map_server:
  ros__parameters:
    yaml_filename: {map_yaml}
lifecycle_manager_replay:
  ros__parameters:
    autostart: true
    node_names: ["map_server", "planner_server"]
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      {plugin}
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      transform_tolerance: 0.3
      rolling_window: false
      track_unknown_space: true
      footprint: "[[0.11, 0.11], [0.11, -0.11], [-0.11, -0.11], [-0.11, 0.11]]"
      resolution: 0.05
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/map"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 2.0
        inflation_radius: 0.35
      always_send_full_costmap: true
"""


def start(map_yaml, planner):
    p = f'/tmp/planfail_replay_{planner}.yaml'
    open(p, 'w').write(TEMPLATE.format(map_yaml=map_yaml, plugin=PLANNERS[planner]))
    procs = []
    for cmd in (['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'base_link'],
                ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '--params-file', p],
                ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', p],
                ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r', '__node:=lifecycle_manager_replay', '--params-file', p]):
        procs.append(subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
    return procs


def stop(procs):
    # 자기가 띄운 프로세스 묶음만(이름으로 죽이지 않는다 — 10-10 실물 스택의 planner를 죽인 사건)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for p in procs:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(1.0)


def plan(node, ac, a, b):
    def ps(x, y):
        m = PoseStamped(); m.header.frame_id = 'map'; m.pose.position.x, m.pose.position.y = float(x), float(y); m.pose.orientation.w = 1.0
        return m
    g = ComputePathToPose.Goal(); g.start = ps(*a); g.goal = ps(*b); g.use_start = True
    t0 = time.time(); f = ac.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    if not f.done() or not f.result().accepted:
        return None, 0.0
    r = f.result().get_result_async(); rclpy.spin_until_future_complete(node, r, timeout_sec=20)
    path = [(q.pose.position.x, q.pose.position.y) for q in r.result().result.path.poses] if r.done() else []
    if len(path) < 2:
        return None, time.time() - t0
    return sum(math.dist(path[k], path[k + 1]) for k in range(len(path) - 1)), time.time() - t0


def sample_goals(d, n, seed=5, clear=0.25):
    """스냅샷 지도에서 목표 n개: 절반은 '아는 빈 칸 중 모르는 칸과 맞닿은 곳'(탐색 목표가 생기는 자리), 절반은 아는 빈 칸 아무 데나.
    점유 칸에서 clear 이상 떨어진 칸만."""
    import random
    import numpy as np
    raw = open(os.path.join(d, 'map.pgm'), 'rb').read(); parts = raw.split(b'\n', 3)
    W, H = map(int, parts[1].split()); px = np.frombuffer(parts[3], np.uint8).reshape(H, W)[::-1, :]     # 행 0 = y 최소
    y = open(os.path.join(d, 'map.yaml')).read()
    res = float(y.split('resolution:')[1].split()[0]); org = y.split('origin: [')[1].split(']')[0].split(','); ox, oy = float(org[0]), float(org[1])
    free, unk, occ = px == 254, px == 205, px == 0
    k = int(clear / res); near_occ = np.zeros_like(occ)
    oj, oi = np.nonzero(occ)
    for dj in range(-k, k + 1):
        for di in range(-k, k + 1):
            if dj * dj + di * di <= k * k:
                jj, ii = np.clip(oj + dj, 0, H - 1), np.clip(oi + di, 0, W - 1); near_occ[jj, ii] = True
    ok = free & ~near_occ
    fr = ok & (np.roll(unk, 1, 0) | np.roll(unk, -1, 0) | np.roll(unk, 1, 1) | np.roll(unk, -1, 1))
    rnd = random.Random(seed); out = []
    for mask, m in ((fr, n // 2), (ok, n - n // 2)):
        cells = np.argwhere(mask)
        for j, i in (cells[rnd.sample(range(len(cells)), min(m, len(cells)))] if len(cells) else []):
            out.append((ox + (i + 0.5) * res, oy + (j + 0.5) * res))
    return out


def stress(a, dirs):
    """스냅샷 지도마다 여러 목표로 planner 비교: 어느 planner든 경로를 낸 목표를 '갈 수 있는 목표'로 보고 planner별 성공 수를 센다."""
    rclpy.init(); node = rclpy.create_node('planfail_replay'); ac = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    res = {}   # (dir, goal index) -> {planner: (length, secs)}
    for d in dirs:
        c = json.load(open(os.path.join(d, 'case.json'))); s = c['start'][:2]
        goals = [tuple(c['goal'])] + sample_goals(d, a.goals)
        for pl in a.planners:
            procs = start(os.path.abspath(os.path.join(d, 'map.yaml')), pl)
            try:
                if not ac.wait_for_server(timeout_sec=25):
                    continue
                time.sleep(3.0)
                for gi, g in enumerate(goals):
                    res.setdefault((d, gi), {})[pl] = plan(node, ac, s, g)
            finally:
                stop(procs)
        n_any = sum(any(v[0] is not None for v in res[(d, gi)].values()) for gi in range(len(goals)))
        print(f'{os.path.basename(d)}: 목표 {len(goals)}개, 갈 수 있는 것 {n_any} | ' + ', '.join(
            f'{pl} {sum(res[(d, gi)].get(pl, (None,))[0] is not None for gi in range(len(goals)))}' for pl in a.planners), flush=True)
    reach = [k for k, v in res.items() if any(x[0] is not None for x in v.values())]
    print(f'\n=== 지도 {len(dirs)}장, 갈 수 있는 목표 {len(reach)}개(전체 {len(res)}) ===')
    for pl in a.planners:
        ok = [res[k][pl] for k in reach if pl in res[k] and res[k][pl][0] is not None]
        ms = sorted(x[1] * 1000 for x in ok)
        print(f'{pl:12s}: 성공 {len(ok)}/{len(reach)} = {len(ok) / max(len(reach), 1):.1%}' + (f', 계산 중앙 {ms[len(ms) // 2]:.0f} ms·최대 {ms[-1]:.0f} ms' if ms else ''))
    if 'navfn' in a.planners:
        for other in a.planners:
            if other == 'navfn':
                continue
            both = [(res[k]['navfn'][0], res[k][other][0]) for k in reach if res[k].get('navfn', (None,))[0] is not None and res[k].get(other, (None,))[0] is not None]
            if both:
                r = sorted(o / n for n, o in both)
                print(f'  경로 길이 {other}/navfn: 중앙 {r[len(r) // 2]:.3f}, 최대 {r[-1]:.2f} ({len(both)}쌍)')
    json.dump({f'{os.path.basename(k[0])}#{k[1]}': {pl: v[0] for pl, v in val.items()} for k, val in res.items()},
              open(f'/tmp/planfail_stress_{int(time.time())}.json', 'w'))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--planners', nargs='+', default=list(PLANNERS), choices=list(PLANNERS))
    ap.add_argument('--goals', type=int, default=0, help='지도마다 목표를 n개 더 뽑아 planner 비교(스트레스)')
    a = ap.parse_args()
    if a.goals:
        return stress(a, [d for pat in a.dirs for d in sorted(glob.glob(pat)) if os.path.exists(os.path.join(d, 'case.json'))])
    dirs = [d for pat in a.dirs for d in sorted(glob.glob(pat)) if os.path.exists(os.path.join(d, 'case.json'))]
    if not dirs:
        print('스냅샷 폴더 없음'); return 1
    rclpy.init(); node = rclpy.create_node('planfail_replay'); ac = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    rows = []
    for d in dirs:
        c = json.load(open(os.path.join(d, 'case.json'))); s, g = c['start'][:2], c['goal']
        print(f'{os.path.basename(d)}: 출발 ({s[0]:+.2f},{s[1]:+.2f}) → 목표 ({g[0]:+.2f},{g[1]:+.2f}), 직선 {math.dist(s, g):.2f} m', flush=True)
        for pl in a.planners:
            procs = start(os.path.abspath(os.path.join(d, 'map.yaml')), pl)
            try:
                if not ac.wait_for_server(timeout_sec=25):
                    print(f'   {pl:12s}: ❌ planner_server 안 뜸'); continue
                time.sleep(3.0)
                L, dt = plan(node, ac, s, g)
                shifted = None
                if L is None:   # 주행기의 복구와 같은 방식: 출발점을 0.1~0.2 m 옮겨서
                    for dd in (0.10, 0.15, 0.20):
                        for k in range(8):
                            L2, _ = plan(node, ac, (s[0] + dd * math.cos(k * math.pi / 4), s[1] + dd * math.sin(k * math.pi / 4)), g)
                            if L2 is not None:
                                shifted = (dd, k * 45, L2); break
                        if shifted:
                            break
                rows.append(dict(case=os.path.basename(d), planner=pl, ok=L is not None, length=L, secs=dt, shifted=shifted))
                print(f'   {pl:12s}: ' + (f'✅ 경로 {L:.2f} m ({dt * 1000:.0f} ms)' if L is not None else
                                         ('❌ 빈 경로' + (f' — 출발점 {shifted[0]} m/{shifted[1]}° 옮기면 {shifted[2]:.2f} m' if shifted else ' — 출발점을 옮겨도 실패'))), flush=True)
            finally:
                stop(procs)
    print('\n요약(성공/전체): ' + ', '.join(f'{pl} {sum(r["ok"] for r in rows if r["planner"] == pl)}/{sum(r["planner"] == pl for r in rows)}' for pl in a.planners))
    return 0


if __name__ == '__main__':
    sys.exit(main())
