#!/usr/bin/env python3
"""왕복의 원인 재현 — 녹화(/scan, /map, /tf)를 실제 전역 코스트맵 + planner_server에 다시 넣고, 그때 로봇이 있던 자리마다 경로를 물어본다(도메인 44, 로봇 없이, 2026-11-01).

묻는 것: 11-01 탐색의 180° 왕복에서 주행기가 받은 경로가 걸음마다 정말 뒤집혔나, 어느 칸이 자리에 따라 막혔다 뚫렸다 하나.

    python3 tools/replay_plan_flip.py --bag bags/… --server docs/calibration/step_nav_server_….jsonl --goals 20 22 --zones ~/maps/이름.keepout.json
"""
import argparse, bisect, json, math, os, signal, subprocess, sys, tempfile, time

os.environ['ROS_DOMAIN_ID'] = '44'
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import keepout
from localize_replay import truth_from_bag
P = os.path.join(HERE, '../robot/src/smart_trash_bringup/config/nav2_params.yaml')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--bag', required=True); ap.add_argument('--server', required=True); ap.add_argument('--zones', required=True)
    ap.add_argument('--goals', type=int, nargs='+', required=True); ap.add_argument('--warm', type=float, default=25.0); ap.add_argument('--cost-png')
    a = ap.parse_args()
    R = [json.loads(l) for l in open(a.server)]; G = [R[k - 1] for k in a.goals]
    first, tr = truth_from_bag(a.bag, 0, 1e9); T = [q[0] for q in tr]
    ev = sorted(((e['t'], k, e) for k, r in enumerate(G) for e in r['log'] if e['kind'] in ('turn', 'drive')), key=lambda x: x[0])
    t_start = ev[0][0] - a.warm; t_end = ev[-1][0] + 3
    tmp = tempfile.mkdtemp(prefix='flip_'); zf = os.path.join(tmp, 'zones.json'); keepout.save(dict(version=1, zones=keepout.load(a.zones)['zones'], prev=[]), zf)
    procs = []

    def spawn(cmd, log):
        p = subprocess.Popen(cmd, stdout=open(os.path.join(tmp, log), 'w'), stderr=subprocess.STDOUT, start_new_session=True); procs.append(p); return p
    st = ['-p', 'use_sim_time:=true']
    spawn([sys.executable, os.path.join(HERE, 'keepout_server.py'), '--keep', '--file', zf, '--ros-args'] + st, 'keepout.log')
    spawn(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0.054', '--z', '0.085', '--frame-id', 'base_link', '--child-frame-id', 'base_laser', '--ros-args'] + st, 'static.log')
    spawn(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', P] + st, 'planner.log')
    spawn(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r', '__node:=lm_flip', '-p', "node_names:=['planner_server']", '-p', 'autostart:=true'] + st, 'lm.log')
    res = []
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import PoseStamped
        rclpy.init(); n = rclpy.create_node('flip_probe', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]); ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose'); cm = {}
        n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: cm.update(m=m), QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE))
        spawn(['ros2', 'bag', 'play', a.bag, '--clock', '200', '--start-offset', str(t_start - first), '--playback-duration', str(t_end - t_start + 2), '--topics', '/scan', '/map', '/tf'], 'play.log')

        def ps(x, y):
            m = PoseStamped(); m.header.frame_id = 'map'; m.pose.position.x, m.pose.position.y = float(x), float(y); m.pose.orientation.w = 1.0; return m

        def plan(s, g):
            q = ComputePathToPose.Goal(); q.start = ps(*s); q.goal = ps(*g); q.use_start = True
            f = ac.send_goal_async(q); t0 = time.time()
            while not f.done() and time.time() - t0 < 4:
                rclpy.spin_once(n, timeout_sec=0.02)
            if not f.done() or not f.result().accepted:
                return []
            r = f.result().get_result_async(); t0 = time.time()
            while not r.done() and time.time() - t0 < 6:
                rclpy.spin_once(n, timeout_sec=0.02)
            return [(w.pose.position.x, w.pose.position.y) for w in r.result().result.path.poses] if r.done() else []
        ac.wait_for_server(timeout_sec=30); i = 0; t0 = time.time(); prev = {}
        while i < len(ev) and time.time() - t0 < (t_end - t_start) + 60:
            rclpy.spin_once(n, timeout_sec=0.02); now = n.get_clock().now().nanoseconds * 1e-9
            if now < ev[i][0]:
                continue
            t, k, e = ev[i]; i += 1
            if e['kind'] != 'drive':
                continue
            j = min(bisect.bisect_left(T, t), len(T) - 1); _, x, y, yaw = tr[j]; g = G[k]['goal'][:2]; pth = plan((x, y), g)
            L = sum(math.dist(pth[q], pth[q + 1]) for q in range(len(pth) - 1)); ah = next((q for q in pth if math.dist(q, (x, y)) >= 0.4), pth[-1]) if pth else (x, y)
            hd = math.degrees(math.atan2(ah[1] - y, ah[0] - x)); diff = (hd - prev.get(k, hd) + 180) % 360 - 180; prev[k] = hd
            c = cm.get('m'); gc = None
            if c is not None:
                A = np.array(c.data, np.int8).reshape(c.info.height, c.info.width)
                np.save(os.path.join(tmp, f'cm_{i:03d}.npy'), A); gc = (c.info.origin.position.x, c.info.origin.position.y, c.info.resolution)
                print_cost = sum(1 for q in pth if A[int((q[1] - gc[1]) / gc[2]), int((q[0] - gc[0]) / gc[2])] < 0)
            else:
                print_cost = -1
            print(f'목표{a.goals[k]} 단계{e["step"]:2d} 자리({x:+.2f},{y:+.2f}) 경로 {L:5.1f} m, 0.4 m 앞 방향 {hd:+5.0f}°(직전과 {diff:+5.0f}°), 경로 중 모르는 칸 {print_cost}개, 끝({pth[-1][0] if pth else 0:+.2f},{pth[-1][1] if pth else 0:+.2f})', flush=True)
            res.append(dict(goal=a.goals[k], step=e['step'], x=x, y=y, L=L, hd=hd, path=pth[::4], cm=f'cm_{i:03d}.npy', origin=gc))
        json.dump(res, open(os.path.join(tmp, 'result.json'), 'w'))
        print('기록', tmp)
    finally:
        for sig in (signal.SIGINT, signal.SIGKILL):
            for p in reversed(procs):
                try:
                    os.killpg(p.pid, sig)
                except OSError:
                    pass
            time.sleep(1.5)


if __name__ == '__main__':
    sys.exit(main())
