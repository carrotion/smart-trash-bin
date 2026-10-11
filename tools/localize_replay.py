#!/usr/bin/env python3
"""저장된 지도에서 위치 잡기 — 녹화 재생으로 확인 (로봇 없이, 2026-10-11).

묻는 것: 저장한 지도(slam_toolbox 자세 그래프)를 불러 놓고 로봇이 다시 다니면, 지도 위 위치를 얼마나 정확히 따라가는가.
특히 **처음 놓은 자리가 알려 준 자리와 어긋나 있을 때**(사람이 대충 놓는다) 얼마나 어긋나도 제자리를 찾는가.

방법: 도메인 44에서 slam_toolbox의 위치 잡기 노드(localization_slam_toolbox_node — 원래 있는 기능)를 저장된 그래프로 띄우고,
녹화의 스캔·오도메트리만 재생한다(tools/replay_slam.py와 같은 방식: /tf는 틀지 않고 오도메트리에서 odom→base_link만 다시 냄).
정답은 녹화 당시 SLAM이 냈던 지도 위 자세(녹화의 /tf에서 map→odom ∘ odom→base_link). 시작 자세는 정답에 --perturb만큼 어긋나게 준다.

⚠️ 지도를 만든 그 녹화로 시험하면 낙관적이다(같은 스캔, 가구도 그대로). 기능과 시작 자세 허용 범위의 **첫 확인**으로만 읽을 것.
   다른 날 녹화·실물 재시작으로 다시 봐야 한다.

    python3 tools/localize_replay.py --graph ~/maps/home_20261011 --bag bags/20261011_155553_explore1 \
        --start-offset 600 --duration 240 --perturb 0.3,0.0,15
"""
import argparse, bisect, math, os, signal, statistics, subprocess, sys, tempfile, threading, time

os.environ['ROS_DOMAIN_ID'] = '44'
HERE = os.path.dirname(os.path.abspath(__file__))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """a ∘ b: (x, y, th) 두 변환의 합성."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def truth_from_bag(bag, t_lo, t_hi):
    """녹화의 /tf에서 지도 위 자세 (t, x, y, yaw) 목록(약 10 Hz). 시각은 녹화 당시의 헤더 시각(초)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    meta = open(os.path.join(bag, 'metadata.yaml')).read()
    storage = 'mcap' if 'storage_identifier: mcap' in meta else 'sqlite3'
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/tf']))
    mo = None; out = []; last = 0.0; first = None
    while rd.has_next():
        _, data, tns = rd.read_next(); tb = tns * 1e-9
        first = first or tb
        if tb - first > t_hi + 5:
            break
        for tr in deserialize_message(data, TFMessage).transforms:
            v = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
            st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                mo = v
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link' and mo is not None:
                if tb - first >= t_lo - 2 and st - last >= 0.1:
                    last = st; out.append((st,) + compose(mo, v))
    return first, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', required=True, help='저장된 지도 이름(확장자 없이, .posegraph/.data가 있는 경로)')
    ap.add_argument('--bag', required=True)
    ap.add_argument('--start-offset', type=float, default=0.0); ap.add_argument('--duration', type=float, default=180.0)
    ap.add_argument('--rate', type=float, default=2.0)
    ap.add_argument('--perturb', default='0,0,0', help='시작 자세를 정답에서 어긋나게: dx,dy(m),도')
    ap.add_argument('--params', default=os.path.join(HERE, '../robot/src/smart_trash_bringup/config/mapper_params_online_async.yaml'))
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    graph = os.path.expanduser(a.graph)
    if not os.path.isfile(graph + '.posegraph'):
        print('❌ 자세 그래프 없음:', graph + '.posegraph'); return 2
    print('녹화에서 정답 자세 읽는 중…', flush=True)
    bag_t0, truth = truth_from_bag(a.bag, a.start_offset, a.start_offset + a.duration)
    T = [p[0] for p in truth]
    t_start = bag_t0 + a.start_offset
    i0 = min(bisect.bisect_left(T, t_start), len(T) - 1); p0 = truth[i0]
    dx, dy, dd = (float(v) for v in a.perturb.split(','))
    start = (p0[1] + dx, p0[2] + dy, wrap(p0[3] + math.radians(dd)))
    print(f'정답 시작 자세 ({p0[1]:+.2f}, {p0[2]:+.2f}, {math.degrees(p0[3]):+.0f}°) → 알려 줄 자세 ({start[0]:+.2f}, {start[1]:+.2f}, {math.degrees(start[2]):+.0f}°)', flush=True)

    import yaml
    P = yaml.safe_load(open(a.params)); key = next(iter(P)); rp = P[key]['ros__parameters']
    rp.update(mode='localization', map_file_name=graph, map_start_pose=[float(start[0]), float(start[1]), float(start[2])], use_sim_time=True)
    rp.pop('map_start_at_dock', None)
    tmp = tempfile.mkdtemp(prefix='localize_replay_'); pf = os.path.join(tmp, 'params.yaml'); yaml.safe_dump(P, open(pf, 'w'))
    procs = []

    def spawn(cmd, log):
        p = subprocess.Popen(cmd, stdout=open(os.path.join(tmp, log), 'w'), stderr=subprocess.STDOUT, start_new_session=True); procs.append(p); return p
    res = []
    try:
        spawn(['ros2', 'run', 'slam_toolbox', 'localization_slam_toolbox_node', '--ros-args', '--params-file', pf, '-r', '__node:=slam_toolbox'], 'slam.log')
        spawn([sys.executable, os.path.join(HERE, 'odom_tf_republisher.py'), '--ros-args', '-p', 'use_sim_time:=true', '-r', '/odom:=/odometry/filtered'], 'odomtf.log')
        # 라이다 자리(정적 변환)는 녹화 맨 앞에 한 번만 들어 있어, 중간부터 틀면 빠진다(첫 시도: 스캔이 전부 버려짐) → 직접 낸다
        spawn(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0.054', '--z', '0.085', '--frame-id', 'base_link',
               '--child-frame-id', 'base_laser', '--ros-args', '-p', 'use_sim_time:=true'], 'static.log')
        time.sleep(5)
        for tr in ('configure', 'activate'):      # jazzy의 slam_toolbox는 수명 주기 노드 — 직접 띄우면 깨워 줘야 한다
            subprocess.run(['ros2', 'lifecycle', 'set', '/slam_toolbox', tr], capture_output=True, timeout=60)
        time.sleep(6)
        import rclpy
        from rclpy.parameter import Parameter
        from tf2_ros import Buffer, TransformListener
        rclpy.init(); n = rclpy.create_node('localize_replay_probe', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        tfb = Buffer(); TransformListener(tfb, n)
        stop = threading.Event()

        def probe():
            last = 0.0
            while not stop.is_set():
                rclpy.spin_once(n, timeout_sec=0.05)
                try:
                    tr = tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
                except Exception:
                    continue
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                if st - last >= 0.2:
                    last = st; res.append((st, tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation)))
        th = threading.Thread(target=probe, daemon=True); th.start()
        print(f'재생 {a.duration:.0f}초(배속 {a.rate})…', flush=True)
        play = spawn(['ros2', 'bag', 'play', a.bag, '--clock', '200', '--rate', str(a.rate), '--start-offset', str(a.start_offset),
                      '--playback-duration', str(a.duration), '--topics', '/scan', '/odometry/filtered'], 'play.log')
        play.wait(timeout=a.duration / a.rate + 120)
        time.sleep(2); stop.set(); th.join(timeout=2)
    finally:
        for p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGINT)
            except OSError:
                pass
        time.sleep(2)
        for p in reversed(procs):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except OSError:
                    pass
    if len(res) < 10:
        print(f'❌ 지도 위 자세를 거의 못 받음({len(res)}개) — 기록: {tmp}')
        print(open(os.path.join(tmp, 'slam.log')).read()[-1500:]); return 1
    E = []
    for st, x, y, yaw in res:
        i = min(bisect.bisect_left(T, st), len(T) - 1)
        if abs(T[i] - st) > 0.3:
            continue
        g = truth[i]; E.append((st - t_start, math.hypot(x - g[1], y - g[2]), abs(math.degrees(wrap(yaw - g[3])))))
    if not E:
        print('❌ 정답과 시각이 맞는 표본이 없음'); return 1
    conv = next((t for k, (t, d, w) in enumerate(E) if all(d2 <= 0.10 and w2 <= 3.0 for _, d2, w2 in E[k:k + 25])), None)
    tail = [e for e in E if conv is not None and e[0] >= conv] or E
    moved = sum(math.hypot(truth[k + 1][1] - truth[k][1], truth[k + 1][2] - truth[k][2]) for k in range(i0, min(len(truth) - 1, bisect.bisect_left(T, t_start + a.duration))))
    if not a.quiet:
        for lo, hi in ((0, 5), (5, 15), (15, 30), (30, 60), (60, 120), (120, 1e9)):
            seg = [e for e in E if lo <= e[0] < hi]
            if seg:
                print(f'  {lo:>3}~{hi if hi < 1e8 else "끝":>3}초: 위치 오차 중앙 {statistics.median(e[1] for e in seg) * 100:5.1f} cm, 최대 {max(e[1] for e in seg) * 100:5.1f} cm | 방향 오차 중앙 {statistics.median(e[2] for e in seg):4.1f}°, 최대 {max(e[2] for e in seg):4.1f}°')
    ok = conv is not None and statistics.median(e[1] for e in tail) <= 0.10
    print(f'{"✅" if ok else "❌"} 어긋남 ({dx:+.2f}, {dy:+.2f}) m, {dd:+.0f}° | 처음 오차 {E[0][1] * 100:.0f} cm·{E[0][2]:.0f}° → '
          + (f'{conv:.0f}초에 제자리(10 cm·3° 안), 그 뒤 위치 중앙 {statistics.median(e[1] for e in tail) * 100:.1f} cm·최대 {max(e[1] for e in tail) * 100:.1f} cm, 방향 최대 {max(e[2] for e in tail):.1f}°'
             if conv is not None else f'끝까지 못 찾음(마지막 오차 {E[-1][1] * 100:.0f} cm·{E[-1][2]:.0f}°)')
          + f' | 그동안 로봇 이동 {moved:.1f} m, 표본 {len(E)}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
