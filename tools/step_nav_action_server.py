#!/usr/bin/env python3
"""4층 주행기(step_navigator)를 Nav2 `NavigateToPose` 액션으로 내놓는다 — 탐색기가 목표를 줄 수 있게.

frontier_exploration_ros2는 목표를 `navigate_to_pose_action_name` 액션으로 보낸다(기본 navigate_to_pose = Nav2 연속 주행).
이 서버를 다른 이름(기본 step_navigate_to_pose)으로 띄우고 탐색기의 그 파라미터만 바꾸면, 탐색기는 그대로 두고
움직임만 멈춤-확인-이동으로 바뀐다. Nav2에서는 planner(경로)와 collision_monitor(3층)만 쓰인다.

    python3 tools/step_nav_action_server.py                                        # 실물
    python3 tools/step_nav_action_server.py --odom-topic /odom --gyro-from-odom    # 시뮬(Stage, use_sim_time)

구조: 주행기는 자기 노드를 직접 돌리며(spin_once) 움직이므로 액션 콜백 안에서 부를 수 없다. 그래서 액션 서버 노드는
따로 스레드에서 돌고, 받은 목표를 주 스레드에 넘겨 한 번에 하나씩 실행한다. 실행 중에 온 새 목표는 거절한다.
목표의 도착 방향은 쓰지 않는다(탐색 목표는 위치만 의미가 있고, 방향을 맞추면 큰 회전이 한 번 더 든다).
"""
import argparse, json, math, os, queue, sys, threading, time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step_navigator import StepNavigator

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def remaining_estimate(goal, live, path_remaining, path_pose):
    """feedback용 남은 거리(m). 경로 길이 기준: 마지막 계획 때의 남은 경로 길이에서 그 뒤 움직인 만큼 뺀다(직선거리보다 작아지진 않게).
    직선거리만 쓰면 벽을 돌아가는 동안 거리가 늘어 탐색기의 "진행 없음" 감시가 정상 목표를 끊는다. 경로를 아직 안 받았으면 직선거리."""
    line = math.hypot(goal[0] - live[0], goal[1] - live[1])
    if path_remaining is None or path_pose is None:
        return line
    return max(line, path_remaining - math.hypot(live[0] - path_pose[0], live[1] - path_pose[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--action', default='step_navigate_to_pose')
    ap.add_argument('--odom-topic', default='/odometry/filtered'); ap.add_argument('--gyro-from-odom', action='store_true')
    ap.add_argument('--cmd-topic', default='/cmd_vel_nav'); ap.add_argument('--step', type=float, default=0.4)
    ap.add_argument('--sim-time', action='store_true', help='use_sim_time(시뮬) — TF 조회 시각을 /clock에 맞춘다')
    ap.add_argument('--label', default='')
    ap.add_argument('--dump-maps', default=os.environ.get('STEP_NAV_DUMP_MAPS', ''),
                    help='목표를 받을 때마다 그 순간의 SLAM 지도·위치·목표를 이 폴더에 떠 둔다(planner 비교용, tools/planfail_replay.py)')
    ap.add_argument('--fail-near', default='', help='(시뮬 도메인 전용) x,y,r,초 — 이 원 안의 목표는 움직이지 않고 그 시간 뒤 실패로 돌려준다. 닫힌 문처럼 안 되는 곳을 탐색기가 어떻게 다루는지 보는 시험')
    a = ap.parse_args()
    fail_near = tuple(float(v) for v in a.fail_near.split(',')) if a.fail_near else None
    if fail_near and os.environ.get('ROS_DOMAIN_ID', '0') == '0':      # 실물은 도메인 0
        print('❌ --fail-near는 시뮬 도메인 전용'); return 2
    rclpy.init()
    sim = [Parameter('use_sim_time', Parameter.Type.BOOL, True)] if a.sim_time else []
    n = rclpy.create_node('step_navigator', parameter_overrides=sim)
    nav = StepNavigator(n, cmd_topic=a.cmd_topic, odom_topic=a.odom_topic, gyro_from_odom=a.gyro_from_odom, step_m=a.step)
    srv_node = rclpy.create_node('step_nav_action_server', parameter_overrides=sim)
    jobs = queue.Queue(maxsize=1); busy = threading.Event(); stopping = threading.Event()
    # ⚠️ 10-09 시뮬 첫 실행: 서버가 뜨자마자 온 첫 목표가 'map→base_link TF 없음'으로 실패(TF가 아직 안 채워짐)
    #   → 위치가 잡힌 뒤에 액션 서버를 연다(탐색기는 서버가 뜰 때까지 기다린다)
    t0 = time.time()
    while nav.map_pose() is None or nav.S['pose'] is None or nav.S['scan'] is None:
        nav.spin(0.2)
        if time.time() - t0 > 60:
            print('❌ 60초 동안 map→base_link TF·odom·scan이 안 옴 — SLAM/Nav2 스택이 떠 있나?'); return 1

    # 계획 실패 진단용: 최신 SLAM 지도를 들고 있다가 '빈 경로'로 끝난 목표마다 지도·출발·목표를 떠 둔다(같은 목표는 한 번만).
    #   ⚠️ 10-10: Stage에서 특정 목표들이 '빈 경로'로 실패하는데, 탐색 도중의 불완전한 SLAM 지도에서만 나서 완성 지도로는
    #   재현이 안 됐다 → 그 순간의 지도가 있어야 planner 설정·다른 planner를 같은 상황에서 비교할 수 있다
    from rclpy.qos import QoSProfile, DurabilityPolicy
    latest_map = {}
    srv_node.create_subscription(OccupancyGrid, '/map', lambda m: latest_map.update(m=m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    dumped = set()

    def dump_planfail(k, goal, start, out_dir=None):
        m = latest_map.get('m')
        if m is None or start is None:
            return None
        if out_dir is None:
            key = (round(goal[0], 1), round(goal[1], 1))
            if key in dumped or len(dumped) >= 12:
                return None
            dumped.add(key)
            d = os.path.join(REPO, 'docs/calibration', f'planfail_{time.strftime("%Y%m%d_%H%M%S")}{("_" + a.label) if a.label else ""}')
        else:
            d = os.path.join(out_dir, f'goal_{k:03d}')
        os.makedirs(d, exist_ok=True)
        W, H, res = m.info.width, m.info.height, m.info.resolution
        # map_server가 읽는 PGM(trinary): 점유 0, 빈 칸 254, 모름 205. 행은 위(y 큰 쪽)부터
        px = bytearray(W * H)
        for j in range(H):
            row = m.data[(H - 1 - j) * W:(H - j) * W]
            for i, v in enumerate(row):
                px[j * W + i] = 205 if v < 0 else (0 if v >= 65 else (254 if v <= 25 else 205))
        with open(os.path.join(d, 'map.pgm'), 'wb') as f:
            f.write(f'P5\n{W} {H}\n255\n'.encode()); f.write(bytes(px))
        with open(os.path.join(d, 'map.yaml'), 'w') as f:
            f.write(f'image: map.pgm\nmode: trinary\nresolution: {res}\norigin: [{m.info.origin.position.x}, {m.info.origin.position.y}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        json.dump(dict(k=k, goal=goal, start=start, map_stamp=m.header.stamp.sec), open(os.path.join(d, 'case.json'), 'w'))
        return d

    def on_goal(req):
        if busy.is_set() or req.pose.header.frame_id not in ('map', ''):
            return GoalResponse.REJECT
        busy.set()
        return GoalResponse.ACCEPT

    def live_pose():
        # 액션 스레드에서 부른다 — 기다리지 않는 조회만(주 스레드가 TF를 채운다)
        try:
            t = nav.tfb.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except Exception:
            return None
        return (t.translation.x, t.translation.y, t.rotation)

    def execute(gh):
        done = threading.Event(); box = {}
        goal = (gh.request.pose.pose.position.x, gh.request.pose.pose.position.y)
        nav.path_remaining = None; nav.path_pose = None      # 앞 목표의 경로 값이 이번 feedback에 섞이지 않게
        jobs.put((goal[0], goal[1], done, box)); t_start = time.time(); t_fb = 0.0
        while not done.wait(0.1):
            if stopping.is_set():
                return NavigateToPose.Result()
            if gh.is_cancel_requested:
                nav.cancel = True
            # ⚠️ 10-09: feedback이 없어 탐색기의 "진행 없음" 감시(기본 20초)를 600초로 늘려 꺼 두다시피 했다 → 0.5초마다 남은 거리를 보낸다
            #   ⚠️ 10-10 시뮬: 경로를 받기 전 직선거리(1.45 m)를 먼저 보냈더니, 벽을 돌아가는 실제 경로(약 5 m)가 그보다 커서
            #   탐색기가 "가장 작은 남은 거리가 안 준다"며 35초에 정상 목표를 취소 → 첫 경로가 나온 뒤부터만 보낸다
            if time.time() - t_fb >= 0.5 and nav.path_remaining is not None:
                t_fb = time.time(); lp = live_pose()
                if lp is not None:
                    fb = NavigateToPose.Feedback()
                    fb.current_pose.header.frame_id = 'map'; fb.current_pose.header.stamp = srv_node.get_clock().now().to_msg()
                    fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = lp[0], lp[1]; fb.current_pose.pose.orientation = lp[2]
                    fb.distance_remaining = float(remaining_estimate(goal, lp, nav.path_remaining, nav.path_pose))
                    el = time.time() - t_start
                    fb.navigation_time.sec = int(el); fb.navigation_time.nanosec = int((el % 1) * 1e9)
                    try:
                        gh.publish_feedback(fb)
                    except Exception:
                        pass
        r = box.get('r', dict(ok=False, msg='결과 없음'))
        res = NavigateToPose.Result()
        if gh.is_cancel_requested:
            gh.canceled()
        elif r['ok']:
            gh.succeed()
        else:
            res.error_code = 1; res.error_msg = r['msg']; gh.abort()
        busy.clear()
        return res

    ActionServer(srv_node, NavigateToPose, a.action, execute, goal_callback=on_goal,
                 cancel_callback=lambda _gh: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(srv_node)
    threading.Thread(target=ex.spin, daemon=True).start()

    out = os.path.join(REPO, 'docs/calibration', f'step_nav_server_{time.strftime("%Y%m%d_%H%M%S")}{("_" + a.label) if a.label else ""}.jsonl')
    print(f'주행기 액션 서버 `{a.action}` 대기 — 기록 {out}', flush=True)
    k = 0
    try:
        while rclpy.ok():
            try:
                x, y, done, box = jobs.get_nowait()
            except queue.Empty:
                nav.spin(0.05); continue
            k += 1; nav.cancel = False; nlog = len(nav.log)
            here = nav.map_pose()
            print(f'[목표 {k}] ({x:+.2f},{y:+.2f})' + (f' ← 지금 ({here[0]:+.2f},{here[1]:+.2f}), {math.hypot(x - here[0], y - here[1]):.2f} m' if here else ''), flush=True)
            if a.dump_maps:
                dump_planfail(k, (x, y), here, out_dir=a.dump_maps)
            try:
                if fail_near and math.hypot(x - fail_near[0], y - fail_near[1]) <= fail_near[2]:
                    nav.path_remaining = 5.0; nav.path_pose = here; te = time.time()
                    while time.time() - te < fail_near[3] and not nav.cancel:
                        nav.spin(0.1)
                    r = dict(ok=False, msg='시험: 닫힌 문 흉내', steps=0, secs=time.time() - te, final_err_m=0.0)
                else:
                    r = nav.go((x, y, None)) if here else dict(ok=False, msg='map→base_link TF 없음', steps=0, secs=0.0, final_err_m=float('nan'))
            except Exception as e:   # 주행기가 죽어도 목표를 준 쪽이 영영 기다리지 않게 실패로 돌려준다
                if not rclpy.ok():
                    break
                r = dict(ok=False, msg=f'주행기 예외: {e!r}', steps=-1, secs=0.0, final_err_m=float('nan'))
            print(f'   → {"✅" if r["ok"] else "❌"} {r["msg"]} | 단계 {r["steps"]}, {r["secs"]:.1f} s, 오차 {r["final_err_m"] * 100:.1f} cm', flush=True)
            if not r['ok'] and '빈 경로' in r['msg']:
                dd = dump_planfail(k, (x, y), nav.map_pose())
                if dd:
                    print(f'   계획 실패 스냅샷: {dd}', flush=True)
            with open(out, 'a') as f:
                f.write(json.dumps(dict(k=k, t=round(time.time(), 1), goal=(x, y), start=here, result=r, log=nav.log[nlog:]),
                                   ensure_ascii=False, default=str) + '\n')
            box['r'] = r; done.set()
    except KeyboardInterrupt:
        pass
    finally:
        # ⚠️ 10-09 시뮬 첫 실행: SIGTERM 뒤 프로세스가 안 끝남 — 목표를 기다리던 액션 스레드가 남아서. 그 스레드부터 풀어 준다
        stopping.set()
        if rclpy.ok():   # 종료 신호로 컨텍스트가 이미 내려갔으면 발행 불가(구동 노드의 명령 끊김 정지 0.5초가 세운다)
            for _ in range(5):
                nav.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
        ex.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
