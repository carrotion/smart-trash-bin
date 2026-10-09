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
from nav2_msgs.action import NavigateToPose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step_navigator import StepNavigator

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--action', default='step_navigate_to_pose')
    ap.add_argument('--odom-topic', default='/odometry/filtered'); ap.add_argument('--gyro-from-odom', action='store_true')
    ap.add_argument('--cmd-topic', default='/cmd_vel_nav'); ap.add_argument('--step', type=float, default=0.4)
    ap.add_argument('--sim-time', action='store_true', help='use_sim_time(시뮬) — TF 조회 시각을 /clock에 맞춘다')
    ap.add_argument('--label', default='')
    a = ap.parse_args()
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

    def on_goal(req):
        if busy.is_set() or req.pose.header.frame_id not in ('map', ''):
            return GoalResponse.REJECT
        busy.set()
        return GoalResponse.ACCEPT

    def execute(gh):
        done = threading.Event(); box = {}
        jobs.put((gh.request.pose.pose.position.x, gh.request.pose.pose.position.y, done, box))
        while not done.wait(0.1):
            if stopping.is_set():
                return NavigateToPose.Result()
            if gh.is_cancel_requested:
                nav.cancel = True
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
            try:
                r = nav.go((x, y, None)) if here else dict(ok=False, msg='map→base_link TF 없음', steps=0, secs=0.0, final_err_m=float('nan'))
            except Exception as e:   # 주행기가 죽어도 목표를 준 쪽이 영영 기다리지 않게 실패로 돌려준다
                if not rclpy.ok():
                    break
                r = dict(ok=False, msg=f'주행기 예외: {e!r}', steps=-1, secs=0.0, final_err_m=float('nan'))
            print(f'   → {"✅" if r["ok"] else "❌"} {r["msg"]} | 단계 {r["steps"]}, {r["secs"]:.1f} s, 오차 {r["final_err_m"] * 100:.1f} cm', flush=True)
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
