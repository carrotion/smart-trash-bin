#!/usr/bin/env python3
"""4층 주행 방식 — 멈춤-확인-이동 목표 주행기 (docs/motion_foundation.md).

1층에서 합격한 동작만 조합한다: ④ 각도 회전(turn_to_angle.TurnToAngle), ②' 정해진 거리 직진(drive_distance.DriveDistance).
Nav2는 **경로 계획만**(planner_server의 ComputePathToPose) 쓰고 움직임 제어(controller)는 쓰지 않는다.

한 단계:
  1. 멈춤·안정(로봇 정지 확인 + settle_s) → SLAM 위치(map→base_link TF) 읽기
  2. 목표까지 xy_tol 안이면: 목표 방향이 있으면 ④로 맞추고 끝
  3. 경로 계획 → 경로 위 step_m 앞 지점(carrot)
  4. carrot 방향과 turn_tol 넘게 어긋나면 ④로 돌기
  5. 가는 쪽 통로 여유 확인(부족하면 경로 다시 받기, 연속 blocked_max회면 실패)
  6. ②'로 min(step_m, carrot까지 거리) 직진
명령은 기본으로 /cmd_vel_nav에 낸다 → collision_monitor(3층: 속도별 정지 구역, 스캔 0.5초 끊김 정지)를 거쳐 /cmd_vel.

실물: tools/nav_test_stack.sh start 후(탐색기 없음)
    python3 tools/step_navigator.py --front 1.0 --left 0.0            # 로봇 기준 상대 목표
    python3 tools/step_navigator.py --map-x 1.2 --map-y -0.4 [--map-yaw 90]
시뮬(도메인 42, Stage): --odom-topic /odom --gyro-from-odom
"""
import argparse, json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from turn_to_angle import TurnToAngle, yaw_of, wrap
from drive_distance import DriveDistance
from speed_floor_sweep import corridor_clearance


class StepNavigator:
    def __init__(self, node, cmd_topic='/cmd_vel_nav', odom_topic='/odometry/filtered', gyro_from_odom=False,
                 step_m=0.4, xy_tol=0.05, yaw_tol_deg=5.0, turn_tol_deg=5.0, settle_s=0.5, blocked_max=3,
                 clear_margin=0.25):
        self.n = node
        self.step_m, self.xy_tol, self.yaw_tol = step_m, xy_tol, math.radians(yaw_tol_deg)
        self.turn_tol, self.settle_s, self.blocked_max, self.clear_margin = math.radians(turn_tol_deg), settle_s, blocked_max, clear_margin
        self.S = {'pose': None, 'v': 0.0, 'gz': 0.0, 'scan': None}
        node.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        if gyro_from_odom:
            self._gyro_from_odom = True
        else:
            self._gyro_from_odom = False
            node.create_subscription(Imu, '/imu/data', lambda m: self.S.update(gz=m.angular_velocity.z), 100)
        node.create_subscription(LaserScan, '/scan', lambda m: self.S.update(scan=m), 5)
        self.pub = node.create_publisher(Twist, cmd_topic, 10)
        self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, node)
        self.planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        self.turner = TurnToAngle(self.pub, self.spin, lambda: self.S['pose'][2], lambda: self.S['gz'])
        self.driver = DriveDistance(self.pub, self.spin, lambda: self.S['pose'], lambda: self.S['v'], clear_fn=self._clear_now)
        self.log = []

    def _on_odom(self, m):
        p = m.pose.pose
        self.S['pose'] = (p.position.x, p.position.y, yaw_of(p.orientation)); self.S['v'] = m.twist.twist.linear.x
        if self._gyro_from_odom:
            self.S['gz'] = m.twist.twist.angular.z

    def spin(self, t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(self.n, timeout_sec=0.005)

    def _clear_now(self, sgn):
        """직진 중 매 주기: 가는 쪽 통로 0.2 m 안 장애물 여부(collision_monitor와 별개의 한 겹).
        한 장 잡음 걸러내기(0.15초 지속)는 1층 drive_distance 안에서 한다(09-27 층 정리)."""
        return self.S['scan'] is not None and corridor_clearance(self.S['scan'], sgn > 0, 0.0 if sgn > 0 else 0.1) > 0.2

    def _corridor_pts(self, room, lidar_x=0.054, half=0.11, corridor=0.10):
        """앞 통로(corridor_clearance와 같은 범위)에서 차체 앞끝으로부터 room + 5 cm 안의 점 (x, y, 거리) — 진단 기록용."""
        m = self.S['scan']; out = []
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
            if (abs(x) <= half + 0.005 and abs(y) <= half + 0.005) or abs(y) > half + corridor or x < half:
                continue
            if x - half <= room + 0.05:
                out.append((round(x, 3), round(y, 3), round(r, 3)))
        return out[:20]

    def map_pose(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def plan(self, start, goal):
        if not self.planner.wait_for_server(timeout_sec=5.0):
            return None, 'planner 없음'
        g = ComputePathToPose.Goal()
        g.goal = PoseStamped(); g.goal.header.frame_id = 'map'
        g.goal.pose.position.x, g.goal.pose.position.y = goal[0], goal[1]
        gy = goal[2] if goal[2] is not None else 0.0
        g.goal.pose.orientation.z, g.goal.pose.orientation.w = math.sin(gy / 2), math.cos(gy / 2)
        g.start = PoseStamped(); g.start.header.frame_id = 'map'
        g.start.pose.position.x, g.start.pose.position.y = start[0], start[1]
        g.start.pose.orientation.z, g.start.pose.orientation.w = math.sin(start[2] / 2), math.cos(start[2] / 2)
        g.use_start = True
        f = self.planner.send_goal_async(g)
        t0 = time.time()
        while not f.done() and time.time() - t0 < 5:
            self.spin(0.02)
        if not f.done() or not f.result().accepted:
            return None, '경로 요청 거부'
        r = f.result().get_result_async(); t0 = time.time()
        while not r.done() and time.time() - t0 < 10:
            self.spin(0.02)
        if not r.done():
            return None, '경로 계산 시간 초과'
        path = [(ps.pose.position.x, ps.pose.position.y) for ps in r.result().result.path.poses]
        return (path if len(path) >= 2 else None), ('' if len(path) >= 2 else '빈 경로')

    def carrot(self, path, here):
        """경로에서 here와 가장 가까운 점부터 한 걸음 앞 지점. 걸음 = 남은 경로를 step_m 이하로 **균등 분할**한 길이.
        ⚠️ 09-27 합격 시험: step_m(0.4)로 자르면 1.28 m 복귀가 0.4×3 뒤 8 cm 남기고 xy_tol(10 cm) 안이라 멈춤
        — 3회 모두 실제 복귀 9.7~11.0 cm(SLAM도 7.8~8.3 cm로 알고 있었음). 균등 분할이면 마지막 걸음이 목표에 떨어진다."""
        i0 = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - here[0], path[i][1] - here[1]))
        segs = [math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]) for i in range(i0, len(path) - 1)]
        step = self.step_m
        if sum(segs) > 0:
            step = sum(segs) / math.ceil(sum(segs) / self.step_m - 1e-9)
        acc = 0.0
        for k, seg in enumerate(segs):
            i = i0 + k
            if acc + seg >= step:
                f = (step - acc) / seg if seg > 0 else 0
                return (path[i][0] + f * (path[i + 1][0] - path[i][0]), path[i][1] + f * (path[i + 1][1] - path[i][1]))
            acc += seg
        return path[-1]

    def settle(self):
        t0 = time.time(); calm = None
        while time.time() - t0 < 3.0:
            self.spin(0.02)
            if abs(self.S['v']) < 0.01 and abs(self.S['gz']) < 0.03:
                calm = calm or time.time()
                if time.time() - calm >= self.settle_s:
                    return
            else:
                calm = None

    def go(self, goal, max_steps=40, timeout=180.0):
        """goal=(x, y, yaw 또는 None) map 좌표. 반환 dict(ok, msg, steps, secs, final_err_m, final_yaw_err_deg)."""
        t0 = time.time(); blocked = 0
        for step in range(max_steps):
            if time.time() - t0 > timeout:
                return self._done(False, '시간 초과', step, t0, goal)
            self.settle()
            here = self.map_pose()
            if here is None:
                return self._done(False, 'map→base_link TF 없음', step, t0, goal)
            dist = math.hypot(goal[0] - here[0], goal[1] - here[1])
            if dist <= self.xy_tol:
                if goal[2] is not None:
                    dyaw = wrap(goal[2] - here[2])
                    if abs(dyaw) > self.yaw_tol:
                        r = self.turner.turn_by(dyaw); self._note(step, 'final_turn', math.degrees(dyaw), r)
                return self._done(True, '도착', step, t0, goal)
            path, why = self.plan(here, goal)
            if path is None:
                blocked += 1; self._note(step, 'plan_fail', why, None)
                if blocked >= self.blocked_max:
                    return self._done(False, f'경로 실패 {blocked}회: {why}', step, t0, goal)
                self.spin(1.0); continue
            c = self.carrot(path, here)
            seg = min(self.step_m + 0.01, math.hypot(c[0] - here[0], c[1] - here[1]))   # 균등 분할 걸음은 step_m 이하(반올림 여유 1 cm)
            head = math.atan2(c[1] - here[1], c[0] - here[0])
            dyaw = wrap(head - here[2])
            if abs(dyaw) > self.turn_tol:
                r = self.turner.turn_by(dyaw); self._note(step, 'turn', math.degrees(dyaw), r)
                if not r['ok']:
                    return self._done(False, f'회전 실패: {r["msg"]}', step, t0, goal)
                self.settle()
            room = corridor_clearance(self.S['scan'], True)
            if room < seg + self.clear_margin:
                # ⚠️ 09-27: 병 옆 통과 뒤 "앞 여유 0"으로 멈췄는데 무엇이 막았는지 남기지 않아 원인을 못 가렸다
                #   (사용자: 그 자리엔 아무것도 없었음, 병은 20~30 cm 옆). 막은 점들을 base_link 좌표로 기록
                blocked += 1; self._note(step, 'blocked', dict(room=round(room, 2), pts=self._corridor_pts(room)), None)
                if blocked >= self.blocked_max:
                    return self._done(False, f'앞 여유 부족 {blocked}회({room:.2f} m)', step, t0, goal)
                self.spin(1.0); continue
            r = self.driver.drive_by(seg); self._note(step, 'drive', round(seg, 3), r)
            if not r['ok']:
                blocked += 1
                if blocked >= self.blocked_max:
                    return self._done(False, f'직진 실패: {r["msg"]}', step, t0, goal)
            else:
                blocked = 0
        return self._done(False, f'단계 {max_steps}회 초과', max_steps, t0, goal)

    def _note(self, step, kind, arg, r):
        e = dict(step=step, kind=kind, arg=arg, t=round(time.time(), 2))
        if r:
            e.update({k: v for k, v in r.items() if k in ('ok', 'msg', 'err_deg', 'err_m', 'fixes', 'secs')})
        self.log.append(e)
        print(f'  단계 {step}: {kind} {arg}' + (f' → {"✅" if r.get("ok") else "❌"} {r.get("msg","")}' if r else ''), flush=True)

    def _done(self, ok, msg, steps, t0, goal):
        for _ in range(5):
            self.pub.publish(Twist()); self.spin(0.03)
        self.settle()
        here = self.map_pose() or (float('nan'),) * 3
        err = math.hypot(goal[0] - here[0], goal[1] - here[1])
        yerr = math.degrees(wrap(goal[2] - here[2])) if goal[2] is not None else None
        return dict(ok=ok, msg=msg, steps=steps, secs=time.time() - t0, final_err_m=err, final_yaw_err_deg=yerr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--front', type=float); ap.add_argument('--left', type=float, default=0.0)
    ap.add_argument('--map-x', type=float); ap.add_argument('--map-y', type=float); ap.add_argument('--map-yaw', type=float)
    ap.add_argument('--keep-heading', action='store_true', help='상대 목표에서 도착 방향 = 출발 방향')
    ap.add_argument('--odom-topic', default='/odometry/filtered'); ap.add_argument('--gyro-from-odom', action='store_true')
    ap.add_argument('--cmd-topic', default='/cmd_vel_nav'); ap.add_argument('--step', type=float, default=0.4)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('step_navigator')
    nav = StepNavigator(n, cmd_topic=a.cmd_topic, odom_topic=a.odom_topic, gyro_from_odom=a.gyro_from_odom, step_m=a.step)
    t0 = time.time()
    while (nav.S['pose'] is None or nav.S['scan'] is None or nav.map_pose() is None) and time.time() - t0 < 15:
        nav.spin(0.2)
    here = nav.map_pose()
    if here is None:
        print('❌ map→base_link TF 없음 — SLAM이 떠 있나?'); return 1
    if a.front is not None:
        c, s = math.cos(here[2]), math.sin(here[2])
        goal = (here[0] + c * a.front - s * a.left, here[1] + s * a.front + c * a.left, here[2] if a.keep_heading else None)
    else:
        goal = (a.map_x, a.map_y, math.radians(a.map_yaw) if a.map_yaw is not None else None)
    print(f'출발 map ({here[0]:+.2f},{here[1]:+.2f},{math.degrees(here[2]):+.0f}°) → 목표 ({goal[0]:+.2f},{goal[1]:+.2f}'
          + (f',{math.degrees(goal[2]):+.0f}°)' if goal[2] is not None else ')'), flush=True)
    try:
        r = nav.go(goal)
    except KeyboardInterrupt:
        r = dict(ok=False, msg='중단', steps=-1, secs=0, final_err_m=float('nan'), final_yaw_err_deg=None)
    finally:
        for _ in range(5):
            nav.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
    print(f'{"✅" if r["ok"] else "❌"} {r["msg"]} | 단계 {r["steps"]}, {r["secs"]:.1f}초, 도착 오차 {r["final_err_m"]*100:.1f} cm'
          + (f', 방향 오차 {r["final_yaw_err_deg"]:+.1f}°' if r['final_yaw_err_deg'] is not None else ''))
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo, 'docs/calibration', f'step_nav_{time.strftime("%Y%m%d_%H%M%S")}{("_" + a.label) if a.label else ""}.json')
    json.dump(dict(goal=goal, result=r, log=nav.log), open(out, 'w'), ensure_ascii=False, indent=1, default=str)
    rclpy.shutdown()
    return 0 if r['ok'] else 2


if __name__ == '__main__':
    sys.exit(main())
