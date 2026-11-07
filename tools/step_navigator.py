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
import lidar_blockage
import keepout
import pose_check


class _VScan:
    """스캔과 같은 모양(ranges·각도·범위)의 가벼운 사본 — 금지 구역 경계를 가상의 벽으로 더할 때 쓴다."""
    __slots__ = ('ranges', 'angle_min', 'angle_increment', 'range_min', 'range_max')

    def __init__(self, m):
        self.ranges = list(m.ranges); self.angle_min, self.angle_increment = m.angle_min, m.angle_increment
        self.range_min, self.range_max = m.range_min, m.range_max


class StepNavigator:
    # 밖(액션 서버)에서 True로 두면 go()가 다음 단계 경계에서 '취소'로 끝낸다. 클래스 기본값으로 둔다 —
    #   가상 시험은 __init__을 건너뛰고 객체를 만들어서, 인스턴스 속성으로 두면 회귀 시험이 깨진다(10-09)
    cancel = False
    # 앞 여유가 걸음 + clear_margin에 모자라면 포기하지 않고 여유가 허락하는 만큼(room − clear_margin)으로 줄여 간다.
    #   그 길이가 min_step 미만이면 막힘. 클래스 기본값인 이유는 위 cancel과 같다(가상 시험은 __init__을 건너뜀)
    min_step = 0.10
    #   원시 스캔(최근 몇 장, 자기 필터 전). None이면 바로 옆 미확인 물체 검사를 건너뜀(시뮬·가상 시험엔 /scan_raw가 없다)
    _raw = None
    #   가장 최근 경로의 남은 길이(m)와 그때 위치 — 액션 서버가 feedback(distance_remaining)을 만들 때 읽는다
    path_remaining = None
    path_pose = None
    #   마지막으로 계획이 된 자리(지도 좌표 x, y) — 목표를 넘어 유지한다. 계획이 안 되는 자리에 들어섰을 때 되돌아갈 곳
    last_good = None
    #   떨어진 뒤 그 자리에서 다시 계획할지. ⚠️ 10-10 지도 가상 시험: True면 문틈에서 "물러남 → 직진 → 물러남" 고리(40회) —
    #   경로가 꺾이는 자리가 회전 반경 안이라 생기는 문제라 재계획으로는 안 풀림. 판정 전까지 False
    replan_after_escape = False
    #   금지 구역(tools/keepout.py): 구역 목록과 경계 점(지도 좌표). 비어 있으면 아무 영향 없음. 클래스 기본값인 이유는 cancel과 같다
    _zones = ()
    _zone_pts = ()
    #   구역 서버의 말을 한 번이라도 들었는가. 실물(__init__)에서는 False로 시작 — 못 들은 채로는 움직이지 않는다(아래 go)
    _zones_seen = True
    #   지도(nav_msgs/OccupancyGrid)와 그 거리장 — 출발 전에 "지금 스캔이 지도 위 내 자리와 맞는가"를 본다(tools/pose_check.py).
    #   None이면 검사를 건너뜀(가상 시험). pose_check_enforce=False면 재서 기록만 하고 거절하지 않는다
    _map_msg = None
    _field = None
    pose_check_enforce = True
    #   왕복 끊기(11-01): 출발 뒤 이 각도보다 큰 회전이 같은 목표에서 flip_max번째면 돌지 않고 끝낸다.
    #   ⚠️ 11-01 실물 탐색: 닫힌 문 옆 틈의 코스트맵이 로봇 위치에 따라 뚫렸다 막혀 경로가 2.8 m ↔ 15 m로 뒤집히며 ±175° 왕복, 7분 허비.
    #   근거: 도착한 목표는 실물 13개·시뮬 122개 모두 이런 회전 0번, 왕복한 목표는 2~5번. 1번은 막다른 길에서 돌아서는 정상 동작일 수 있어 허용
    flip_deg = 150.0
    flip_max = 2
    #   시간 제한: 남은 경로 1 m당 이만큼(실측 약 20초/m의 2배)까지 늘린다. 시작값(기본 180초)보다 줄이지 않고 timeout_cap을 넘기지 않는다.
    #   ⚠️ 11-01 실물 목표 10: 왕복이 아니라 24단계를 착실히 가던 중에 180초에 끊겼다(약 9 m가 한계였다)
    route_s_per_m = 40.0
    timeout_cap = 600.0
    #   바로 옆 미확인 물체에서 빠져나오기(11-07): 방금 한 동작이 직진이었으면 그만큼(최대 back_out_max) 곧게 되돌아 나온다.
    #   ⚠️ 11-01 세 번째 탐색: 의자 밑에서 14° 돌고 26 cm 들어간 직후 오른쪽 다리가 옆면 3~7 cm에 붙어 거절만 반복 — 녹화로 복원하니
    #   뒤는 비어 있었다(방금 지나온 길). 제자리 회전은 모서리(반경 15.6 cm)가 다리를 친다. 의자는 늘 움직이니 자리로 피하지 않고 빠져나온다
    back_out_max = 0.30
    last_fwd = 0.0
    #   좁은 틈 앞 정렬(11-07): 앞 여유가 모자라고 경로 1.2 m 안에 좁은 곳(경로 점에서 가장 가까운 장애물이 gap_narrow 안)이 있으면
    #   그 좁은 곳 gap_stage_m 앞의 경로 점으로 정확히 가서 경로 방향으로 돌아선 뒤 곧게 들어간다(목표당 2번까지).
    #   ⚠️ 11-07 실물 탐색: 폭 38~39 cm 의자 통로를 가운데로 가면 양옆 8 cm인데, 0.4 m 걸음으로 모서리를 질러 4~7 cm 치우쳐
    #   들어가 옆 여유(5 cm)에 걸려 우상단을 못 갔다. 지도 시험(의자 통로 40 cm, 무작위 20번): 이 동작 없이 통과 5/20
    gap_narrow = 0.24
    gap_stage_m = 0.40
    #   회전 전 반경 검사에 더하는 여유: 실제 회전 중심은 base_link와 다르다(09-27 큰 회전 뒤 앞뒤로 3.7~10.3 cm 밀림, 부호도 바뀜 → 2~5 cm).
    #   지도 시험의 가상 차체도 4 cm 뒤를 축으로 돈다 — 반경을 base_link 기준 그대로 두면 모서리가 원 밖을 쓸어 의자 다리에 1.6 cm까지 갔다(11-07).
    #   의자 통로 40번: 0 cm 통과 39·최소 1.6 cm / 2 cm 통과 35·최소 4.2 cm / 4 cm 통과 29(통로 안에서 돌 자리가 없다며 포기) → 2 cm
    rot_extra = 0.02
    #   모터 보드 엔코더 마지막 수신 시각(monotonic). None이면 검사 안 함(시뮬·가상 시험엔 엔코더 토픽이 없다).
    #   ⚠️ 11-07 사고: 보드가 멈춘 뒤에도 주행기는 1분 넘게 걸음·후진을 명령했다(위치 추정은 20 m 튐) → 1초 넘게 끊기면 걸음을 내딛지 않는다
    _enc_ts = None
    enc_stale_s = 1.0

    def __init__(self, node, cmd_topic='/cmd_vel_nav', odom_topic='/odometry/filtered', gyro_from_odom=False,
                 step_m=0.4, xy_tol=0.05, yaw_tol_deg=5.0, turn_tol_deg=5.0, settle_s=0.5, blocked_max=3,
                 clear_margin=0.25, back_max=0.15, side_margin=0.05, rot_radius=0.22, raw_scan_topic='/scan_raw'):
        self.n = node
        self.step_m, self.xy_tol, self.yaw_tol = step_m, xy_tol, math.radians(yaw_tol_deg)
        self.turn_tol, self.settle_s, self.blocked_max, self.clear_margin = math.radians(turn_tol_deg), settle_s, blocked_max, clear_margin
        # 통로 옆 여유: 10 cm(09-24, 직진이 휘던 시절 값) → 5 cm. ⚠️ 09-27 장애물 코스: planner 경로가 병 옆을 지나
        #   병이 옆면 7 cm에 오자 "앞 여유 0" 3회로 중단. 근거: 직진 59회 옆밀림 최대 3.3 cm(95% 1.4 cm),
        #   3층 collision_monitor 직진 정지 구역이 옆 3 cm → 주행기가 그보다 한 겹 넓게
        self.back_max, self.side_margin = back_max, side_margin
        # 제자리 회전 전 반경 확인: 3층 collision_monitor 회전 구역(반경 0.20) + 2 cm. ⚠️ 09-27 장애물 코스: 병 옆에서
        #   −32° 회전이 3층에 막혀 "1.5초 동안 0°" 실패 — 주행기가 회전 전 주변을 안 봤다
        self.rot_radius = rot_radius
        self.S = {'pose': None, 'v': 0.0, 'gz': 0.0, 'scan': None}
        node.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        if gyro_from_odom:
            self._gyro_from_odom = True
        else:
            self._gyro_from_odom = False
            node.create_subscription(Imu, '/imu/data', lambda m: self.S.update(gz=m.angular_velocity.z), 100)
        node.create_subscription(LaserScan, '/scan', lambda m: self.S.update(scan=m), 5)
        from std_msgs.msg import Int32MultiArray
        node.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: setattr(self, '_enc_ts', time.monotonic()), 10)
        # ⚠️ 10-10 실측: 이 라이다는 약 13.5 cm보다 가까운 것을 0.08 m로 읽는다 → 옆에 바짝(바퀴 바깥면 약 3 cm 안) 붙은 물체는
        #   차체 다각형 안으로 찍혀 자기 필터에 지워지고 /scan엔 안 보인다(서 있는 동안 놓인 병: 그대로 돌면 침). 원시 스캔에서
        #   기준(자기 구조물 방향) 밖에 0.08 m 반사가 새로 생기면 붙은 물체나 라이다 가림(10-09 종이테이프)으로 보고 움직이지 않는다
        import collections
        self._raw = collections.deque(maxlen=5)
        node.create_subscription(LaserScan, raw_scan_topic, lambda m: self._raw.append(m), 5)   # deque.append는 콜백으로 직접 못 씀(서명 없음)
        self._near_base = lidar_blockage.load_baseline()
        # 금지 구역: keepout_server가 마지막 값을 늦게 붙은 구독자에게도 준다(같은 방식으로 구독)
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        from std_msgs.msg import String
        self._zones_seen = False
        node.create_subscription(String, '/keepout_zones', self._on_zones,
                                 QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        from nav_msgs.msg import OccupancyGrid
        node.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
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

    def _on_zones(self, m):
        try:
            self.set_zones(keepout.clean_zones(json.loads(m.data).get('zones', []))); self._zones_seen = True
        except (ValueError, AttributeError) as e:
            print(f'⚠️ 금지 구역 메시지 오류 — 이전 구역 유지: {e}', flush=True)

    def _on_map(self, m):
        self._map_msg = m; self._field = None        # 거리장은 쓸 때 만든다(탐색 중엔 지도가 5초마다 온다)

    def _pose_check(self, here):
        """지금 스캔을 지도 위 자세 here에 놓아 본 결과(pose_check.check + ok, why). 지도나 스캔이 없으면 None."""
        m, sc = self._map_msg, self.S['scan']
        if m is None or sc is None or here is None:
            return None
        if self._field is None:
            self._field = pose_check.Field.from_msg(m)
        r = pose_check.check(self._field, here, pose_check.scan_points(sc))
        r['ok'], r['why'] = pose_check.verdict(r)
        return r

    def set_zones(self, zones):
        self._zones = tuple(zones); self._zone_pts = tuple(keepout.boundary_points(zones))

    def _scan(self, lidar_x=0.054):
        """지금 스캔 + 금지 구역 경계(가상의 벽). 구역 선을 실제 벽처럼 다루려고 스캔에 점으로 더한다 — 앞 여유·회전 반경·
        떨어지기·가까운 출발점 찾기가 전부 스캔을 읽으므로 한 곳에서 더하면 모두 같은 규칙을 따른다. 구역이 없으면 스캔 그대로.
        서 있을 때의 판단에만 쓴다(직진 중 매 주기 검사 _clear_now는 실제 스캔만 — 구역은 움직이지 않고 걸음 전에 여유를 확인했다)."""
        m = self.S['scan']
        if m is None or not self._zone_pts:
            return m
        here = self.map_pose()
        if here is None:
            return m
        c, s = math.cos(here[2]), math.sin(here[2]); n = len(m.ranges); out = None; two_pi = 2 * math.pi
        for wx, wy in self._zone_pts:
            dx, dy = wx - here[0], wy - here[1]
            if abs(dx) > 2.0 or abs(dy) > 2.0:
                continue
            x, y = c * dx + s * dy - lidar_x, -s * dx + c * dy
            i = int(round(((math.atan2(y, x) - m.angle_min) % two_pi) / m.angle_increment))
            if i >= n:
                if i > n:
                    continue      # 스캔이 한 바퀴를 다 덮지 않는 경우 그 밖 방향
                i = 0
            r = max(math.hypot(x, y), m.range_min + 1e-3)
            if out is None:
                out = _VScan(m)
            if not (m.range_min <= out.ranges[i] <= m.range_max) or r < out.ranges[i]:
                out.ranges[i] = r
        return out or m

    def spin(self, t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(self.n, timeout_sec=0.005)

    def _clear_now(self, sgn):
        """직진 중 매 주기: 가는 쪽 통로 0.2 m 안 장애물 여부(collision_monitor와 별개의 한 겹).
        한 장 잡음 걸러내기(0.15초 지속)는 1층 drive_distance 안에서 한다(09-27 층 정리)."""
        return self.S['scan'] is not None and corridor_clearance(self.S['scan'], sgn > 0, 0.0 if sgn > 0 else 0.1,
                                                                 self.side_margin) > 0.2

    def _corridor_pts(self, room, lidar_x=0.054, half=0.11, corridor=0.10):
        """앞 통로(corridor_clearance와 같은 범위)에서 차체 앞끝으로부터 room + 5 cm 안의 점 (x, y, 거리) — 진단 기록용."""
        m = self._scan(); out = []
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

    def carrot(self, path, here, ahead=None):
        """경로에서 here와 가장 가까운 점부터 한 걸음 앞 지점. 걸음 = 남은 경로를 step_m 이하로 **균등 분할**한 길이
        (ahead를 주면 그 거리 앞 지점 — 좁은 곳에서 경로를 더 바짝 따라갈 때).
        ⚠️ 09-27 합격 시험: step_m(0.4)로 자르면 1.28 m 복귀가 0.4×3 뒤 8 cm 남기고 xy_tol(10 cm) 안이라 멈춤
        — 3회 모두 실제 복귀 9.7~11.0 cm(SLAM도 7.8~8.3 cm로 알고 있었음). 균등 분할이면 마지막 걸음이 목표에 떨어진다."""
        i0 = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - here[0], path[i][1] - here[1]))
        segs = [math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]) for i in range(i0, len(path) - 1)]
        step = self.step_m
        if sum(segs) > 0:
            step = sum(segs) / math.ceil(sum(segs) / self.step_m - 1e-9)
        if ahead is not None:
            step = min(step, ahead)
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

    def go(self, goal, max_steps=120, timeout=180.0):
        """goal=(x, y, yaw 또는 None) map 좌표. 반환 dict(ok, msg, steps, secs, final_err_m, final_yaw_err_deg)."""
        t0 = time.time(); blocked = 0; final_fix = 0; near_turns = 0; arrive_tol = self.xy_tol; retreats = 0; flips = 0; longest = 0.0; backouts = 0; stages = 0
        self.path_remaining = None; self.path_pose = None
        # 금지 구역 정보를 아직 못 받았으면 잠깐 기다리고, 끝내 없으면 움직이지 않는다.
        #   ⚠️ 10-10 Stage: 주행기 서버가 뜨자마자 온 첫 목표가 구역 안이었는데, 구역 목록이 아직 안 도착해 '구역 없음'으로 출발 판단
        #   (planner가 경로를 안 내 줘 움직이진 않았다). 안전 기능은 "모르면 없는 셈"이 아니라 "모르면 멈춤"이어야 한다
        tz = time.time()
        while not self._zones_seen and time.time() - tz < 5.0:
            self.spin(0.1)
        if not self._zones_seen:
            return self._done(False, '금지 구역 정보를 받지 못함 — 구역 서버(keepout_server)가 떠 있는지 확인', 0, t0, goal)
        for step in range(max_steps):
            if time.time() - t0 > timeout:
                return self._done(False, '시간 초과', step, t0, goal)
            if self.cancel:
                return self._done(False, '취소', step, t0, goal)
            if self._enc_ts is not None and time.monotonic() - self._enc_ts > self.enc_stale_s:
                return self._done(False, f'모터 보드 응답 없음({time.monotonic() - self._enc_ts:.1f}초) — 구동 노드가 보드를 리셋했는지 확인', step, t0, goal)
            # 금지 구역: 목표가 구역 안(차체가 선을 넘어야 닿는 자리 포함)이면 더 가지 않고 끝낸다 — planner도 못 가는 자리라
            #   경로 실패로 3번 기다리다 끝나는 것보다 이유를 바로 알린다(탐색기는 실패한 목표를 잠시 제외한다).
            #   매 단계 본다: 가는 도중에 사람이 구역을 그릴 수 있다
            gd, gz = keepout.nearest(goal[0], goal[1], self._zones)
            if gd < keepout.INSCRIBED:
                return self._done(False, f'목표가 금지 구역{self._zname(gz)} 안', step, t0, goal)
            self.settle()
            nu = self._near_unknown()
            if nu:
                blocked += 1; self._note(step, 'near_unknown', dict(sectors=nu[:12]), None)
                # 두 번째로도 보이면(지나가는 손이 아니면) 방금 들어온 직진을 되돌린다 — 물체가 뒤쪽 방향이면 하지 않는다
                if blocked >= 2 and backouts < 1 and self.last_fwd > 0.03 and not any(135 <= d <= 225 for d, _ in nu):
                    if self._straight(step, -min(self.last_fwd, self.back_out_max), 'back_out', margin=0.05):
                        backouts += 1; blocked = 0; self.spin(1.0); continue
                if blocked >= self.blocked_max:
                    return self._done(False, f'라이다 바로 옆 미확인 물체 또는 가림({len(nu) * lidar_blockage.BIN}°: {nu[0][0]}°~{nu[-1][0]}°, 0°=정면·90°=왼쪽)', step, t0, goal)
                self.spin(1.0); continue
            here = self.map_pose()
            if here is None:
                return self._done(False, 'map→base_link TF 없음', step, t0, goal)
            # 지도 위 내 자리가 지금 스캔과 맞는가 — 출발 전(첫 단계)에 본다. ⚠️ 10-11: 위치 잡기를 켠 채 사람이 로봇을 58 cm 옮겼는데
            #   자세는 옮기기 전 그대로였다(경고 없음). 그대로 걸으면 틀린 자리 기준으로 걷고 금지 구역도 틀린 자리에 놓인다.
            #   한 번 안 맞으면 새 스캔으로 두 번 더 본다(지나가는 사람). 매 단계 값은 기록만 한다 — 주행 중 거절할지는 자료가 쌓인 뒤 판단
            pc = self._pose_check(here)
            if pc is not None:
                if step == 0 and not pc['ok']:
                    for _ in range(2):
                        self.spin(0.7); here = self.map_pose() or here; pc = self._pose_check(here) or pc
                        if pc['ok']:
                            break
                self.log.append(dict(step=step, kind='pose_check', t=round(time.time(), 2), ok=pc['ok'], n=pc['n'], on=pc['on'], free=pc['free'], unk=pc['unk'],
                                     ratio=None if pc['ratio'] is None else round(pc['ratio'], 3)))
                if step == 0 and not pc['ok'] and self.pose_check_enforce:
                    return self._done(False, f'지도 위 자리가 스캔과 안 맞음 — {pc["why"]}. 로봇이 옮겨졌을 수 있다 → 자리를 다시 찾을 것(tools/relocate.py)', step, t0, goal)
            hd, hz = keepout.nearest(here[0], here[1], self._zones)
            if hd == 0.0:
                # 구역 안에서는 경계가 사방의 벽이라 스스로 못 나온다(나오게 두면 구역의 뜻이 없다) → 사람이 정리
                return self._done(False, f'로봇이 금지 구역{self._zname(hz)} 안에 있음 — 구역을 고치거나 로봇을 옮길 것', step, t0, goal)
            dist = math.hypot(goal[0] - here[0], goal[1] - here[1])
            if dist <= arrive_tol:
                if goal[2] is not None:
                    dyaw = wrap(goal[2] - here[2])
                    if abs(dyaw) > self.yaw_tol:
                        r = self._turn(step, dyaw, 'final_turn')
                        if not r['ok']:
                            return self._done(False, f'마지막 회전 실패: {r["msg"]}', step, t0, goal)
                        # ⚠️ 09-27 합격 시험: 제자리 회전 중심이 base_link와 달라 마지막 150~175° 회전이 위치를 민다
                        #   — 실제 복귀 오차가 거의 앞뒤 성분(−10.3, −3.9, +5.1, −4.0, −3.7 cm). 방향은 이미 맞았으니
                        #   옆 오차가 작으면 돌지 않고 앞뒤 직진으로만 보정한다(1회)
                        self.settle(); here = self.map_pose() or here
                        along, lat = self._goal_in_body(goal, here)
                        if final_fix < 1 and math.hypot(along, lat) > self.xy_tol and abs(lat) <= self.xy_tol:
                            final_fix += 1
                            if self._straight(step, along, 'final_fix'):
                                continue
                return self._done(True, '도착' if arrive_tol == self.xy_tol else '도착(근접 허용)', step, t0, goal)
            path, why = self.plan(here, goal)
            if path is None:
                # ⚠️ 10-10 Stage: 로봇이 문 A 안(문설주에서 0.15 m)에 섰을 때 "빈 경로" → 그 자리에서 이후 목표 35개가 전부 실패,
                #   지도 58 m²에서 정지. NavFn은 출발 칸이 "벽에 닿는 거리"(중심 0.11 m 안에 벽 칸)면 경로를 못 만든다 — SLAM 벽이
                #   한 칸만 가깝게 그려져도 걸린다(지도 가상 시험: 벽 0.06 m에서 재현). 스캔으로 비어 있음이 확인된 가까운 점에서 다시 계획
                path, shift = self._plan_from_nearby(here, goal)
                if path is not None:
                    self._note(step, 'plan_shift', dict(d=shift[0], deg=round(shift[1])), None)
            if path is None:
                # ⚠️ 10-10 Stage 스냅샷: SLAM 지도가 어긋나 문틈에 유령 벽이 그려지면 로봇이 "지도상 벽 속"에 서게 되고, 어떤 planner도
                #   (NavFn·Smac 2D·Theta* 전부) 거기서 출발하는 경로를 못 낸다. 그 자리에 남으면 다음 목표들도 줄줄이 실패(35개 연속).
                #   → 마지막으로 계획이 됐던 자리로 되돌아간다(방금 지나온 곳). 같은 목표에서 두 번 되돌아가면 포기 — 계획되는 자리에 남는다
                self._note(step, 'plan_fail', why, None)
                if self.last_good is not None and 0.03 < math.hypot(self.last_good[0] - here[0], self.last_good[1] - here[1]) <= 0.6 \
                        and self._retreat(step, here):
                    retreats += 1
                    if retreats >= 2:
                        return self._done(False, f'경로 실패: {why}(계획되는 자리로 {retreats}번 되돌아감)', step, t0, goal)
                    continue
                blocked += 1
                if blocked >= self.blocked_max:
                    return self._done(False, f'경로 실패 {blocked}회: {why}', step, t0, goal)
                self.spin(1.0); continue
            self.last_good = (here[0], here[1])
            i0 = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - here[0], path[i][1] - here[1]))
            self.path_remaining = sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]) for i in range(i0, len(path) - 1))
            self.path_pose = (here[0], here[1])
            # 지금까지 본 것보다 긴 경로가 나왔을 때만 늘린다. ⚠️ 첫 구현: 재계획마다 늘려 제자리걸음 2 m 목표가 600초까지 갔다(가상 시험에서 잡음)
            if self.path_remaining > longest:
                longest = self.path_remaining
                timeout = min(max(timeout, time.time() - t0 + self.route_s_per_m * longest), max(timeout, self.timeout_cap))
            c = self.carrot(path, here)
            seg = min(self.step_m + 0.01, math.hypot(c[0] - here[0], c[1] - here[1]))   # 균등 분할 걸음은 step_m 이하(반올림 여유 1 cm)
            head = math.atan2(c[1] - here[1], c[0] - here[0])
            dyaw = wrap(head - here[2])
            # 가까운 목표(back_max 안)가 뒤쪽이면 돌지 말고 후진 — ⚠️ 09-27: 5 cm 뒤 목표에 174° 돌고 5 cm 가고 다시 돎
            if abs(dyaw) > math.pi / 2 and dist <= self.back_max:
                rdyaw = wrap(dyaw - math.pi)
                if abs(rdyaw) > self.turn_tol:
                    r = self._turn(step, rdyaw, 'turn', replan_after_escape=self.replan_after_escape)
                    if r.get('moved'):
                        continue
                    if r.get('blocked'):
                        blocked += 1
                        if blocked >= self.blocked_max:
                            return self._done(False, f'회전 반경 막힘 {blocked}회', step, t0, goal)
                        self.spin(1.0); continue
                    if not r['ok']:
                        return self._done(False, f'회전 실패: {r["msg"]}', step, t0, goal)
                    self.settle()
                if self._straight(step, -seg, 'back'):
                    continue
                # 뒤가 막혔으면 아래 보통 경로(돌아서 전진)로
            # 돌기 전에 그 방향 통로를 지금 스캔으로 미리 본다. 막히면 더 가까운 경로 지점(걸음의 1/2, 1/4)을 향한다 —
            #   가까운 지점일수록 경로를 그대로 따라가므로, 좁은 틈 앞에서 먼저 경로 위로 올라선 뒤 똑바로 들어간다.
            #   ⚠️ 10-09 our_home 시뮬 (4.8, −1.2): 폭 0.40~0.45 m 문틈을 문설주 밑에서 0.35 m 앞 지점으로 질러가려다
            #   −26° 돌고 막힘 → +23° 돌고 막힘 → 다시 −24° … 3회 포기. 돌아 본 뒤에야 막힌 걸 알던 헛회전도 없앤다
            for ahead in (None, self.step_m / 2, self.step_m / 4):
                c2 = c if ahead is None else self.carrot(path, here, ahead)
                seg2 = min(self.step_m + 0.01, math.hypot(c2[0] - here[0], c2[1] - here[1]))
                if seg2 < 0.03:
                    continue
                dyaw2 = wrap(math.atan2(c2[1] - here[1], c2[0] - here[0]) - here[2])
                room2 = self._room_dir(dyaw2)
                if room2 >= seg2 + self.clear_margin or room2 - self.clear_margin - 0.01 >= self.min_step:
                    if ahead is not None:
                        self._note(step, 'lookahead', dict(ahead=round(ahead, 2), dyaw=round(math.degrees(dyaw2), 1), room=round(room2, 2)), None)
                        c, seg, dyaw = c2, seg2, dyaw2
                    break
            if abs(dyaw) > self.turn_tol:
                # ⚠️ 가상 시험(09-27)에서 잡은 진동: 가까운 목표에 크게 돌면 회전 중심 어긋남이 로봇을 목표 너머로 밀어
                #   (180°면 회전 중심 거리의 2배) 목표가 다시 뒤로 가고 또 돈다 — 뒤가 막혀 후진도 못 하면 무한 반복
                if dist <= self.back_max and abs(dyaw) > math.pi / 2:
                    if near_turns >= 1 and dist <= 2 * self.xy_tol:
                        return self._done(True, '도착(근접 허용)', step, t0, goal)
                    if near_turns >= 2:
                        return self._done(False, f'가까운 목표 주변 진동({dist * 100:.0f} cm)', step, t0, goal)
                    near_turns += 1
                flip = step > 0 and abs(dyaw) > math.radians(self.flip_deg) and dist > self.back_max
                if flip and flips + 1 >= self.flip_max:
                    self._note(step, 'flip_stop', dict(dyaw=round(math.degrees(dyaw), 1), path_m=round(self.path_remaining, 2), flips=flips + 1), None)
                    return self._done(False, f'경로가 뒤집혀 왕복 중 — {flips + 1}번째 큰 회전({math.degrees(dyaw):+.0f}°) 전에 멈춤, 남은 경로 {self.path_remaining:.1f} m', step, t0, goal)
                r = self._turn(step, dyaw, 'turn', replan_after_escape=self.replan_after_escape)
                if r.get('moved'):
                    continue
                if r.get('blocked'):
                    blocked += 1
                    if blocked >= self.blocked_max:
                        return self._done(False, f'회전 반경 막힘 {blocked}회', step, t0, goal)
                    self.spin(1.0); continue
                if not r['ok']:
                    return self._done(False, f'회전 실패: {r["msg"]}', step, t0, goal)
                flips += flip
                self.settle()
            room = corridor_clearance(self._scan(), True, 0.0, self.side_margin)
            if room < seg + self.clear_margin and room - self.clear_margin - 0.01 >= self.min_step:
                # ⚠️ 10-09 our_home 시뮬: 앞 여유 0.55 m인데 걸음 0.345 + 여유 0.25 = 0.595가 안 돼 같은 자리에서 막힘 31번 반복
                #   (탐색기가 같은 목표를 계속 다시 보냄, 지도 300초 뒤 정지). 걸음을 줄이면 그 걸음은 안전하다(여유 0.25 유지)
                seg = room - self.clear_margin - 0.01
                self._note(step, 'shorten', dict(room=round(room, 2), seg=round(seg, 3)), None)
            if room < seg + self.clear_margin and dist <= 2 * self.xy_tol and arrive_tol == self.xy_tol:
                # ⚠️ 10-10 지도 가상 시험 무작위 40쌍: 실패 2건이 모두 목표 9 cm 앞(합격 기준 10 cm 안)에서 "앞 여유 0.26 m"로
                #   3번 막혀 실패 보고 — 목표가 벽 가까이라 마지막 9 cm가 "간 뒤 0.25 m 남기기"에 걸림. 안전 여유를 줄여
                #   벽으로 밀어 넣지 않고, 이미 2×xy_tol(10 cm) 안이면 도착으로 인정(방향 지정이 있으면 위에서 마지막 회전)
                arrive_tol = 2 * self.xy_tol
                self._note(step, 'near_accept', dict(dist=round(dist, 3), room=round(room, 2)), None)
                continue
            if room < seg + self.clear_margin and stages < 2 and self._gap_stage(step, path, here):
                stages += 1; blocked = 0; continue
            if room < seg + self.clear_margin:
                # ⚠️ 09-27: 병 옆 통과 뒤 "앞 여유 0"으로 멈췄는데 무엇이 막았는지 남기지 않아 원인을 못 가렸다
                #   (사용자: 그 자리엔 아무것도 없었음, 병은 20~30 cm 옆). 막은 점들을 base_link 좌표로 기록
                blocked += 1; self._note(step, 'blocked', dict(room=round(room, 2), pts=self._corridor_pts(room, corridor=self.side_margin)), None)
                if blocked >= self.blocked_max:
                    return self._done(False, f'앞 여유 부족 {blocked}회({room:.2f} m)', step, t0, goal)
                self.spin(1.0); continue
            r = self.driver.drive_by(seg); self._note(step, 'drive', round(seg, 3), r)
            self.last_fwd = seg if r['ok'] else 0.0
            if not r['ok']:
                blocked += 1
                if blocked >= self.blocked_max:
                    return self._done(False, f'직진 실패: {r["msg"]}', step, t0, goal)
            else:
                blocked = 0
        return self._done(False, f'단계 {max_steps}회 초과', max_steps, t0, goal)

    def _gap_stage(self, step, path, here, lidar_x=0.054):
        """경로 앞 1.2 m 안에서 양옆이 다 막힌 가장 좁은 틈을 지금 스캔으로 찾아, 그 틈 가운데 선의 gap_stage_m 앞으로 가서
        틈 방향으로 선다. 너무 가까우면 먼저 곧게 물러선다. 움직였으면 True.
        ⚠️ 첫 구현(11-07 지도 시험): ① 경로는 틈 가운데로 나지 않는다(좁은 곳에서 경로-다리 1.7 cm) → 양옆 장애물로 가운데를 잡는다
        ② 한쪽만 막힌 곳(막이 벽 모서리)도 좁은 곳으로 잡아 폭 4.6 cm로 포기 → 양쪽이 다 막힌 곳만 틈으로 본다
        ③ 물러서기만 하고 끝내 정렬 기회를 썼다 → 물러선 뒤 같은 동작 안에서 정렬까지 한다"""
        m = self._scan()
        if m is None or len(path) < 3:
            return False
        c, s = math.cos(here[2]), math.sin(here[2]); pts = []
        for i, r in enumerate(m.ranges):
            if m.range_min <= r <= min(m.range_max, 2.0):
                a = m.angle_min + i * m.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
                if not (abs(x) <= 0.11 and abs(y) <= 0.11):
                    pts.append((here[0] + c * x - s * y, here[1] + s * x + c * y))
        if not pts:
            return False
        i0 = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - here[0], path[i][1] - here[1]))
        acc, best = 0.0, None
        for i in range(i0, len(path)):
            if i > i0:
                acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if acc > 1.2:
                break
            a0, a1 = path[max(i - 3, 0)], path[min(i + 3, len(path) - 1)]
            t = math.atan2(a1[1] - a0[1], a1[0] - a0[0]); tx, ty, nx, ny = math.cos(t), math.sin(t), -math.sin(t), math.cos(t)
            L = R = None
            for px, py in pts:
                dx, dy = px - path[i][0], py - path[i][1]
                if abs(dx * tx + dy * ty) <= 0.12:
                    o = dx * nx + dy * ny
                    if 0 <= o <= 0.5 and (L is None or o < L[0]):
                        L = (o, px, py)
                    elif -0.5 <= o < 0 and (R is None or -o < R[0]):
                        R = (-o, px, py)
            if L is not None and R is not None:
                w = math.hypot(L[1] - R[1], L[2] - R[2])       # 양옆 장애물 점 사이 실제 거리
                if w < 0.50 and (best is None or w < best[0]):
                    best = (w, i, acc, t, L, R)
        if best is None:
            return False
        width, n, an, t, L, R = best
        if width < 2 * 0.11 + 0.06:
            self._note(step, 'gap_too_narrow', dict(width=round(width, 3)), None)
            return False
        # 틈 축 = 양옆 점을 잇는 선에 수직(경로 쪽 방향). ⚠️ 경로 방향으로 섰더니 경로가 틈에 26~31° 비스듬히 들어가 몸도 비스듬히 섰다
        cx0, cy0 = (L[1] + R[1]) / 2, (L[2] + R[2]) / 2
        ax = math.atan2(L[2] - R[2], L[1] - R[1]) - math.pi / 2
        if math.cos(ax - t) < 0:
            ax += math.pi
        t = wrap(ax); tx, ty = math.cos(t), math.sin(t)
        sp = (cx0 - tx * self.gap_stage_m, cy0 - ty * self.gap_stage_m)
        self._note(step, 'gap_stage', dict(width=round(width, 3), at=round(an, 2), center=(round(cx0, 2), round(cy0, 2)), stage=(round(sp[0], 2), round(sp[1], 2)), dir=round(math.degrees(t))), None)
        moved = False
        if an < self.gap_stage_m:      # 틈에 너무 가깝다 — 곧게 물러서서 정렬할 자리를 만든다
            if self._straight(step, -(self.gap_stage_m - an), 'gap_back', margin=0.05):
                moved = True; self.settle(); here = self.map_pose() or here
        d = math.hypot(sp[0] - here[0], sp[1] - here[1])
        if d > 0.03:
            r = self._turn(step, wrap(math.atan2(sp[1] - here[1], sp[0] - here[0]) - here[2]), 'gap_turn')
            if not r.get('ok') or r.get('moved'):
                return moved or bool(r.get('moved'))
            self.settle()
            if not self._straight(step, d, 'gap_approach', margin=0.05):
                return True
            self.settle(); here = self.map_pose() or here
        dyaw = wrap(t - here[2])
        if abs(dyaw) > self.turn_tol:
            r = self._turn(step, dyaw, 'gap_align')
            if not r.get('ok'):
                return True
            self.settle()
        # 정렬한 축 그대로 틈 너머까지 곧게(다음 걸음에서 경로 방향으로 다시 틀면 비스듬해진다). 앞 여유가 허락하는 만큼만
        # 틈이 끝나는 곳까지: 축 옆(차체 반폭 밖 ~ 30 cm)에 장애물이 있는 마지막 지점 너머로 차체가 다 빠질 만큼.
        #   ⚠️ 첫 구현은 틈 가운데에서 35 cm만 가서, 다리가 두 쌍(앞·뒤 40 cm)인 의자 밑에서 뒷다리 사이에 서서 돌다 모서리가 1.6 cm까지 갔다
        here = self.map_pose() or here
        end = 0.0
        for px, py in pts:
            dx, dy = px - cx0, py - cy0; al, la = dx * tx + dy * ty, abs(-dx * ty + dy * tx)
            if 0.0 <= al <= 1.0 and 0.11 < la < 0.30:
                end = max(end, al)
        through = max(0.0, (cx0 - here[0]) * tx + (cy0 - here[1]) * ty) + max(0.35, end + 0.11 + 0.10)
        room = corridor_clearance(self._scan(), True, 0.0, self.side_margin)
        d = min(through, room - self.clear_margin - 0.01)
        if d >= self.min_step:
            self._straight(step, d, 'gap_through')
        return True

    @staticmethod
    def _zname(z):
        return f' "{z["name"]}"' if z and z.get('name') else ''

    def _free_dir(self, bearing, half_width=math.radians(15), lidar_x=0.054, half=0.115):
        """로봇 기준 방향 bearing(rad)으로 차체 중심에서 가장 가까운 스캔 점까지 거리(±15° 부채꼴, 차체 안 제외)."""
        m = self._scan(); best = float('inf')
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
            if abs(x) <= half and abs(y) <= half:
                continue
            if abs(wrap(math.atan2(y, x) - bearing)) <= half_width:
                best = min(best, math.hypot(x, y))
        return best

    def _plan_from_nearby(self, here, goal):
        """지금 자리에서 계획이 안 될 때: 가까운 점(0.10·0.15·0.20 m, 8방향)을 출발점으로 다시 계획. 그 방향이 스캔상 점까지
        d + 0.10 m 넘게 비어 있을 때만(벽 너머 점을 출발점으로 삼지 않게), 더 열린 방향부터. 반환 (경로, (d, 방향°)) 또는 (None, None)."""
        dirs = sorted(((self._free_dir(k * math.pi / 4), k * math.pi / 4) for k in range(8)), reverse=True)
        for d in (0.10, 0.15, 0.20):
            for free, a in dirs:
                if free < d + 0.10:
                    continue
                ang = here[2] + a
                path, _ = self.plan((here[0] + d * math.cos(ang), here[1] + d * math.sin(ang), here[2]), goal)
                if path is not None:
                    return path, (d, math.degrees(wrap(a)))
        return None, None

    def _retreat(self, step, here):
        """last_good(마지막으로 계획이 된 자리)로 곧게 되돌아간다. 그 자리가 거의 바로 뒤(±25°)면 후진, 아니면 그쪽으로 돌아 전진.
        가는 쪽 통로는 스캔으로 확인(_straight). 움직였으면 True."""
        dx, dy = self.last_good[0] - here[0], self.last_good[1] - here[1]; d = math.hypot(dx, dy)
        dyaw = wrap(math.atan2(dy, dx) - here[2])
        if abs(wrap(dyaw - math.pi)) <= math.radians(25):
            return self._straight(step, -d, 'retreat', margin=0.05)
        if abs(dyaw) > self.turn_tol:
            r = self._turn(step, dyaw, 'turn')
            if not r['ok']:
                return False
            self.settle()
        return self._straight(step, d, 'retreat', margin=0.05)

    def _near_unknown(self):
        """원시 스캔 최근 몇 장에서 기준(자기 구조물) 밖 방향에 12 cm 안 반사가 새로 있는 2° 칸 목록 — 합이 6° 넘을 때만(아니면 빈 목록)."""
        if not self._raw or len(self._raw) < 3:
            return []
        nb = lidar_blockage.new_blocked(lidar_blockage.near_fraction(list(self._raw)), self._near_base)
        return nb if len(nb) * lidar_blockage.BIN > lidar_blockage.MAX_NEW_DEG else []

    def _room_dir(self, dyaw, lidar_x=0.054, half=0.11):
        """dyaw만큼 돈 뒤의 앞 통로 여유(m)를 지금 스캔으로 미리 계산 — 점들을 차체 중심에서 −dyaw 돌려 corridor_clearance와
        같은 규칙(차체 폭 + 옆 side_margin, 차체 앞끝부터)으로 본다. 실제 판정은 돈 뒤에 한 번 더 한다(회전 중심 어긋남·새 스캔)."""
        m = self._scan(); c, s = math.cos(-dyaw), math.sin(-dyaw); best = float('inf')
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
            if abs(x) <= half + 0.005 and abs(y) <= half + 0.005:
                continue   # 차체 안 = 자기 구조물
            xr, yr = c * x - s * y, s * x + c * y
            if abs(yr) > half + self.side_margin:
                continue
            gap = xr - half
            if gap > 0:
                best = min(best, gap)
        return best

    def _near_pts(self, r_max, lidar_x=0.054, half=0.115):
        """스캔 점 중 base_link에서 r_max 안(차체 사각형 제외) — 로봇 기준 (x, y)."""
        m = self._scan(); out = []
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
            if abs(x) <= half and abs(y) <= half:
                continue
            if math.hypot(x, y) <= r_max:
                out.append((x, y))
        return out

    def _turn(self, step, dyaw, kind, replan_after_escape=False):
        """제자리 회전. 먼저 반경 rot_radius 안이 비었는지 보고, 막혔으면 앞/뒤로 가장 짧게(5~25 cm) 떨어진 뒤 돈다.
        떨어질 수 없으면 돌지 않고 dict(ok=False, blocked=True)."""
        R = self.rot_radius + self.rot_extra
        near = [p for p in self._near_pts(R)]
        if near:
            # 돌 자리 찾기: 앞뒤로 곧게 최대 60 cm. ⚠️ 11-07 다섯 번째 탐색: 왼쪽 벽이 차체 옆(중심에서 20 cm)을 따라 이어진 자리에서
            #   25 cm 안에는 돌 자리가 없어 "회전 반경 막힘"으로 갇혔다 — 벽이 끝나는 45~50 cm 뒤까지 가야 돌 수 있었다
            pts = self._near_pts(R + 0.65); move = None
            for d in [k / 100 for k in range(5, 61, 5)]:
                for s in (d, -d):
                    if all(math.hypot(x - s, y) > R for x, y in pts):
                        move = s; break
                if move is not None:
                    break
            self._note(step, 'rot_near', dict(pts=[(round(x, 3), round(y, 3)) for x, y in near[:6]], move=move), None)
            if move is None and self.last_fwd > 0.03:
                # 돌 자리가 없으면 회전 없이 방금 들어온 직진을 곧게 되돌린다(들어온 길이라 옆 간격이 그대로) → 호출한 쪽이 다시 계획
                d = -min(self.last_fwd, 0.40)
                if self._straight(step, d, 'rot_retrace', margin=0.05):
                    self.last_fwd = 0.0
                    return dict(ok=True, moved=True, msg='')
            if move is None or not self._straight(step, move, 'rot_clear', margin=0.05):
                return dict(ok=False, blocked=True, msg='회전 반경 안 장애물 — 떨어질 곳 없음')
            self.settle()
            if self._near_pts(R):
                return dict(ok=False, blocked=True, msg='떨어진 뒤에도 회전 반경 안 장애물')
            if replan_after_escape:
                # ⚠️ 10-10 지도 가상 시험(문틈): 떨어지느라 위치가 10~20 cm 바뀌었는데 떨어지기 전에 정한 각도로 돌아
                #   앞이 막힘(헛회전 2회). 위치가 바뀌었으면 돌지 말고 경로·방향부터 다시 정한다
                return dict(ok=True, moved=True, msg='')
        r = self.turner.turn_by(dyaw); self._note(step, kind, math.degrees(dyaw), r); self.last_fwd = 0.0
        return r

    def _goal_in_body(self, goal, here):
        """목표까지 로봇 기준 (앞, 왼쪽) 거리."""
        dx, dy = goal[0] - here[0], goal[1] - here[1]; c, s = math.cos(here[2]), math.sin(here[2])
        return c * dx + s * dy, -s * dx + c * dy

    def _straight(self, step, d, kind, margin=None):
        """돌지 않고 앞(+)/뒤(−)로 d m. 가는 쪽 여유가 |d| + margin(기본 clear_margin) 이상일 때만. 실행했으면 True."""
        room = corridor_clearance(self._scan(), d > 0, 0.0 if d > 0 else 0.1, self.side_margin)
        if room < abs(d) + (self.clear_margin if margin is None else margin):
            self._note(step, kind + '_blocked', dict(d=round(d, 3), room=round(room, 2)), None)
            return False
        r = self.driver.drive_by(d); self._note(step, kind, round(d, 3), r)
        self.last_fwd = d if d > 0 and r.get('ok') else 0.0
        return True

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
