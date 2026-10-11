#!/usr/bin/env python3
"""지도 위 자리를 다시 찾아 위치 잡기에 알려 준다 — 저장 지도로 다니는 중에 로봇을 손으로 옮겼을 때 (2026-10-11).

왜: 위치 잡기(slam_toolbox localization)는 바퀴·IMU가 말해 주지 않는 이동(들어 옮김)을 모른다. 주행기는 출발 전에
스캔과 지도가 안 맞으면 거절한다(tools/pose_check.py) — 그다음에 할 일이 이것이다. 스택을 내렸다 켜지 않아도 된다.

방법: ① 지금 스캔으로 저장 지도 전체에서 자리를 찾는다(tools/locate_on_map.py, 기동 때와 같은 방법·같은 확신 기준)
      ② slam_toolbox의 /initialpose(원래 있는 기능 — RViz의 "2D Pose Estimate"가 쓰는 것)로 그 자세를 알려 준다
      ③ 지도 위 자세가 그리로 옮겨졌는지, 스캔이 지도와 맞는지 확인한다
로봇은 움직이지 않는다. 서 있을 때만 한다. 지도를 그리는 중(매핑 방식)에는 쓸 수 없다 — 그때는 스택을 다시 켠다.

    python3 tools/relocate.py                 # 찾고 → 알려 주고 → 확인
    python3 tools/relocate.py --dry           # 찾기만(알려 주지 않음)
    python3 tools/relocate.py --set=0.5,-0.1,0    # (시험용) 찾지 않고 이 자세(x, y, 도)를 알려 준다 — 일부러 틀리게 만들 때
    시뮬(도메인 42): --odom-topic /odom
"""
import argparse, math, os, sys, time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pose_check
from locate_on_map import locate, whole_area


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry', action='store_true'); ap.add_argument('--set', help='x,y,도 — 찾지 않고 이 자세를 알려 준다(시험용)')
    ap.add_argument('--odom-topic', default='/odometry/filtered')
    a = ap.parse_args()
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); n = rclpy.create_node('relocate'); got = {}
    n.create_subscription(LaserScan, '/scan', lambda m: got.update(scan=m), qos_profile_sensor_data)
    n.create_subscription(Odometry, a.odom_topic, lambda m: got.update(v=(m.twist.twist.linear.x, m.twist.twist.angular.z)), 10)
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.update(map=m), QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    par = n.create_client(GetParameters, '/slam_toolbox/get_parameters'); tfb = Buffer(); TransformListener(tfb, n)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.05)

    def pose():
        try:
            tr = tfb.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        q = tr.transform.rotation
        return (tr.transform.translation.x, tr.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def agree(tag):
        """지금 자세에서 스캔이 지도와 맞는지 한 줄로."""
        got.pop('scan', None); t0 = time.time()
        while 'scan' not in got and time.time() - t0 < 3:
            spin(0.1)
        p = pose()
        if p is None or 'scan' not in got or 'map' not in got:
            print(f'  {tag}: 자세·스캔·지도 중 못 받은 것이 있음'); return None, p
        r = pose_check.check(pose_check.Field.from_msg(got['map']), p, pose_check.scan_points(got['scan'])); ok, why = pose_check.verdict(r)
        print(f'  {tag}: 지도 위 ({p[0]:+.2f}, {p[1]:+.2f}) m, {math.degrees(p[2]):+.1f}° — 아는 칸의 점 {r["on"] + r["free"]}개 중 벽 위 '
              + (f'{r["ratio"] * 100:.0f}%' if r['ratio'] is not None else '–') + (' ✅' if ok else ' ❌ 안 맞음'), flush=True)
        return ok, p

    try:
        if not par.wait_for_service(timeout_sec=5.0):
            print('❌ SLAM(slam_toolbox)이 떠 있지 않다'); return 2
        f = par.call_async(GetParameters.Request(names=['mode', 'map_file_name'])); t0 = time.time()
        while not f.done() and time.time() - t0 < 5:
            spin(0.05)
        mode, base = (f.result().values[0].string_value, f.result().values[1].string_value) if f.done() else ('', '')
        if mode != 'localization' or not base:
            print(f'❌ 저장 지도로 다니는 중이 아니다(SLAM 방식: {mode or "모름"}) — 지도를 그리는 중에는 자리를 다시 알려 줄 수 없다. 스택을 다시 켤 것'); return 2
        t0 = time.time()
        while (pose() is None or not {'scan', 'v', 'map'} <= set(got)) and time.time() - t0 < 10:
            spin(0.1)
        if not {'scan', 'v'} <= set(got):
            print('❌ 스캔이나 오도메트리가 안 들어온다:', sorted(got)); return 2
        if abs(got['v'][0]) > 0.01 or abs(got['v'][1]) > 0.03:
            print(f'❌ 로봇이 움직이는 중이다(속도 {got["v"][0]:+.2f} m/s, {got["v"][1]:+.2f} rad/s) — 서 있을 때만 한다'); return 2
        print(f'지도 {os.path.basename(base)}', flush=True)
        agree('지금')
        if a.set:
            x, y, deg = (float(v) for v in a.set.split(',')); yaw = math.radians(deg)
            print(f'  (시험) 찾지 않고 ({x:+.2f}, {y:+.2f}) m, {deg:+.1f}° 를 알려 준다')
        else:
            r = locate(base, pose_check.scan_points(got['scan']), *whole_area(base)); b = r[0]; x, y, yaw = b['x'], b['y'], b['yaw']
            print(f'  스캔으로 찾은 자리: ({x:+.2f}, {y:+.2f}) m, {math.degrees(yaw):+.1f}° — 벽 위 {b["on_wall"] * 100:.0f}%, 점수 {b["score"]:.2f}'
                  + (f' (2등 {r[1]["score"]:.2f})' if len(r) > 1 else ''), flush=True)
            if b['on_wall'] < 0.85 or (len(r) > 1 and r[1]['score'] > 0.85 * b['score']):
                print('❌ 자리를 확신하지 못해 그대로 두었습니다. 로봇을 벽이 잘 보이는 곳에 두고 다시 누르세요.'); return 1
        if a.dry:
            return 0
        t0 = time.time()
        while pub.get_subscription_count() == 0 and time.time() - t0 < 3:
            spin(0.1)
        if pub.get_subscription_count() == 0:
            print('❌ /initialpose를 듣는 노드가 없다'); return 2
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'; m.header.stamp = n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        pub.publish(m); t0 = time.time(); p = None
        while time.time() - t0 < 10:              # 위치 잡기가 다음 스캔을 그 자리 근처에서 맞춰 보고 자세를 옮길 때까지
            spin(0.3); p = pose()
            if p and math.hypot(p[0] - x, p[1] - y) < 0.15 and abs((p[2] - yaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(5):
                break
        spin(1.0); ok, p = agree('알려 준 뒤')
        if a.set:                                 # 시험용: 위치 잡기가 그 근처에서 스스로 다시 맞추므로 알려 준 자리 그대로 안 갈 수 있다
            return 0
        if p is None or math.hypot(p[0] - x, p[1] - y) >= 0.15:
            print('❌ 위치 잡기가 찾은 자리로 옮겨 가지 않았습니다 — 다시 눌러 보세요.'); return 1
        print((f'✅ 자리를 다시 잡았습니다: 지도 위 ({p[0]:+.2f}, {p[1]:+.2f}) m, {math.degrees(p[2]):+.0f}°. 지도 화면의 로봇 위치가 실제와 맞는지 한 번 보세요.') if ok
              else '❌ 자리를 다시 잡았지만 스캔이 지도와 안 맞습니다 — 지도 화면에서 확인하세요.')
        return 0 if ok else 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
