#!/usr/bin/env python3
"""작업이 끝났음을 바퀴로 알린다 — 도크 위에서 앞뒤로 3번(사용자 요청, 2026-11-01: 메시지를 계속 지켜볼 수 없으니).

⚠️ 바닥에서 돌면 로봇이 움직이므로, 도크에 있을 때 떠 둔 스캔과 지금 스캔이 같을 때만 돈다(움직이지 않고 판단).
같은 자리 스캔 차이는 최대 0.1 cm(09-26 실측), 중앙값이라 지나가는 사람에 흔들리지 않는다. 기준 2 cm. 다르면 돌지 않고 알린다.

    python3 tools/wheel_signal.py --save /tmp/dock_scan.json   # 도크 위에서 기준 뜨기
    python3 tools/wheel_signal.py --base /tmp/dock_scan.json --dry   # 판단만
    python3 tools/wheel_signal.py --base /tmp/dock_scan.json   # 판단 후 알림
"""
import argparse, glob, json, os, sys, time, types, urllib.request

os.environ['ROS_DOMAIN_ID'] = '0'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dock_guard import scan_change


def as_dict(m):
    return dict(ranges=[float(r) for r in m.ranges], range_min=m.range_min, range_max=m.range_max, angle_min=m.angle_min, angle_increment=m.angle_increment)


def stack_running():
    """실물 자율 스택 런치가 떠 있는가. 명령줄 부분일치는 쓰지 않는다(이 문구가 든 셸까지 걸린다, 11-01 오판)."""
    for f in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            argv = open(f, 'rb').read().split(b'\0')
        except OSError:
            continue
        if len(argv) > 3 and argv[1].endswith(b'/ros2') and argv[2] == b'launch' and b'autonomous_mapping.launch.py' in argv:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--save'); ap.add_argument('--base'); ap.add_argument('--dry', action='store_true'); ap.add_argument('--reps', type=int, default=3)
    a = ap.parse_args()
    if not a.save and not a.base:
        ap.error('--save 또는 --base')
    rclpy.init(); n = rclpy.create_node('wheel_signal'); got = {}
    n.create_subscription(LaserScan, '/scan', lambda m: got.update(scan=m), qos_profile_sensor_data)
    t0 = time.time()
    while 'scan' not in got and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    try:
        if 'scan' not in got:
            print('❌ 스캔이 안 들어옴 — 돌지 않음'); return 2
        if a.save:
            json.dump(as_dict(got['scan']), open(a.save, 'w')); print(f'기준 스캔 저장: {a.save}'); return 0
        ch = scan_change(types.SimpleNamespace(**json.load(open(a.base))), got['scan'])
        print(f'도크 기준과 스캔 차이 {"비교 불가" if ch is None else f"{ch * 100:.1f} cm"}')
        if ch is None or ch > 0.02:
            print('❌ 도크 자리가 아니다 — 돌지 않음'); return 1
        if stack_running():
            print('❌ 자율 스택이 떠 있다 — 돌지 않음'); return 1
        if a.dry:
            print('도크 위로 판단(--dry: 돌지 않음)'); return 0
        pub = n.create_publisher(Twist, '/cmd_vel', 10); time.sleep(1.0)

        def send(v, secs):
            m = Twist(); m.linear.x = float(v); e = time.time() + secs
            while time.time() < e:
                pub.publish(m); time.sleep(0.05)
        for _ in range(a.reps):
            send(0.15, 1.5); send(0.0, 0.5); send(-0.15, 1.5); send(0.0, 0.5)
        send(0.0, 1.0); time.sleep(2.0)
        r = json.load(urllib.request.urlopen('http://localhost:8769/api/overview', timeout=3))['motor']['rate']
        ok = max(abs(x) for x in r) <= 50
        print(f'알림 끝. 2초 뒤 바퀴 속도(틱/초) {[round(x) for x in r]} → {"✅ 정지" if ok else "❌ 아직 돈다"}')
        return 0 if ok else 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
