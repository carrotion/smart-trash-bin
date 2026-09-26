#!/usr/bin/env python3
"""정지 → 직진 출발 시 바퀴별 출발 지연·출발 거리와 출발 직후 방향 틀어짐 측정.

앞/뒤를 번갈아 달려 제자리 근처에 머문다(앞뒤 0.3 m 여유 검사). 첫 출발 뒤 도크 검사(dock_guard).
velocity 모드 diff_drive_node 실행 중일 것. ⚠️ 바닥 전용.

    python3 tools/start_lag_test.py <라벨> <속도 m/s> <반복> [달리는 초, 기본 1.0]

2026-09-26: 사용자 관찰 "출발할 때 방향이 살짝 틀어진다"를 이 도구로 재현(0.5초에 3~4°)하고,
뒷바퀴 출발 보정(ff_pwm0/ff_pwm0_kinetic) 좌우 비대칭이 원인임을 확인 → docs/todo.md 1f.
"""
import rclpy, time, math, sys, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance
from wheel_config import NAMES, wheel_distances
from dock_guard import looks_on_dock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray
rclpy.init(); n = rclpy.create_node('start_lag'); S = {'g': [], 'e': [], 'scan': None}
n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S['e'].append((time.time(), list(m.data))), 50)
n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.005)
t0 = time.time()
while (S['scan'] is None or not S['e'] or len(S['g']) < 50) and time.time() - t0 < 10: spin(0.1)
label, V, reps = sys.argv[1], float(sys.argv[2]), int(sys.argv[3])
DUR = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
rows = []
for rep in range(reps):
    for sgn in (1, -1):
        clear = corridor_clearance(S["scan"], sgn > 0, 0.0 if sgn > 0 else 0.1)
        if clear < 0.3:
            print(f'❌ {"앞" if sgn > 0 else "뒤"} 여유 {clear:.2f} m — 중단'); break
        scan0 = S['scan']
        spin(1.5)   # 완전 정지 후 출발(구동 노드 전환 정지 간격보다 길게)
        S['g'] = []; S['e'] = [S['e'][-1]]; e0 = S['e'][0][1]; ts = time.time()
        while time.time() - ts < DUR:
            c = Twist(); c.linear.x = sgn * V; pub.publish(c); spin(0.02)
        for _ in range(5): pub.publish(Twist()); spin(0.03)
        spin(0.5)
        if rep == 0 and sgn > 0:   # 첫 출발 뒤 도크 검사
            d = wheel_distances(e0, S['e'][-1][1]); bad, msg = looks_on_dock((d[0] + d[1]) / 2, scan0, S['scan']); print(msg)
            if bad: break
        lag = []
        for k in range(4):
            first = next((t - ts for t, d in S['e'][1:] if abs(d[k] - e0[k]) >= 3), None)
            lag.append(first)
        yaw05 = 0.0; last = ts
        for t, v in S['g']:
            if ts <= t <= ts + 0.5: yaw05 += v * (t - last); last = t
            elif t > ts + 0.5: break
        yaw10 = 0.0; last = ts
        for t, v in S['g']:
            if ts <= t <= ts + 1.0: yaw10 += v * (t - last); last = t
        def dist_at(tt):
            c = [d for t, d in S['e'][1:] if t - ts <= tt]
            return wheel_distances(e0, c[-1]) if c else [0, 0, 0, 0]
        D = {tt: [x * sgn * 100 for x in dist_at(tt)] for tt in (0.3, 0.5, 1.0)}
        print('   바퀴별 전진거리 cm (M1 M2 M3 M4) ' + ' | '.join(f'{tt}s: ' + ' '.join(f'{x:4.1f}' for x in D[tt]) for tt in D), flush=True)
        hs = []
        for T in (0.5, 1.0, 1.5, 2.0, 2.5):
            if T > DUR + 0.01: break
            y = 0.0; last = ts
            for t, v in S['g']:
                if ts <= t <= ts + T: y += v * (t - last); last = t
            hs.append(f'{T}s {math.degrees(y):+.1f}°')
        print('   방향 추이: ' + ', '.join(hs), flush=True)
        r = dict(dist=D, label=label, v=V, dir='앞' if sgn > 0 else '뒤', rep=rep + 1, lag=lag,
                 yaw05_deg=math.degrees(yaw05), yaw10_deg=math.degrees(yaw10))
        rows.append(r)
        ls = ' '.join(f'{NAMES[k].split()[0]} {("%.2f" % x) if x is not None else "--"}s' for k, x in enumerate(lag))
        print(f'{label} {r["dir"]} #{rep+1}: 출발지연 {ls} | 방향 0.5초 {r["yaw05_deg"]:+.1f}° 1초 {r["yaw10_deg"]:+.1f}°', flush=True)
    else:
        continue
    break
out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs/calibration', f'start_lag_{time.strftime("%Y%m%d_%H%M")}_{label}.json')
json.dump(rows, open(out, 'w'), ensure_ascii=False, indent=1)
print('저장:', out)
