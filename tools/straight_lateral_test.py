#!/usr/bin/env python3
# 사용: straight_lateral_test.py <속도들 콤마> [초]  예) 0.3,0.5 3.0 — ⚠️ 바닥 전용, 앞뒤 여유 검사
# 2026-09-26: 사용자 관찰 '30%·60%에서 오른쪽으로 벗어난 뒤 복귀를 제대로 못 한다' 확인용
# 직진 중 방향(자이로 적분)과 옆 밀림(EKF 위치, 출발 방향 기준 수직 거리) 추이
import rclpy, time, math, sys
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance
from turn_to_angle import yaw_of
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
rclpy.init(); n=rclpy.create_node('lateral'); S={'ekf':None,'scan':None}
n.create_subscription(Odometry,'/odometry/filtered',lambda m: S.update(ekf=(m.pose.pose.position.x,m.pose.pose.position.y,yaw_of(m.pose.pose.orientation))),20)
n.create_subscription(LaserScan,'/scan',lambda m:S.update(scan=m),5)
pub=n.create_publisher(Twist,'/cmd_vel',10)
def spin(t):
    e=time.time()+t
    while time.time()<e: rclpy.spin_once(n,timeout_sec=0.005)
t0=time.time()
while (S['ekf'] is None or S['scan'] is None) and time.time()-t0<10: spin(0.1)
DUR=float(sys.argv[2]) if len(sys.argv)>2 else 4.0
for V in [float(x) for x in sys.argv[1].split(',')]:
    for sv in (1,-1):
        need=V*DUR+0.4; clear=corridor_clearance(S['scan'],sv>0,0.0 if sv>0 else 0.1)
        if clear<need: print(f'❌ {"앞" if sv>0 else "뒤"} 여유 {clear:.2f} < {need:.2f} — 건너뜀'); continue
        spin(1.5); x0,y0,th0=S['ekf']; ts=time.time(); marks=[]; nxt=0.5
        while time.time()-ts<DUR:
            c=Twist(); c.linear.x=sv*V; pub.publish(c); spin(0.02)
            if time.time()-ts>=nxt:
                x,y,th=S['ekf']; dx,dy=x-x0,y-y0
                along=dx*math.cos(th0)+dy*math.sin(th0); lat=-dx*math.sin(th0)+dy*math.cos(th0)
                marks.append((nxt,along,lat,math.degrees((th-th0+math.pi)%(2*math.pi)-math.pi))); nxt+=0.5
        for _ in range(5): pub.publish(Twist()); spin(0.03)
        spin(0.8)
        s=' | '.join(f'{t:.1f}s 방향 {h:+.1f}° 옆 {l*100:+.1f}cm' for t,a,l,h in marks if abs(t-round(t))<0.01 or t==0.5)
        print(f'{V} m/s {"앞" if sv>0 else "뒤"}: {s} (진행 {marks[-1][1]*100:+.0f}cm)',flush=True)
