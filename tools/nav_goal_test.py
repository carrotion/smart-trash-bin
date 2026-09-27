#!/usr/bin/env python3
# Nav2 단일 목표 시험: 현재 odom 위치 기준 (dx, dy) m로 NavigateToPose. 사용: nav_goal_test.py dx dy [제한초]
# 1초마다 남은 거리·명령을 찍고 결과 상태(4=성공)를 출력. 2026-09-27 시뮬에서 DWB 목표 허용 오차 버그를 이걸로 찾음
import rclpy, time, math, sys
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n=rclpy.create_node('goal_test'); S={'odom':None,'cmd':[], 'nav':[]}
n.create_subscription(Odometry,'/odom',lambda m:S.update(odom=(m.pose.pose.position.x,m.pose.pose.position.y)),10)
n.create_subscription(Twist,'/cmd_vel',lambda m:S['cmd'].append((m.linear.x,m.angular.z)),50)
n.create_subscription(Twist,'/cmd_vel_nav',lambda m:S['nav'].append((m.linear.x,m.angular.z)),50)
TR=[]
ac=ActionClient(n,NavigateToPose,'navigate_to_pose')
if not ac.wait_for_server(timeout_sec=20): print('❌ 액션 서버 없음'); sys.exit(1)
t0=time.time()
while S['odom'] is None and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.1)
x0,y0=S['odom']; dx=float(sys.argv[1]) if len(sys.argv)>1 else 1.0; dy=float(sys.argv[2]) if len(sys.argv)>2 else 0.0
g=NavigateToPose.Goal(); g.pose.header.frame_id='odom'; g.pose.pose.position.x=x0+dx; g.pose.pose.position.y=y0+dy; g.pose.pose.orientation.w=1.0
f=ac.send_goal_async(g)
while not f.done(): rclpy.spin_once(n,timeout_sec=0.1)
h=f.result()
if not h.accepted: print('❌ 목표 거부'); sys.exit(1)
r=h.get_result_async(); t0=time.time()
last=0
while not r.done() and time.time()-t0<float(sys.argv[3] if len(sys.argv)>3 else 90):
    rclpy.spin_once(n,timeout_sec=0.1)
    if time.time()-last>=1.0 and S['odom']:
        last=time.time(); c=S['nav'][-1] if S['nav'] else (0,0)
        TR.append(f"  {time.time()-t0:5.1f}s 남은 거리 {math.hypot(x0+dx-S['odom'][0],y0+dy-S['odom'][1]):.2f} m | 명령 v {c[0]:+.3f} w {c[1]:+.2f}")
x1,y1=S['odom']; st=r.result().status if r.done() else 'timeout'
v=[a for a,b in S['cmd'] if abs(a)>1e-3]; vn=[a for a,b in S['nav'] if abs(a)>1e-3]
print('\n'.join(TR)); print(f'결과 상태 {st}(4=성공) {time.time()-t0:.1f}초 | 이동 ({x1-x0:+.2f},{y1-y0:+.2f}) m, 목표 ({dx:+.2f},{dy:+.2f}) | cmd_vel_nav {len(S["nav"])}개(최대 {max(vn,default=0):.2f}) → /cmd_vel {len(S["cmd"])}개(최대 {max(v,default=0):.2f})')
