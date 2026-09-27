#!/usr/bin/env python3
# Nav2 단일 목표 시험: 로봇 **정면 기준** (앞 dx, 왼쪽 dy) m 지점으로 NavigateToPose(odom 프레임, EKF 자세 기준).
# 사용: nav_goal_test.py dx dy [제한초]. 제한 시간이 지나면 **목표를 취소**하고 정지 명령을 보낸다.
# ⚠️ 09-27 첫 실물 시도에서 odom x축 방향·바퀴 /odom 기준으로 목표를 잡아 로봇 정면이 아니었고(+25°), 끝나도 목표가 살아 있었다
# 1초마다 남은 거리·명령을 찍고 결과 상태(4=성공)를 출력. 2026-09-27 시뮬에서 DWB 목표 허용 오차 버그를 이걸로 찾음
import rclpy, time, math, sys, os
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n=rclpy.create_node('goal_test'); S={'odom':None,'cmd':[], 'nav':[]}
def _od(m):
    q=m.pose.pose.orientation
    S.update(odom=(m.pose.pose.position.x,m.pose.pose.position.y), yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
n.create_subscription(Odometry,os.environ.get('NAV_GOAL_ODOM','/odometry/filtered'),_od,10)   # Nav2 odom 프레임 = EKF(시뮬은 NAV_GOAL_ODOM=/odom)
n.create_subscription(Twist,'/cmd_vel',lambda m:S['cmd'].append((m.linear.x,m.angular.z)),50)
n.create_subscription(Twist,'/cmd_vel_nav',lambda m:S['nav'].append((m.linear.x,m.angular.z)),50)
TR=[]
ac=ActionClient(n,NavigateToPose,'navigate_to_pose')
if not ac.wait_for_server(timeout_sec=20): print('❌ 액션 서버 없음'); sys.exit(1)
t0=time.time()
while S['odom'] is None and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.1)
x0,y0=S['odom']; th=S['yaw']; fx=float(sys.argv[1]) if len(sys.argv)>1 else 1.0; fy=float(sys.argv[2]) if len(sys.argv)>2 else 0.0
dx=fx*math.cos(th)-fy*math.sin(th); dy=fx*math.sin(th)+fy*math.cos(th)   # 로봇 정면 기준 → odom 프레임
g=NavigateToPose.Goal(); g.pose.header.frame_id='odom'; g.pose.pose.position.x=x0+dx; g.pose.pose.position.y=y0+dy
g.pose.pose.orientation.z=math.sin(th/2); g.pose.pose.orientation.w=math.cos(th/2)   # 도착 방향 = 출발 방향
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
if not r.done():   # 제한 시간 초과 → 목표 취소 + 정지(목표가 살아서 계속 움직이지 않게)
    cf=h.cancel_goal_async(); t1=time.time()
    while not cf.done() and time.time()-t1<3: rclpy.spin_once(n,timeout_sec=0.05)
    sp=n.create_publisher(Twist,'/cmd_vel_nav',10)
    for _ in range(5): sp.publish(Twist()); rclpy.spin_once(n,timeout_sec=0.05)
    print('⏱ 제한 시간 초과 — 목표 취소함')
x1,y1=S['odom']; st=r.result().status if r.done() else 'timeout'
v=[a for a,b in S['cmd'] if abs(a)>1e-3]; vn=[a for a,b in S['nav'] if abs(a)>1e-3]
print('\n'.join(TR)); print(f'결과 상태 {st}(4=성공) {time.time()-t0:.1f}초 | 이동 ({x1-x0:+.2f},{y1-y0:+.2f}) m, 목표 ({dx:+.2f},{dy:+.2f}) | cmd_vel_nav {len(S["nav"])}개(최대 {max(vn,default=0):.2f}) → /cmd_vel {len(S["cmd"])}개(최대 {max(v,default=0):.2f})')
