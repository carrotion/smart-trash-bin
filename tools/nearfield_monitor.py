#!/usr/bin/env python3
# 사용: nearfield_monitor.py [초] — Nav2 로컬 코스트맵이 떠 있을 때. 2026-09-26 #10 1차 시도에서 물체가 코스트맵에 안 잡혀 원인 단계 구분용으로 작성
# 전방 근접 물체: 라이다 원본 / 필터 후 / 코스트맵 LETHAL을 1초마다 한 줄로
import rclpy, time, math, sys
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import Costmap
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('nf_monitor'); S={}
tfb=Buffer(); TransformListener(tfb,n)
n.create_subscription(LaserScan,'/scan_raw',lambda m:S.update(raw=m),5)
n.create_subscription(LaserScan,'/scan',lambda m:S.update(scan=m),5)
n.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:S.update(cm=m),2)
LX=0.054
def nearest(m):
    best=None
    for i,r in enumerate(m.ranges):
        if not (m.range_min<=r<=m.range_max): continue
        a=m.angle_min+i*m.angle_increment; x=LX+r*math.cos(a); y=r*math.sin(a)
        if x>0.10 and abs(y)<0.25:
            d=math.hypot(x,y)
            if best is None or d<best[0]: best=(d,x,y)
    return best
def cm_near():
    cm=S.get('cm')
    if cm is None: return None
    try: tf=tfb.lookup_transform(cm.header.frame_id,'base_link',rclpy.time.Time())
    except Exception: return 'TF?'
    bx,by=tf.transform.translation.x,tf.transform.translation.y; q=tf.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    res=cm.metadata.resolution; ox,oy=cm.metadata.origin.position.x,cm.metadata.origin.position.y; w,h=cm.metadata.size_x,cm.metadata.size_y
    best=None
    for my in range(h):
        for mx in range(w):
            c=cm.data[my*w+mx]
            if c<253 or c==255: continue
            wx=ox+(mx+0.5)*res-bx; wy=oy+(my+0.5)*res-by
            fx=wx*math.cos(yaw)+wy*math.sin(yaw); fy=-wx*math.sin(yaw)+wy*math.cos(yaw)
            if fx>0.05 and abs(fy)<0.25:
                d=math.hypot(fx,fy)
                if best is None or d<best: best=d
    return best
t0=time.time(); last=0; D=float(sys.argv[1]) if len(sys.argv)>1 else 120
while time.time()-t0<D:
    rclpy.spin_once(n,timeout_sec=0.05)
    if time.time()-last<1.0: continue
    last=time.time()
    f=lambda b: '없음' if b is None else f'{b[0]*100:.0f}cm(y {b[2]*100:+.0f})'
    c=cm_near(); cs='없음' if c is None else (c if isinstance(c,str) else f'{c*100:.0f}cm')
    print(f'[{time.time()-t0:5.1f}s] 원본 {f(nearest(S["raw"])) if "raw" in S else "-"} | 필터후 {f(nearest(S["scan"])) if "scan" in S else "-"} | 코스트맵 {cs}  (로봇 중심 기준, 앞면=11cm)',flush=True)
