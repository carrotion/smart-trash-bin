#!/usr/bin/env python3
"""금지 구역 서버 — 구역 파일(tools/keepout.py 형식)을 읽어 자율 스택에 알린다. 자율 스택과 함께 뜬다(런치).

내는 것(전부 마지막 값을 늦게 붙은 구독자도 받는 방식):
  /keepout_filter_info  nav2_msgs/CostmapFilterInfo  — 전역 코스트맵 KeepoutFilter가 이걸 보고 격자 토픽을 구독
  /keepout_filter_mask  nav_msgs/OccupancyGrid       — 구역 격자(경로 계획이 피함)
  /keepout_zones        std_msgs/String(JSON)        — 구역 꼭짓점(주행기가 가상의 벽으로 씀, 대시보드가 "적용됨" 확인)
파일이 바뀌면 1초 안에 다시 낸다(대시보드가 파일을 고친다 — 서로 직접 부르지 않는다).

⚠️ 켜질 때 파일의 구역을 prev로 치우고 비운다: SLAM이 새 지도를 새 원점으로 그리므로 지난 구역은 좌표가 맞지 않는다.
   --keep을 주면 그대로 둔다(시뮬·같은 지도에서 서버만 다시 켤 때).

    python3 tools/keepout_server.py [--file ~/maps/keepout_zones.json] [--keep]
"""
import argparse, array, json, os, sys

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keepout

RES, HALF = 0.05, 20.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=keepout.FILE)
    ap.add_argument('--keep', action='store_true', help='켜질 때 구역을 비우지 않는다')
    a, _ = ap.parse_known_args()        # --ros-args …(재생 도구의 use_sim_time)는 rclpy가 sys.argv에서 읽는다 — 11-14 재생에서 거절돼 구역 서버가 안 떴다
    if not (a.keep or os.environ.get('KEEPOUT_KEEP')):      # 저장 지도로 시작할 땐 그 지도의 구역을 그대로 쓴다(런치가 환경 변수로 알림)
        d = keepout.load(a.file)
        if d['zones']:
            keepout.save(dict(d, prev=d['zones'], zones=[]), a.file)
            print(f'새 지도 시작: 지난 구역 {len(d["zones"])}개를 치워 둠(화면에서 다시 불러 쓸 수 있음)', flush=True)
    rclpy.init(); n = rclpy.create_node('keepout_server')
    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    p_info = n.create_publisher(CostmapFilterInfo, '/keepout_filter_info', latched)
    p_mask = n.create_publisher(OccupancyGrid, '/keepout_filter_mask', latched)
    p_zone = n.create_publisher(String, '/keepout_zones', latched)
    info = CostmapFilterInfo(); info.header.frame_id = 'map'; info.type = 0; info.filter_mask_topic = '/keepout_filter_mask'
    info.base = 0.0; info.multiplier = 1.0
    p_info.publish(info)
    seen = {'key': None}

    def tick():
        try:
            key = os.stat(a.file).st_mtime_ns
        except OSError:
            key = 0
        if key == seen['key']:
            return
        seen['key'] = key
        d = keepout.load(a.file)
        try:
            zones = keepout.clean_zones(d['zones'])
        except ValueError as e:
            print(f'⚠️ 구역 파일 오류 — 적용하지 않음: {e}', flush=True); return
        m, ox, oy = keepout.build_mask(zones, RES, HALF)
        g = OccupancyGrid(); g.header.frame_id = 'map'; g.header.stamp = n.get_clock().now().to_msg()
        g.info.resolution = RES; g.info.height, g.info.width = m.shape
        g.info.origin.position.x, g.info.origin.position.y = ox, oy; g.info.origin.orientation.w = 1.0
        g.data = array.array('b', m.tobytes())
        p_mask.publish(g)
        p_zone.publish(String(data=json.dumps(dict(version=d['version'], zones=zones), ensure_ascii=False)))
        print(f'금지 구역 {len(zones)}개 적용(version {d["version"]}, 통행 불가 칸 {int((m == 100).sum())})', flush=True)
    tick()
    n.create_timer(1.0, tick)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
