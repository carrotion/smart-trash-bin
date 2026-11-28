#!/usr/bin/env python3
"""저장 지도 위 미션 이동 — 지도 좌표 목표를 차례로 주행기 액션(step_navigate_to_pose)에 보낸다 (2026-11-01).

탐색기 없이 "거기로 가" 하는 것의 시험·실사용 도구. 목표 하나가 실패하면 멈춘다(다음 목표로 넘어가지 않는다 — 어디서 왜 막혔는지 먼저 본다).
목표마다 배터리를 보고(11.4 V 미만이면 시작 안 함), 도착하면 지도 위 자세와 주행기가 잰 자리 검사 값을 남긴다. Ctrl-C/종료 신호면 목표를 취소한다.

    python3 tools/mission_go.py 북쪽방:-1.0,5.1 복도:-0.1,-5.0 제자리:0.66,0.49
    python3 tools/mission_go.py 거실:3.58,3.18,0.30 둘러보기:3.58,3.18,,90   # 이름:x,y[,장소 반경 m[,마지막 방향 도]]
"""
import json, math, os, signal, sys, time, urllib.request

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

MIN_V = 11.4
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def volts(settle_s=0.0):
    """배터리 전압. settle_s를 주면 그동안 몇 번 읽어 가장 높은 값(주행 직후 처짐이 풀린 값)을 돌려준다.
    11-28: 거실 도착 1초 뒤 11.38 V로 읽혀 다음 목표(안방)를 거절했다 — 3초 뒤엔 11.46~11.51 V였다(멈춰 있을 때 값)."""
    def one():
        try:
            return json.load(urllib.request.urlopen('http://localhost:8769/api/overview', timeout=3)).get('battery', {}).get('voltage')
        except Exception:
            return None
    vals = [one()]; end = time.time() + settle_s
    while time.time() < end:
        time.sleep(0.5); vals.append(one())
    vals = [v for v in vals if v is not None]
    return max(vals) if vals else None


def main():
    goals = []
    for g in sys.argv[1:]:
        name, xy = g.rsplit(':', 1); v = xy.split(',')
        goals.append((name, float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 and v[2] else None,   # 셋째 값 = 장소 반경(m, 11-27)
                      float(v[3]) if len(v) > 3 and v[3] else None))                                # 넷째 값 = 마지막 방향(도, 11-28)
    if not goals:
        print(__doc__); return 2
    # ⚠️ 11-14: Ctrl-C(SIGINT)에 rclpy 기본 처리기가 먼저 컨텍스트를 닫아, 취소 요청을 보내지 못한 채 죽었다 → 로봇은 목표로 계속 갔다(비상정지로 세움).
    #   rclpy의 신호 처리를 끄고 SIGINT도 SIGTERM처럼 "멈춤 표시"만 하게 해서, finally에서 살아 있는 컨텍스트로 목표를 취소한다
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n = rclpy.create_node('mission_go'); ac = ActionClient(n, NavigateToPose, 'step_navigate_to_pose')
    tfb = Buffer(); TransformListener(tfb, n); stop = []; current = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1)); signal.signal(signal.SIGINT, lambda *_: stop.append(1))

    def pose():
        try:
            t = tfb.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.0)).transform
        except Exception:
            return None
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def spin_until(f, secs):
        e = time.time() + secs
        while not f() and time.time() < e and not stop:
            rclpy.spin_once(n, timeout_sec=0.1)
        return f()

    out = os.path.join(REPO, 'docs/calibration', f'mission_{time.strftime("%Y%m%d_%H%M%S")}.json'); rec = []
    try:
        if not ac.wait_for_server(timeout_sec=20):
            print('❌ 주행기 액션 서버 없음'); return 2
        if not spin_until(lambda: pose() is not None, 10):
            print('❌ map→base_link 없음'); return 2
        for k, (name, x, y, area, yaw_deg) in enumerate(goals):
            v = volts(settle_s=3.0 if k else 0.0)          # 둘째 목표부터는 앞 주행의 전압 처짐이 풀리기를 잠깐 기다린다
            if v is not None and v < MIN_V:
                print(f'🔋 {v:.2f} V — {MIN_V} V 미만이라 "{name}" 시작 안 함'); return 3
            p0 = pose(); t0 = time.time()
            print(f'▶ {name} ({x:+.2f}, {y:+.2f}) ← 지금 ({p0[0]:+.2f}, {p0[1]:+.2f}), 직선 {math.hypot(x - p0[0], y - p0[1]):.1f} m, {v} V', flush=True)
            g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.header.stamp = n.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = x, y; g.pose.pose.orientation.w = 1.0
            bt = ([f'area_m={area:.2f}'] if area else []) + ([f'yaw_deg={yaw_deg:.1f}'] if yaw_deg is not None else [])
            # NAV_SET='step_long_m=0.5;open_side_m=0.40' — 이 실행의 목표만 주행기 설정을 바꾼다(11-28 A/B 바닥 시험)
            bt += [f'set.{kv.strip()}' for kv in os.environ.get('NAV_SET', '').split(';') if kv.strip()]
            if bt:
                g.behavior_tree = ';'.join(bt)                 # 장소 반경(막혀도 이 안이면 도착)·마지막 방향 — step_nav_action_server가 읽음
            fg = ac.send_goal_async(g)
            if not spin_until(fg.done, 10) or not fg.result().accepted:
                print('❌ 목표 거절됨'); return 2
            gh = fg.result(); current[:] = [gh]; fr = gh.get_result_async()
            while not fr.done() and not stop:
                rclpy.spin_once(n, timeout_sec=0.1)
            if stop:
                return 4
            res = fr.result(); ok = res.status == 4; current.clear()
            spin_until(lambda: False, 1.0); p1 = pose() or p0; err = math.hypot(x - p1[0], y - p1[1]); secs = time.time() - t0
            print(f'{"✅" if ok else "❌"} {name}: {secs:.0f}초, 지도 위 도착 오차 {err * 100:.1f} cm, 자세 ({p1[0]:+.2f}, {p1[1]:+.2f}, {math.degrees(p1[2]):+.0f}°)'
                  + ('' if ok else f' — {res.result.error_msg}'), flush=True)
            rec.append(dict(name=name, goal=(x, y), start=p0, end=p1, ok=ok, secs=round(secs, 1), err_m=round(err, 3), msg=res.result.error_msg, volts=v))
            json.dump(rec, open(out, 'w'), ensure_ascii=False, indent=1)
            if not ok:
                print(f'❌ 미션 중단 — 기록 {out}'); return 1
        print(f'✅ 미션 완료 — 기록 {out}'); return 0
    except KeyboardInterrupt:
        return 4
    finally:
        if current:
            print('목표 취소'); fc = current[0].cancel_goal_async(); e = time.time() + 3
            while not fc.done() and time.time() < e:
                rclpy.spin_once(n, timeout_sec=0.1)
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
