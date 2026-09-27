#!/usr/bin/env python3
"""4층 멈춤-확인-이동 합격 시험 — 정해진 코스를 여러 번 돌려 숫자로 판정(docs/motion_foundation.md 4층).

코스(출발 자세 기준, 앞 x·왼쪽 y, m):
  loop     : (1.0, 0) → (1.0, 0.8) → 출발점 복귀(출발 방향 맞춤)       필요 공간 앞 1.8·왼쪽 1.2·뒤/오른쪽 0.4
  obstacle : (1.4, 0) → 출발점 복귀(출발 방향 맞춤)  — 앞 0.6 m에 병을 둔 채로(우회 필요)
구간마다: 도착 오차(SLAM), 방향 오차(지정 시), 단계·회전·멈칫 수, 시간, 회전 최대(자이로 0.2초 평균).
복귀 구간마다: **출발 스캔과 라이다 ICP로 실제 어긋남**(작은 이동이라 ICP가 믿을 만함 — 1 m 이동엔 부적합, 09-27).

판정(안): 구간 도착 ≤ 10 cm·방향 ≤ 5° **90%**, 실제 복귀 ≤ 10 cm·5° **90%**, 회전마다 최대(0.2초) ≤ 1.2 rad/s **95%**
(1층 ④ 기준을 그대로 물려받음 — 09-27 처음 적은 1.04(=1.3×0.8)는 ④ 기준이 절대값 1.2로 바뀌기 전 계산이 남은 것),
충돌 0(사람 관찰). 구간 하나라도 실패(주행기 정지)하면 그 자리에서 멈추고 보고한다(위치를 모르는 채 다음 목표로 가지 않음).

    tools/nav_test_stack.sh start
    python3 tools/step_nav_acceptance.py --course loop --reps 3
    python3 tools/step_nav_acceptance.py --course obstacle --reps 2
"""
import argparse, json, math, os, subprocess, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step_navigator import StepNavigator
from turn_to_angle import wrap
from wheel_config import wheel_distances
from dock_guard import looks_on_dock
from drive_distance_test import icp_motion

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COURSES = {'loop': [(1.0, 0.0, None), (1.0, 0.8, None), (0.0, 0.0, 0.0)],
           'obstacle': [(1.4, 0.0, None), (0.0, 0.0, 0.0)]}
TOL_M, TOL_DEG, PEAK_MAX, PEAK_RATIO, RATIO = 0.10, 5.0, 1.2, 0.95, 0.9


def peak_rate(samples, win=0.2):
    """(t, gz) 목록의 win초 이동평균 |gz| 최댓값."""
    best, j, acc = 0.0, 0, 0.0
    for i, (t, g) in enumerate(samples):
        acc += g
        while samples[j][0] < t - win:
            acc -= samples[j][1]; j += 1
        n = i - j + 1
        if n >= 3:
            best = max(best, abs(acc / n))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--course', choices=list(COURSES), default='loop')
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    bat = subprocess.run(['bash', '-c', f'tail -1 {REPO}/logs/battery/$(date +%Y%m%d).csv | cut -d, -f3'],
                         capture_output=True, text=True).stdout.strip()
    print(f'배터리 {bat or "?"} V' + (' ⚠️ 11.4 V 미만 — 결과가 전압 탓일 수 있음' if bat and float(bat) < 11.4 else ''), flush=True)

    rclpy.init(); n = rclpy.create_node('step_nav_acceptance')
    nav = StepNavigator(n)
    G = {'g': [], 'enc': None}
    turn_peaks = []
    _turn_by = nav.turner.turn_by

    def turn_by(d):   # 회전마다 최대(0.2초 평균) 기록
        i = len(G['g']); r = _turn_by(d); pk = round(peak_rate(G['g'][i:]), 2)
        turn_peaks.append(dict(deg=round(math.degrees(d), 1), peak_rad_s=pk)); r['peak_rad_s'] = pk
        print(f'      회전 {math.degrees(d):+.0f}° 최대 {pk} rad/s', flush=True)
        return r
    nav.turner.turn_by = turn_by
    n.create_subscription(Imu, '/imu/data', lambda m: G['g'].append((time.monotonic(), m.angular_velocity.z)), 100)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: G.update(enc=list(m.data)), 20)
    t0 = time.time()
    while (nav.S['pose'] is None or nav.S['scan'] is None or G['enc'] is None or nav.map_pose() is None) and time.time() - t0 < 15:
        nav.spin(0.2)
    if nav.map_pose() is None or G['enc'] is None:
        print('❌ SLAM 위치(map→base_link) 또는 엔코더 없음 — nav_test_stack.sh start 했나?'); return 1
    if n.count_subscribers('/cmd_vel_nav') == 0:
        print('❌ /cmd_vel_nav 구독자(collision_monitor) 없음 — 명령이 바퀴까지 안 간다'); return 1

    nav.settle(); nav.spin(0.5)
    p0 = nav.map_pose(); scan0 = nav.S['scan']; enc0 = list(G['enc'])
    c0, s0 = math.cos(p0[2]), math.sin(p0[2])

    def to_map(wp):
        x, y, yaw = wp
        return (p0[0] + c0 * x - s0 * y, p0[1] + s0 * x + c0 * y, None if yaw is None else wrap(p0[2] + math.radians(yaw)))

    print(f'코스 {a.course} × {a.reps}회, 출발 map ({p0[0]:+.2f},{p0[1]:+.2f},{math.degrees(p0[2]):+.0f}°)', flush=True)
    legs, returns, stop_msg = [], [], ''
    try:
        for rep in range(a.reps):
            for k, wp in enumerate(COURSES[a.course]):
                goal = to_map(wp); G['g'] = []; nlog = len(nav.log)
                print(f'[{rep + 1}회 {k + 1}구간] 목표 앞 {wp[0]:.1f}·왼쪽 {wp[1]:.1f} m' + (' (출발 방향)' if wp[2] is not None else ''), flush=True)
                r = nav.go(goal)
                ev = nav.log[nlog:]
                leg = dict(rep=rep + 1, leg=k + 1, wp=wp, ok=r['ok'], msg=r['msg'], secs=round(r['secs'], 1),
                           err_cm=round(r['final_err_m'] * 100, 1),
                           yaw_err_deg=None if r['final_yaw_err_deg'] is None else round(r['final_yaw_err_deg'], 1),
                           steps=r['steps'], turns=sum(e['kind'] in ('turn', 'final_turn') for e in ev),
                           hesitations=sum(e['kind'] in ('blocked', 'plan_fail') or (e['kind'] == 'drive' and not e.get('ok', True)) for e in ev),
                           peak_rad_s=round(peak_rate(G['g']), 2), events=ev)
                legs.append(leg)
                print(f'   → {"✅" if r["ok"] else "❌"} {r["msg"]} | {leg["err_cm"]} cm'
                      + (f', {leg["yaw_err_deg"]:+.1f}°' if leg['yaw_err_deg'] is not None else '')
                      + f' | 단계 {leg["steps"]}, 회전 {leg["turns"]}, 멈칫 {leg["hesitations"]}, {leg["secs"]} s, 회전 최대 {leg["peak_rad_s"]} rad/s', flush=True)
                if rep == 0 and k == 0:   # ⚠️ 09-26 도크 위 시험 재발 방지
                    d = wheel_distances(enc0, G['enc'])
                    bad, msg = looks_on_dock((d[0] + d[1]) / 2, scan0, nav.S['scan'])
                    print('   ' + msg, flush=True)
                    if bad:
                        stop_msg = '도크 의심'; return 1
                if not r['ok']:
                    stop_msg = f'{rep + 1}회 {k + 1}구간 실패: {r["msg"]}'; return 2
                if wp[:2] == (0.0, 0.0):
                    nav.settle(); nav.spin(1.0)
                    here = nav.map_pose()
                    ex, ey = here[0] - p0[0], here[1] - p0[1]
                    guess = (c0 * ex + s0 * ey, -s0 * ex + c0 * ey, wrap(here[2] - p0[2]))
                    tx, ty, th, resid = icp_motion(scan0, nav.S['scan'], guess)
                    rt = dict(rep=rep + 1, real_cm=round(math.hypot(tx, ty) * 100, 1), real_x_cm=round(tx * 100, 1),
                              real_y_cm=round(ty * 100, 1), real_deg=round(th, 1), slam_cm=round(math.hypot(*guess[:2]) * 100, 1),
                              slam_deg=round(math.degrees(guess[2]), 1), icp_resid_cm=round(resid * 100, 1))
                    returns.append(rt)
                    print(f'   복귀 실제(라이다 ICP): {rt["real_cm"]} cm (앞 {rt["real_x_cm"]:+}, 왼 {rt["real_y_cm"]:+}), {rt["real_deg"]:+}°'
                          f' | SLAM 판단 {rt["slam_cm"]} cm, {rt["slam_deg"]:+}° | 잔차 {rt["icp_resid_cm"]} cm', flush=True)
    except KeyboardInterrupt:
        stop_msg = '사용자 중단'
    finally:
        for _ in range(5):
            nav.pub.publish(Twist()); nav.spin(0.03)
        arr = [lg for lg in legs if lg['ok'] and lg['err_cm'] <= TOL_M * 100 and (lg['yaw_err_deg'] is None or abs(lg['yaw_err_deg']) <= TOL_DEG)]
        ret = [x for x in returns if x['real_cm'] <= TOL_M * 100 and abs(x['real_deg']) <= TOL_DEG]
        peak = max([lg['peak_rad_s'] for lg in legs] or [0.0])
        r_peak = sum(t['peak_rad_s'] <= PEAK_MAX for t in turn_peaks) / len(turn_peaks) if turn_peaks else 1.0
        planned = a.reps * len(COURSES[a.course])
        r_arr = len(arr) / planned; r_ret = len(ret) / a.reps
        verdict = r_arr >= RATIO and r_ret >= RATIO and r_peak >= PEAK_RATIO and not stop_msg
        print(f'\n=== 4층 합격 시험({a.course}) {"✅ 통과" if verdict else "❌ 불통과"} ===')
        print(f'구간 도착 ≤ 10 cm·5°: {len(arr)}/{planned} = {r_arr:.0%} (기준 90%)')
        print(f'실제 복귀 ≤ 10 cm·5°: {len(ret)}/{a.reps} = {r_ret:.0%} (기준 90%)')
        print(f'회전마다 최대 ≤ {PEAK_MAX} rad/s: {sum(t["peak_rad_s"] <= PEAK_MAX for t in turn_peaks)}/{len(turn_peaks)} = {r_peak:.0%} (기준 95%), 전체 최대 {peak:.2f}')
        if stop_msg:
            print(f'중단: {stop_msg}')
        print('충돌 여부는 사람 관찰로 확인(기준 0회)')
        out = os.path.join(REPO, 'docs/calibration', f'step_nav_accept_{time.strftime("%Y%m%d_%H%M")}_{a.course}{("_" + a.label) if a.label else ""}.json')
        json.dump(dict(course=a.course, reps=a.reps, battery_v=bat, start=p0, legs=legs, returns=returns, turn_peaks=turn_peaks, stop=stop_msg,
                       arrive_ratio=r_arr, return_ratio=r_ret, peak_rad_s=peak, peak_ratio=r_peak, passed=verdict),
                  open(out, 'w'), ensure_ascii=False, indent=1, default=str)
        print('저장:', out)
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
