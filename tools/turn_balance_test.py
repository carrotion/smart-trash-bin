#!/usr/bin/env python3
"""제자리 회전 좌우 출력 균형 측정 — 균형값별 90° 회전 밀림을 라이다로 잰다.

배경(2026-09-25): 회전 출발 매트릭스에서 우회전만 90°마다 10~14 cm 밀렸다(좌회전 1~4 cm,
회전 중심 약 8 cm 치우침). diff_drive_node의 turn_balance[방향] = b는 오른쪽 출력 ×√b,
왼쪽 ÷√b. 균형값을 바꿔 가며 밀림이 최소인 b를 찾는다.
사용자 지적: 여러 번 돌며 벽에 닿을 뻔했다 → 매 회전 전 주변 여유 확인.

균형값마다 reps회: [대상 방향 90° → 반대 방향 90°(기본 균형)로 복귀], 각 회전의 밀림
크기와 방향(출발 자세 기준 dx, dy)을 기록.

    python3 tools/turn_balance_test.py --dir cw --balances 0.8,0.9,1.0,1.1,1.2,1.3,1.4
결과: docs/calibration/turn_balance_YYYYMMDD_HHMM.json
"""
import argparse, json, math, os, statistics as st, sys, time
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Imu, LaserScan

from rotation_quality_test import scan_points
from turn_start_matrix import icp_with_guess

MIN_CLEAR = 0.35


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', choices=('cw', 'ccw'), default='cw')
    ap.add_argument('--balances', default='0.8,0.9,1.0,1.1,1.2,1.3,1.4')
    ap.add_argument('--w', type=float, default=0.4)
    ap.add_argument('--reps', type=int, default=2)
    args = ap.parse_args()
    balances = [float(x) for x in args.balances.split(',')]
    tgt_sgn = -1 if args.dir == 'cw' else 1
    idx = 1 if args.dir == 'cw' else 0

    rclpy.init()
    n = rclpy.create_node('turn_balance_test')
    S = {'g': [], 'scan': None}
    n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    pcli = n.create_client(SetParameters, '/diff_drive_node/set_parameters')

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.01)

    def set_balance(pair):
        req = SetParameters.Request(parameters=[Parameter(name='turn_balance', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(x) for x in pair]))])
        fut = pcli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
        return fut.result() is not None

    def stop(settle):
        for _ in range(5):
            pub.publish(Twist()); spin(0.03)
        spin(settle)

    def turn90(sgn, bias):
        """90° 회전 후 (dx, dy, dθ, 잔차, 여유). 여유 부족이면 None."""
        before = scan_points(S['scan'])
        near = min((math.hypot(x, y) for x, y in before), default=9)
        if near < MIN_CLEAR:
            return None, near
        S['g'] = []; ts = time.time(); last = ts; yaw = 0.0
        while abs(yaw) < math.pi / 2 - args.w * 0.12 and time.time() - ts < math.pi / 2 / args.w + 6:
            c = Twist(); c.angular.z = sgn * args.w; pub.publish(c); spin(0.02)
            now = time.time()
            g = [v for t, v in S['g'] if t > last]
            if g:
                yaw += (st.mean(g) - bias) * (now - last)
            last = now
        t_stop = time.time()
        stop(1.2)
        yaw += sum((v - bias) * 0.02 for t, v in S['g'] if t > t_stop)
        after = scan_points(S['scan'])
        return icp_with_guess(after, before, math.degrees(yaw)), near

    t0 = time.time()
    while (S['scan'] is None or len(S['g']) < 50) and time.time() - t0 < 10:
        spin(0.1)
    if not pcli.wait_for_service(timeout_sec=3.0):
        print('❌ diff_drive_node 파라미터 서비스 없음'); return 1
    S['g'] = []
    spin(3.0)
    bias = st.mean(g for _, g in S['g'])

    rows = []
    try:
        for b in balances:
            pair = [1.0, 1.0]; pair[idx] = b
            if not set_balance(pair):
                print('❌ 파라미터 설정 실패'); return 1
            for rep in range(args.reps):
                res, near = turn90(tgt_sgn, bias)
                if res is None:
                    print(f'❌ 주변 최근접 {near:.2f} m — 밀려서 여유 부족, 중단'); return 1
                dx, dy, dth, resid = res
                rows.append({'balance': b, 'dir': args.dir, 'rep': rep + 1, 'dx_cm': dx * 100, 'dy_cm': dy * 100,
                             'shift_cm': math.hypot(dx, dy) * 100, 'dtheta': dth, 'resid_cm': resid * 100,
                             'clear_m': near})
                # 복귀(반대 방향, 기본 균형)
                set_balance([1.0, 1.0])
                back, near2 = turn90(-tgt_sgn, bias)
                set_balance(pair)
                bshift = math.hypot(back[0], back[1]) * 100 if back else None
                rows[-1]['return_shift_cm'] = bshift
                print(f'균형 {b:.2f} #{rep + 1}: {args.dir.upper()} 밀림 {math.hypot(dx, dy) * 100:5.1f} cm '
                      f'(앞뒤 {dx * 100:+5.1f}, 좌우 {dy * 100:+5.1f}) 회전 {dth:+.1f}° | 여유 {near:.2f} m | '
                      f'복귀 회전 밀림 {bshift:.1f} cm' if bshift is not None else '', flush=True)
                if back is None:
                    print(f'❌ 복귀 전 여유 {near2:.2f} m — 중단'); return 1
    finally:
        stop(0.3)
        set_balance([1.0, 1.0])

    print('\n=== 균형값별 평균 밀림 ===')
    for b in balances:
        r = [x for x in rows if x['balance'] == b]
        if r:
            print(f'  {b:.2f}: {st.mean(x["shift_cm"] for x in r):5.1f} cm '
                  f'(앞뒤 {st.mean(x["dx_cm"] for x in r):+5.1f}, 좌우 {st.mean(x["dy_cm"] for x in r):+5.1f})')
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(repo, time.strftime('docs/calibration/turn_balance_%Y%m%d_%H%M.json'))
    json.dump(rows, open(path, 'w'), ensure_ascii=False, indent=1)
    print('저장:', path)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
