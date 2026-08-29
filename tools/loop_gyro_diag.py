#!/usr/bin/env python3
"""사용자가 폰/대시보드로 직접 조종하는 동안 자이로/오도메트리를 연속 로깅.

round_trip_gyro_diag.py는 스크립트가 직접 cmd_vel을 몰아서 왕복시키지만,
이건 **순수 수동 조종 시험용** — 로봇을 전혀 조종하지 않고 /imu/data(원시
자이로), /odom(휠), /odometry/filtered(EKF)만 구독해서 시간축으로 기록한다.
Layer 1 매핑 시험(거실->A방->복귀)에서 "최종 지도만 있고 과정을 못 본다"는
문제를 풀기 위한 것 — 나중에 로그 타임스탬프와 지도/기억을 대조해서 정확히
어느 구간에서 요(yaw) 오차가 붙는지 짚어낼 수 있다.

Ctrl+C로 멈추면 그때까지 기록을 저장한다.

2026-08-29: 배터리 전압(INA219, 0x40)도 같이 기록한다 — 순간적인 전압
강하(모터 대전류 구간)가 휠 오도메트리 동결과 시간적으로 겹치는지 보기 위함
(`docs/odometry_comprehensive_review_20260824.md`의 후보 2번: 모터드라이버
보드 전압 처짐 가설). motor_dashboard.py의 안전용 이동평균과 달리 여기는
**원시값**을 그대로 기록한다 — 순간 처짐 자체가 관심사라서 평균으로 뭉개면
안 된다. IMU와 같은 I2C 버스 1을 쓰지만 커널 i2c-dev 트랜잭션은 프로세스 간
원자적이라(시리얼 포트 공유와 다름) 충돌 위험은 없음, 다만 버스 부하를 아끼려
5Hz로 제한한다(50Hz인 /imu/data 콜백마다 읽지 않음).

2026-08-29 추가: `/wheel_encoders`(4채널 원시 누적 카운트)도 같이 기록한다.
이전 시험에서 odom_yaw가 몇십~백여 초씩 동결되는 동안 EKF/자이로는 계속
실제로 움직이는 패턴을 발견했는데, 이게 diff_drive_node의 "엔코더 응답
자체가 안 옴" 경고로는 하나도 안 잡혔다 — 즉 응답은 매번 정상으로 왔지만
그 안의 **특정 채널 값만 안 바뀌는(고착)** 조용한 실패로 보인다. d_left =
(M1+M2)/2, d_right=(M3+M4)/2로 계산되므로(diff_drive_node.py 참고), 어느
채널이 고착됐는지 원시값을 직접 봐야 확정할 수 있다.

사용법:
    python3 tools/loop_gyro_diag.py --out docs/calibration/loop_gyro_diag.csv
"""
import argparse
import csv
import math
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

try:
    import smbus2
except ImportError:
    smbus2 = None

BATTERY_I2C_ADDR = 0x40
BATTERY_READ_INTERVAL_S = 0.2  # 5Hz — motor_dashboard.py와 별개로 여기 전용


def read_battery_voltage(bus):
    raw = bus.read_word_data(BATTERY_I2C_ADDR, 0x02)
    val = ((raw & 0xff) << 8) | (raw >> 8)
    if val & 0x1:
        raise IOError("INA219 OVF")
    return (val >> 3) * 0.004


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('loop_gyro_diag')
    state = {'gyro_z': None, 'ekf_yaw': None, 'odom_yaw': None,
             'ekf_x': None, 'ekf_y': None, 'odom_x': None, 'odom_y': None,
             'battery_v': None, 'enc': [None, None, None, None]}

    f = open(args.out, 'w', newline='')
    writer = csv.DictWriter(f, fieldnames=[
        't', 'gyro_z', 'ekf_x', 'ekf_y', 'ekf_yaw_deg',
        'odom_x', 'odom_y', 'odom_yaw_deg', 'battery_v',
        'enc_m1', 'enc_m2', 'enc_m3', 'enc_m4'])
    writer.writeheader()

    def on_imu(msg):
        state['gyro_z'] = msg.angular_velocity.z
        e = state['enc']
        writer.writerow({
            't': time.time(),
            'gyro_z': state['gyro_z'],
            'ekf_x': state['ekf_x'], 'ekf_y': state['ekf_y'],
            'ekf_yaw_deg': math.degrees(state['ekf_yaw']) if state['ekf_yaw'] is not None else '',
            'odom_x': state['odom_x'], 'odom_y': state['odom_y'],
            'odom_yaw_deg': math.degrees(state['odom_yaw']) if state['odom_yaw'] is not None else '',
            'battery_v': state['battery_v'],
            'enc_m1': e[0], 'enc_m2': e[1], 'enc_m3': e[2], 'enc_m4': e[3],
        })

    def on_ekf(msg):
        p = msg.pose.pose
        state['ekf_x'], state['ekf_y'] = p.position.x, p.position.y
        state['ekf_yaw'] = yaw_from_quat(p.orientation)

    def on_odom(msg):
        p = msg.pose.pose
        state['odom_x'], state['odom_y'] = p.position.x, p.position.y
        state['odom_yaw'] = yaw_from_quat(p.orientation)

    def on_enc(msg):
        state['enc'] = list(msg.data)

    n.create_subscription(Imu, '/imu/data', on_imu, 50)
    n.create_subscription(Odometry, '/odometry/filtered', on_ekf, 20)
    n.create_subscription(Odometry, '/odom', on_odom, 20)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', on_enc, 20)

    bus_holder = {'bus': None}
    if smbus2 is not None:
        def on_battery_timer():
            try:
                if bus_holder['bus'] is None:
                    bus_holder['bus'] = smbus2.SMBus(1)
                state['battery_v'] = read_battery_voltage(bus_holder['bus'])
            except Exception:
                bus_holder['bus'] = None  # 다음 틱에 재오픈
        n.create_timer(BATTERY_READ_INTERVAL_S, on_battery_timer)
    else:
        print('⚠️ smbus2 없음 — 배터리 전압 없이 기록(gyro/odom만)')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    print(f'로깅 시작 — 조종 끝나면 Ctrl+C (또는 SIGTERM). 저장: {args.out}')
    try:
        while not flag['v']:
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        f.close()
        n.destroy_node()
        rclpy.shutdown()
    print(f'저장 완료: {args.out}')


if __name__ == '__main__':
    main()
