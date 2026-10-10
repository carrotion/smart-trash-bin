#!/usr/bin/env python3
"""하드웨어 작업 후 자동 점검 — docs/hardware_change_procedure.md 0장.

로봇을 움직이지 않는다(읽기만 한다). 전원을 켠 직후, 그리고 어떤 시험이든
시작하기 전에 돌린다. 하나라도 ❌면 다음 단계로 가지 말 것.

    python3 tools/hw_preflight.py
"""
import os, subprocess, sys, time

from wheel_config import ENCODER_SIGN, PWM_SIGN, COUNTS_PER_REV, WHEEL_RADIUS

SERVICES = [
    'smart-trash-diff-drive', 'smart-trash-ekf', 'smart-trash-imu', 'smart-trash-imu-tf',
    'smart-trash-lidar', 'smart-trash-motor-dashboard', 'smart-trash-power-monitor',
    'smart-trash-blackbox', 'smart-trash-disk-watchdog', 'smart-trash-map-autosave',
]
USB = {'1a86:7522': ('모터드라이버 CH340', '/dev/motor'),
       '10c4:ea60': ('라이다 CP2102', '/dev/lidar')}

fails = []
warns = []


def ok(msg, hint=''):
    print(f'  ✅ {msg}')


def bad(msg, hint=''):
    print(f'  ❌ {msg}' + (f'\n      -> {hint}' if hint else ''))
    fails.append(msg)


def warn(msg, hint=''):
    print(f'  ⚠️  {msg}' + (f'\n      -> {hint}' if hint else ''))
    warns.append(msg)


def check_usb():
    print('\n[USB 장치]')
    out = subprocess.run(['lsusb'], capture_output=True, text=True).stdout
    for vid, (name, link) in USB.items():
        if vid not in out:
            bad(f'{name}({vid}) 미인식',
                '커넥터 이탈 의심 — 2026-09-24에 케이블 고정 작업 중 USB-C가 두 번 빠졌다')
        elif not os.path.exists(link):
            bad(f'{name} 인식됐는데 {link} 링크 없음',
                'udev 규칙 확인: tools/udev/99-smart-trash.rules')
        else:
            ok(f'{name} -> {link} ({os.path.realpath(link)})')


def check_services():
    print('\n[상시 서비스]')
    for s in SERVICES:
        st = subprocess.run(['systemctl', 'is-active', s], capture_output=True,
                            text=True).stdout.strip()
        if st == 'active':
            ok(s)
        elif st == 'activating' and s == 'smart-trash-diff-drive':
            bad(f'{s}: activating에서 멈춤', '모터 포트(/dev/motor)를 못 여는 중일 가능성')
        else:
            bad(f'{s}: {st}')


def check_gyro_bias():
    """diff_drive_node가 이번 기동 때 잡은 자이로 바이어스를 로그에서 확인.

    노드는 뜨는 순간 정지 상태라고 가정하고 바이어스를 잰다. 2026-09-24에 USB-C를
    다시 꽂는 순간 노드가 떴고 사람이 로봇을 만지던 중이라 -0.438 rad/s(정상 약
    -0.005)가 잡혔다 → 회전 폐루프가 도크에선 오른쪽 바퀴를 역전시키고 바닥
    직진을 -51° 틀었다. 노드가 이제 스스로 거부하지만 한 번 더 본다.
    """
    print('\n[diff_drive 자이로 바이어스]')
    # 이번 기동 이후 로그만 본다(서비스 시작 시각 기준)
    since = subprocess.run(['systemctl', 'show', '-p', 'ActiveEnterTimestamp', '--value',
                            'smart-trash-diff-drive'], capture_output=True, text=True).stdout.strip()
    since = ' '.join(since.split()[1:3])  # "Thu 2026-09-24 21:05:53 KST" -> 날짜 시각
    out = subprocess.run(['journalctl', '-u', 'smart-trash-diff-drive', '--since', since,
                          '--no-pager', '-o', 'cat'], capture_output=True, text=True).stdout
    done = [l for l in out.splitlines() if '바이어스 보정 완료' in l]
    rejected = [l for l in out.splitlines() if '바이어스 측정 거부' in l]
    if not done:
        (warn if rejected else bad)(
            '이번 기동에서 바이어스 보정 완료 기록 없음' + (' (거부 반복 중)' if rejected else ''),
            '로봇을 가만히 두면 몇 초 안에 잡힌다. 계속 안 되면 IMU 확인. 그동안 회전 폐루프 꺼짐')
        return
    try:
        val = float(done[-1].split('완료:')[1].split('rad/s')[0])
    except (IndexError, ValueError):
        warn(f'바이어스 로그 해석 실패: {done[-1]}')
        return
    if abs(val) < 0.03:
        ok(f'{val:+.5f} rad/s')
    else:
        bad(f'{val:+.5f} rad/s — 정상(약 -0.005)과 동떨어짐',
            '로봇을 가만히 두고 sudo systemctl restart smart-trash-diff-drive')


def check_ros():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, MagneticField, LaserScan
    from std_msgs.msg import Int32MultiArray, String
    from tf2_ros import Buffer, TransformListener
    from rcl_interfaces.srv import GetParameters

    rclpy.init()
    n = Node('hw_preflight')
    # ⚠️ TF 리스너는 센서 수집이 끝난 뒤에 만든다. 한 노드에서 /tf(고빈도)까지
    # 같이 받으면 단일 스레드 spin이 못 따라가 작은 큐의 메시지가 버려지고,
    # 16.5Hz로 멀쩡히 나오는 /wheel_encoders를 3.5Hz로 오판했다(2026-09-24).
    samples = {'imu': [], 'mag': [], 'scan': 0, 'enc': 0, 'tier': None}
    n.create_subscription(Imu, '/imu/data', lambda m: samples['imu'].append(
        (m.angular_velocity.z, m.linear_acceleration.z)), 100)
    n.create_subscription(MagneticField, '/imu/mag', lambda m: samples['mag'].append(
        (m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z)), 100)
    n.create_subscription(LaserScan, '/scan', lambda m: samples.update(
        scan=samples['scan'] + 1), 50)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: samples.update(
        enc=samples['enc'] + 1), 50)
    n.create_subscription(String, '/battery_tier', lambda m: samples.update(tier=m.data), 5)

    t_end = time.time() + 4.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    print('\n[센서 — 주기가 아니라 값이 변하는지]')
    imu = samples['imu']
    if len(imu) < 20:
        bad(f'/imu/data 수신 부족({len(imu)}개/4초)')
    elif len(set(imu)) <= 1:
        bad('/imu/data 값이 전혀 안 변함 — 센서가 멈춘 값을 발행 중')
    else:
        ok(f'/imu/data {len(imu)}개, 값 변화 있음 (가속도 z 평균 '
           f'{sum(a for _, a in imu) / len(imu):.2f} m/s²)')
    mag = samples['mag']
    if len(mag) < 10:
        warn(f'/imu/mag 수신 부족({len(mag)}개)')
    elif len(set(mag)) <= 1:
        # 경고로 둔다: 자기센서는 EKF에 안 들어가 주행/오도메트리와 무관(나침반만 영향).
        # 멈춤 감지기가 "회전 중"에만 검사해서 정지 상태 멈춤을 못 잡는다(2026-09-24 발견).
        warn(f'/imu/mag {len(mag)}개가 전부 같은 값 — 센서 멈춤 의심',
             '살아있으면 정지 상태에서도 LSB 노이즈가 있다(08-17). 주행엔 무관, 나침반만 영향')
    else:
        ok(f'/imu/mag {len(mag)}개, 값 변화 있음')
    (ok if samples['scan'] >= 20 else bad)(f'/scan {samples["scan"]}개/4초 (기대 ~40)')
    # 실제 발행은 ~16Hz지만 이 도구가 단일 스레드라 일부를 놓친다 — 살아서 흐르는지만 본다
    (ok if samples['enc'] >= 30 else bad)(f'/wheel_encoders {samples["enc"]}개/4초 (흐름 확인용)')
    if samples['tier'] is None:
        bad('/battery_tier 미수신', 'motor-dashboard 서비스/INA219 확인')
    elif samples['tier'] == 'ok':
        ok('배터리 등급 ok')
    else:
        warn(f'배터리 등급 {samples["tier"]}', '측정 전 충전 권장')

    print('\n[TF]')
    tf = Buffer()
    TransformListener(tf, n)
    t_end = time.time() + 2.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    try:
        t = tf.lookup_transform('base_link', 'imu_link', rclpy.time.Time())
        v = t.transform.translation
        ok(f'base_link->imu_link ({v.x:.3f}, {v.y:.3f}, {v.z:.3f})')
    except Exception:
        bad('base_link->imu_link 없음',
            '없으면 EKF가 IMU를 통째로 버린다(2026-08-29) — smart-trash-imu-tf 서비스')

    print('\n[diff_drive_node 파라미터 vs tools/wheel_config.py]')
    cli = n.create_client(GetParameters, '/diff_drive_node/get_parameters')
    if not cli.wait_for_service(timeout_sec=3.0):
        bad('diff_drive_node 파라미터 서비스 응답 없음')
    else:
        names = ['encoder_signs', 'pwm_signs', 'counts_per_rev', 'counts_per_rev_rear',
                 'wheel_radius', 'motor_pwm_offset_m1', 'motor_pwm_offset_m2',
                 'motor_pwm_offset_m3', 'motor_pwm_offset_m4']
        req = GetParameters.Request(names=names)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
        vals = {}
        for name, v in zip(names, fut.result().values if fut.result() else []):
            vals[name] = (list(v.integer_array_value) or list(v.double_array_value)
                          or v.double_value or v.integer_value)
        expect = {
            'encoder_signs': ENCODER_SIGN, 'pwm_signs': PWM_SIGN,
            'counts_per_rev': COUNTS_PER_REV[0], 'counts_per_rev_rear': COUNTS_PER_REV[2],
            'wheel_radius': WHEEL_RADIUS,
        }
        for k, e in expect.items():
            got = vals.get(k)
            same = (list(got) == list(e)) if isinstance(e, list) else (
                got is not None and abs(float(got) - float(e)) < 1e-6)
            (ok if same else bad)(f'{k}: 노드 {got} / wheel_config {e}',
                                  '' if same else '둘 중 하나가 낡았다 — 같은 커밋에서 맞출 것')
        offs = [vals.get(f'motor_pwm_offset_m{i}') for i in (1, 2, 3, 4)]
        if any(o for o in offs):
            warn(f'motor_pwm_offset = {offs}',
                 '현재 채널 매핑 기준으로 잰 값인지 확인(매핑이 바뀌면 무효)')
        else:
            ok('motor_pwm_offset 전부 0 (재캘리브레이션 전 상태)')

    print('\n[/cmd_vel 발행자]')
    pubs = [i.node_name for i in n.get_publishers_info_by_topic('/cmd_vel')]
    if pubs:
        # 대시보드는 버튼을 누를 때만 발행한다(대기 중 0개 실측, 09-24) — 참고용 경고
        warn(f'{pubs}', '시험 중 아무도 조종하지 않으면 무해. 확실히 하려면: sudo systemctl stop '
             'smart-trash-motor-dashboard smart-trash-map-dashboard')
    else:
        ok('없음 — 측정 가능한 상태')

    n.destroy_node()
    rclpy.shutdown()


def check_lidar_blockage():
    print('[라이다 가림]')
    import lidar_blockage as lb
    topics = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout.split()
    scans = lb.collect_live('/scan_raw' if '/scan_raw' in topics else '/scan', 4.0)
    good, msg = lb.check(scans)
    (ok if good else bad)(msg, '' if good else '10-09: 종이테이프 2곳이 붙은 채 합격 시험을 함 — tools/lidar_blockage.py')


def main():
    print('하드웨어 점검 (로봇을 움직이지 않음)')
    check_usb()
    check_services()
    check_gyro_bias()
    try:
        check_ros()
    except Exception as e:
        bad(f'ROS 점검 중 예외: {e}')
    try:
        check_lidar_blockage()
    except Exception as e:
        bad(f'라이다 가림 점검 중 예외: {e}')
    print(f'\n=== 결과: ❌ {len(fails)}개, ⚠️ {len(warns)}개 ===')
    if fails:
        print('❌가 있으면 시험/주행으로 넘어가지 말 것.')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
