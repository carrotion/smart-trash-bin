#!/usr/bin/env python3
"""EKF vyaw 융합 손실 재현 실험 하네스 (pc_tasks.md task 12, 2026-08-29 밤).

배경: 10바퀴(3600°) 정밀 회전 시험에서 원시 자이로 적분은 99.29% 정확한데
`/odometry/filtered`(EKF 융합)는 22.18%(SLAM 켠 채)/37.03%(SLAM 끈 채)만
반영함(docs/midterm_review_20260829.md 7절). `process_noise_covariance`의
vyaw를 1.5로 올리는 시도는 이미 해봤고 **오히려 악화**됐음 — 반복 금지.

이 도구는 실물 로봇 없이 가짜 /imu/data(+선택적 가짜 /odom)를 발행하고
`ekf_node`를 실제와 동일한 파라미터로 띄운 뒤 `/odometry/filtered`의
angular_z 수렴 여부를 관찰한다 — robot_localization의 EKF 융합 메커니즘
자체에 문제가 있는지, 아니면 실물 환경 특유의 요인인지 격리하기 위함.

⚠️⚠️ **결론(2026-08-29 밤 조사 완료)**: 아래 7개 시나리오 전부 PC에서는
**문제가 재현되지 않았다**(전부 명령값 근처로 정확히 수렴, ±15% 이내) —
  1. minimal_imu_only  (odom0 없이 imu0만, 최소 구성)
  2. full_with_odom    (실제 ekf.yaml과 동일, odom0+imu0)
  3. noisy             (자이로에 노이즈 주입, 실측 요동 폭 흉내)
  4. vx_injected       (odom0에 0이 아닌 vx 주입 — 스키드-스티어 회전 중
                         좌우 바퀴 비대칭으로 vx가 실제로 섞일 수 있다는 가설)
  5. long_duration     (65초 관찰 — 시간에 따라 서서히 나빠지는지 확인)
  6. forced_unsettled  (imu_publisher_node의 gyro_unsettled_covariance(50.0)
                         게이팅이 시험 내내 발동 중이라는 가설 — angular_velocity_
                         covariance[8]을 50.0으로 강제 고정해도 여전히 정확히 수렴)
  7. two_d_mode_false  (two_d_mode를 명시적으로 꺼도 동일)

**이게 의미하는 것**: robot_localization의 표준 EKF 융합 메커니즘 자체와
우리 ekf.yaml 설정(imu0_config 인덱스 포함)은 정상이다. 문제는 파라미터가
아니라 **PC 재현으로는 못 잡는 실물 특유의 요인**일 가능성이 높다. 다음
후보(미검증, 실물에서만 확인 가능):
  - diff_drive_node가 시작 시 자체적으로 자이로 바이어스를 재보정하는데
    (imu_publisher_node와 별개 경로로 IMU에 접근하는 것으로 보임), 회전
    폐루프가 활발히 도는 동안 두 프로세스가 I2C를 동시에 건드리며
    imu_publisher_node의 자이로 발행 타이밍/값이 실제로 왜곡될 가능성.
  - `gyro_scale_test.py`(회전 시험 스크립트, 이 세션엔 없어 리뷰 못 함)의
    /cmd_vel 발행 패턴이 imu_publisher_node의 `_on_cmd_vel` eps(1e-3) 전환
    판정을 예상 밖으로 자주 트리거하고 있을 가능성 — 실측 필요.
  - 실물(ARM, 실제 DDS/QoS 타이밍)과 PC(x86) 사이의 메시지 드롭/지연 차이.
  → **다음 최선의 진단**: 실물에서 회전 시험 중
    `ros2 topic echo /imu/data --field angular_velocity_covariance`로
    covariance[8]이 실제로 0.0004에 머무는지 자주 50.0으로 튀는지 직접
    관찰(5분이면 됨, PC 재현보다 훨씬 저비용 확증 수단).

사용법:
    python3 tools/ekf_vyaw_repro.py --list-scenarios
    python3 tools/ekf_vyaw_repro.py --scenario full_with_odom
    python3 tools/ekf_vyaw_repro.py --scenario forced_unsettled --duration 20
"""
import argparse
import csv
import os
import random
import signal
import subprocess
import sys
import time

REPO = os.path.expanduser('~/smart-trash-bin')
SCRATCH = '/tmp/ekf_vyaw_repro'
os.makedirs(SCRATCH, exist_ok=True)

IMU0_ONLY_YAML = '''ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: {two_d_mode}
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    publish_tf: false
    imu0: imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 20
    imu0_remove_gravitational_acceleration: true
'''

WITH_ODOM_YAML = '''ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: {two_d_mode}
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    publish_tf: false
    odom0: odom
    odom0_config: [true,  true,  false,
                   false, false, false,
                   true,  false, false,
                   false, false, false,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
    imu0: imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 20
    imu0_remove_gravitational_acceleration: true
'''

# (yaml_template, two_d_mode, wz, imu_noise_std, use_odom, odom_vx, odom_noise,
#  force_unsettled_cov, duration)
SCENARIOS = {
    'minimal_imu_only': (IMU0_ONLY_YAML, True, 1.0, 0.0, False, 0.0, 0.0, 0.0, 15),
    'full_with_odom': (WITH_ODOM_YAML, True, 1.0, 0.0, True, 0.0, 0.0, 0.0, 15),
    'noisy': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.0, 0.0, 0.0, 15),
    'vx_injected': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.03, 0.02, 0.0, 15),
    'long_duration': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.03, 0.02, 0.0, 65),
    'forced_unsettled': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.0, 0.0, 50.0, 20),
    'two_d_mode_false': (WITH_ODOM_YAML, False, 1.0, 0.25, True, 0.03, 0.02, 0.0, 15),
}


def make_yaml(template, two_d_mode, tag):
    path = os.path.join(SCRATCH, f'ekf_{tag}.yaml')
    with open(path, 'w') as f:
        f.write(template.format(two_d_mode=str(two_d_mode).lower()))
    return path


def publish_fake_imu(stop_flag, wz, noise_std, force_unsettled_cov, rate=50.0):
    import rclpy
    from sensor_msgs.msg import Imu
    rclpy.init(args=[])
    n = rclpy.create_node('fake_imu_pub')
    pub = n.create_publisher(Imu, '/imu/data', 20)

    def tick():
        if stop_flag['v']:
            return
        msg = Imu()
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0
        wz_noisy = wz + (random.gauss(0.0, noise_std) if noise_std > 0 else 0.0)
        msg.angular_velocity.z = wz_noisy
        msg.angular_velocity_covariance[0] = 0.0004
        msg.angular_velocity_covariance[4] = 0.0004
        msg.angular_velocity_covariance[8] = (
            force_unsettled_cov if force_unsettled_cov > 0 else 0.0004)
        msg.linear_acceleration.z = 9.81
        msg.linear_acceleration_covariance[0] = 0.02
        msg.linear_acceleration_covariance[4] = 0.02
        msg.linear_acceleration_covariance[8] = 0.02
        pub.publish(msg)

    n.create_timer(1.0 / rate, tick)
    while not stop_flag['v']:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()


def publish_fake_odom(stop_flag, vx, noise_std, rate=30.0):
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init(args=[])
    n = rclpy.create_node('fake_odom_pub')
    pub = n.create_publisher(Odometry, '/odom', 10)
    state = {'x': 0.0, 't_prev': None}

    def tick():
        if stop_flag['v']:
            return
        now = n.get_clock().now()
        v = vx + (random.gauss(0.0, noise_std) if noise_std > 0 else 0.0)
        if state['t_prev'] is not None:
            dt = (now - state['t_prev']).nanoseconds / 1e9
            state['x'] += v * dt
        state['t_prev'] = now
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = state['x']
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = v
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 99999.0
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 99999.0
        pub.publish(msg)

    n.create_timer(1.0 / rate, tick)
    while not stop_flag['v']:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()


def run_scenario(name, duration_override=None):
    if name not in SCENARIOS:
        print(f'알 수 없는 시나리오: {name}. --list-scenarios로 확인.')
        return 1
    (template, two_d_mode, wz, imu_noise, use_odom, odom_vx, odom_noise,
     force_unsettled, default_duration) = SCENARIOS[name]
    duration = duration_override or default_duration
    yaml_path = make_yaml(template, two_d_mode, name)

    setup = f'source /opt/ros/jazzy/setup.bash && '
    ekf_log = os.path.join(SCRATCH, f'{name}_ekf.log')
    with open(ekf_log, 'w') as logf:
        ekf_proc = subprocess.Popen(
            ['bash', '-c', f'{setup}exec ros2 run robot_localization ekf_node '
                            f'--ros-args --params-file {yaml_path}'],
            stdout=logf, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    # 자식 프로세스로 publisher들을 fork해서 부모 인터프리터의 rclpy 컨텍스트와
    # 절대 안 섞이게 한다(한 프로세스 안에 rclpy.init()을 여러 번/여러 노드로
    # 걸면 컨텍스트 충돌 위험 — optuna_search.py 등 다른 도구도 항상 서브프로세스로
    # 격리하는 패턴을 따름).
    import multiprocessing as mp
    stop_imu = mp.Value('b', False)
    stop_odom = mp.Value('b', False)

    def imu_worker():
        flag = {'v': False}
        publish_fake_imu(flag, wz, imu_noise, force_unsettled)

    def odom_worker():
        flag = {'v': False}
        publish_fake_odom(flag, odom_vx, odom_noise)

    procs = []
    p_imu = mp.Process(target=imu_worker)
    p_imu.start()
    procs.append(p_imu)
    if use_odom:
        p_odom = mp.Process(target=odom_worker)
        p_odom.start()
        procs.append(p_odom)

    time.sleep(3)

    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init(args=[])
    obs = rclpy.create_node('ekf_observe')
    samples = []
    t0 = time.time()
    obs.create_subscription(Odometry, '/odometry/filtered',
                             lambda m: samples.append((time.time() - t0,
                                                        m.twist.twist.angular.z)), 20)
    deadline = time.time() + duration
    while time.time() < deadline:
        rclpy.spin_once(obs, timeout_sec=0.05)
    obs.destroy_node()
    rclpy.shutdown()

    # SIGTERM(terminate())을 주면 자식 프로세스 안의 rclpy executor가 종료
    # 신호를 못 삼키고 ExternalShutdownException 트레이스백을 그대로 뱉는다
    # (실측 확인 — 결과 자체엔 영향 없지만 사용자가 에러로 오인하기 쉬움).
    # 관찰이 끝난 뒤라 정상 종료 절차가 필요 없으므로 바로 SIGKILL한다.
    for p in procs:
        p.kill()
        p.join(timeout=5)
    try:
        os.killpg(os.getpgid(ekf_proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    ekf_proc.wait(timeout=10)

    if not samples:
        print(f'[{name}] 샘플 없음 — ekf_node가 안 떴을 수 있음, 로그 확인: {ekf_log}')
        return 1

    vals = [v for _, v in samples]
    last10 = vals[-10:]
    avg = sum(last10) / len(last10)
    print(f'[{name}] N={len(vals)} target_wz={wz} last_avg={avg:.4f} '
          f'min={min(vals):.4f} max={max(vals):.4f} '
          f'ratio={avg/wz*100:.1f}%')

    series_path = os.path.join(SCRATCH, f'{name}_series.csv')
    with open(series_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'angular_z'])
        w.writerows(samples)
    print(f'  시계열: {series_path}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', help='SCENARIOS 중 하나')
    ap.add_argument('--duration', type=float, default=None,
                     help='관찰 시간(초) 오버라이드')
    ap.add_argument('--list-scenarios', action='store_true')
    ap.add_argument('--all', action='store_true', help='전체 시나리오 순차 실행')
    args = ap.parse_args()

    if args.list_scenarios:
        for name, cfg in SCENARIOS.items():
            print(f'  {name:20s} two_d_mode={cfg[1]} wz={cfg[2]} imu_noise={cfg[3]} '
                  f'odom={cfg[4]} duration={cfg[8]}s')
        return 0

    if args.all:
        # ⚠️ 각 시나리오를 같은 파이썬 프로세스 안에서 연달아 돌리면(이전
        # 버전에서 실측) 첫 번째 이후로 ekf_node가 계속 못 뜬다 — 아마
        # multiprocessing.Process가 반복적으로 fork/rclpy.init 하면서
        # DDS 세션 상태가 오염되는 것으로 추정(정확한 근본원인은 미확정).
        # 서브프로세스로 완전히 격리하면 매번 새 인터프리터라 안전하다.
        rc = 0
        for name in SCENARIOS:
            cmd = [sys.executable, __file__, '--scenario', name]
            if args.duration:
                cmd += ['--duration', str(args.duration)]
            r = subprocess.run(cmd)
            rc |= r.returncode
        return rc

    if not args.scenario:
        ap.error('--scenario 또는 --list-scenarios 또는 --all 중 하나 필요')
    return run_scenario(args.scenario, args.duration)


if __name__ == '__main__':
    sys.exit(main())
