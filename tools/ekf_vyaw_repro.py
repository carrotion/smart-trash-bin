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

⚠️⚠️ **1차 조사 결론(2026-08-29 밤)**: 아래 7개 시나리오 전부 PC에서는
**문제가 재현되지 않았다**(전부 명령값 근처로 정확히 수렴, 91.2~101.7%) —
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

⚠️⚠️⚠️ **진짜 근본원인 확정(Remi, 실물 debug 로그로 발견) + 이 도구로
독립 재검증 완료**: `base_link->imu_link` 정적 TF가 시스템에 아예 없었다
(`hardware_inventory.md`의 "vyaw만 쓰니까 위치 무관 → TF 불필요"는 물리적
추론은 맞지만 틀렸다 — `robot_localization`은 어떤 필드를 쓰든 메시지
프레임→타깃 프레임 변환을 항상 먼저 시도하고 실패하면 **메시지 전체를
버린다**, `Could not transform measurement into base_link. Ignoring...`).
위 7개 시나리오가 전부 `frame_id=base_link`로 가짜 IMU를 발행해서 애초에
TF 변환이 필요 없었던 게 재현 실패의 진짜 이유였다:
  8. missing_imu_tf    (frame_id=imu_link, TF 미발행) → **재현 성공**
                        (융합 실패, imu0 processMeasurement 사실상 무효)
  9. imu_tf_fixed      (frame_id=imu_link, base_link->imu_link 정적 TF 발행)
                        → 정상 융합으로 회복

**교훈**: 필드가 물리적으로 위치 무관이어도, 미들웨어(robot_localization)는
메시지 단위로 먼저 걸러낼 수 있다 — "이 값은 장착 위치와 무관하니 TF가
필요 없다"는 추론만으로 TF 생략을 단정하면 안 된다.

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
#  force_unsettled_cov, duration, imu_frame_id, publish_static_tf)
SCENARIOS = {
    'minimal_imu_only': (IMU0_ONLY_YAML, True, 1.0, 0.0, False, 0.0, 0.0, 0.0, 15,
                          'base_link', False),
    'full_with_odom': (WITH_ODOM_YAML, True, 1.0, 0.0, True, 0.0, 0.0, 0.0, 15,
                        'base_link', False),
    'noisy': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.0, 0.0, 0.0, 15,
              'base_link', False),
    'vx_injected': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.03, 0.02, 0.0, 15,
                     'base_link', False),
    'long_duration': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.03, 0.02, 0.0, 65,
                       'base_link', False),
    'forced_unsettled': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.0, 0.0, 50.0, 20,
                          'base_link', False),
    'two_d_mode_false': (WITH_ODOM_YAML, False, 1.0, 0.25, True, 0.03, 0.02, 0.0, 15,
                          'base_link', False),
    # 2026-08-29 밤 추가 — Remi가 실물에서 확정한 진짜 근본원인 재현/검증.
    # base_link->imu_link 정적 TF가 없으면 robot_localization이 "Could not
    # transform measurement into base_link. Ignoring..."로 IMU 메시지를
    # 전부 버린다(imu0 관련 processMeasurement 호출 자체가 0회였음, debug
    # 로그로 확인됨). 위 7개 시나리오가 전부 frame_id=base_link로 발행해서
    # TF 변환이 애초에 필요 없었던 게 재현 실패의 진짜 이유였다 — 이 두
    # 시나리오로 그 사실을 독립적으로 재확인한다.
    'missing_imu_tf': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.03, 0.02, 0.0, 15,
                        'imu_link', False),  # TF 없음 -> 재현 기대(융합 실패)
    'imu_tf_fixed': (WITH_ODOM_YAML, True, 1.0, 0.25, True, 0.03, 0.02, 0.0, 15,
                      'imu_link', True),  # 정적 TF 발행 -> 정상 융합 기대
}


def make_yaml(template, two_d_mode, tag):
    path = os.path.join(SCRATCH, f'ekf_{tag}.yaml')
    with open(path, 'w') as f:
        f.write(template.format(two_d_mode=str(two_d_mode).lower()))
    return path


def publish_fake_imu(stop_flag, wz, noise_std, force_unsettled_cov, rate=50.0,
                      frame_id='base_link'):
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
        msg.header.frame_id = frame_id
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
     force_unsettled, default_duration, imu_frame_id, publish_static_tf) = SCENARIOS[name]
    duration = duration_override or default_duration
    yaml_path = make_yaml(template, two_d_mode, name)

    setup = f'source /opt/ros/jazzy/setup.bash && '
    ekf_log = os.path.join(SCRATCH, f'{name}_ekf.log')
    with open(ekf_log, 'w') as logf:
        ekf_proc = subprocess.Popen(
            ['bash', '-c', f'{setup}exec ros2 run robot_localization ekf_node '
                            f'--ros-args --params-file {yaml_path}'],
            stdout=logf, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    tf_proc = None
    if publish_static_tf:
        # Remi가 실물에서 확정한 실제 장착 기하(hardware_inventory.md)와 동일—
        # x=+0.105 y=0 z=0.090, 회전 없음.
        tf_proc = subprocess.Popen(
            ['bash', '-c', f'{setup}exec ros2 run tf2_ros static_transform_publisher '
                            f'0.105 0 0.090 0 0 0 base_link {imu_frame_id}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    # 자식 프로세스로 publisher들을 fork해서 부모 인터프리터의 rclpy 컨텍스트와
    # 절대 안 섞이게 한다(한 프로세스 안에 rclpy.init()을 여러 번/여러 노드로
    # 걸면 컨텍스트 충돌 위험 — optuna_search.py 등 다른 도구도 항상 서브프로세스로
    # 격리하는 패턴을 따름).
    import multiprocessing as mp

    def imu_worker():
        flag = {'v': False}
        publish_fake_imu(flag, wz, imu_noise, force_unsettled, frame_id=imu_frame_id)

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
    if tf_proc is not None:
        try:
            os.killpg(os.getpgid(tf_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        tf_proc.wait(timeout=10)

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
                  f'odom={cfg[4]} duration={cfg[8]}s imu_frame={cfg[9]} '
                  f'static_tf={cfg[10]}')
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
