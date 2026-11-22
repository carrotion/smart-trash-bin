#!/usr/bin/env python3
"""라이다 데이터 끊김 감시 — 끊기면 라이다 드라이버만 다시 연다 (2026-11-22, todo #13).

왜: 11번째 탐색(11-22 16:20)에서 바닥의 슬리퍼·옷과 부딪힌 충격 1초 뒤 LD06 드라이버가 "get ldlidar data is time out"만 내고
데이터를 멈췄다. USB는 붙은 채였고, 드라이버는 죽지 않고 계속 시간 초과만 찍어 런치의 respawn도 소용없었다. 충돌 감시가 로봇을 세우고
지도 갱신이 멈춰 탐색이 끝났다. 드라이버를 다시 열자 곧바로 회복했다 → 그 일을 자동으로 한다.

하는 일: /scan_raw(드라이버 출력)가 stall_s초 넘게 안 오면 드라이버 프로세스(실행파일 ldlidar_stl_ros2_node)를 끝낸다(TERM, 2초 뒤 남으면 KILL).
런치가 respawn으로 다시 띄운다(mapping.launch.py). 다시 띄운 뒤 cooldown_s 동안은 또 끝내지 않는다(여는 데 걸리는 시간).
처음 스캔이 들어오기 전에는 아무것도 하지 않는다(기동 중). 서비스(smart-trash-lidar) 소속 드라이버는 건드리지 않는다.
로봇을 움직이지 않는다. 끊긴 동안의 정지는 충돌 감시(source_timeout 0.5 s)가 맡는다.

    python3 tools/lidar_watchdog.py [--stall 3.0] [--cooldown 15]        # 자율 스택 런치가 띄운다
"""
import argparse, os, signal, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import killproc

DRIVER = 'ldlidar_stl_ros2_node'


class Watchdog:
    """시간·프로세스 다루기를 밖에서 넣을 수 있게 — 시험(test_lidar_watchdog.py)은 ROS 없이 돌린다."""

    def __init__(self, stall_s=3.0, cooldown_s=15.0, clock=time.monotonic, find=None, kill=None, log=print):
        self.stall_s, self.cooldown_s, self.clock, self.log = stall_s, cooldown_s, clock, log
        self.find = find or (lambda: [p for p, _, _ in killproc.find([DRIVER])])
        self.kill = kill or self._kill
        self.last_scan = None; self.last_restart = -1e9; self.restarts = 0

    def on_scan(self):
        if self.last_scan is not None and self.clock() - self.last_scan > self.stall_s:
            self.log(f'라이다 스캔 다시 들어옴({self.clock() - self.last_scan:.1f}초 끊긴 뒤)')
        self.last_scan = self.clock()

    def _kill(self, pids):
        for p in pids:
            try:
                os.kill(p, signal.SIGTERM)
            except ProcessLookupError:
                pass
        end = time.monotonic() + 2.0
        while time.monotonic() < end and any(os.path.exists(f'/proc/{p}') for p in pids):
            time.sleep(0.1)
        for p in pids:
            if os.path.exists(f'/proc/{p}'):
                try:
                    os.kill(p, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def tick(self):
        """주기마다 부른다. 드라이버를 끝냈으면 True."""
        now = self.clock()
        if self.last_scan is None or now - self.last_scan <= self.stall_s or now - self.last_restart < self.cooldown_s:
            return False
        pids = self.find()
        if not pids:
            return False
        self.log(f'라이다 스캔이 {now - self.last_scan:.1f}초 끊김 → 드라이버 다시 열기(pid {pids})')
        self.kill(pids); self.last_restart = now; self.restarts += 1
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stall', type=float, default=3.0); ap.add_argument('--cooldown', type=float, default=15.0)
    ap.add_argument('--topic', default='/scan_raw')
    a = ap.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n = rclpy.create_node('lidar_watchdog')
    w = Watchdog(a.stall, a.cooldown, log=lambda s: print(f'[lidar_watchdog] {s}', flush=True))
    n.create_subscription(LaserScan, a.topic, lambda m: w.on_scan(), qos_profile_sensor_data)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1)); signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    print(f'[lidar_watchdog] {a.topic} 감시 시작(끊김 {a.stall}초, 다시 열고 {a.cooldown}초 쉼)', flush=True)
    try:
        while not stop:
            rclpy.spin_once(n, timeout_sec=0.5); w.tick()
    finally:
        print(f'[lidar_watchdog] 끝(다시 연 횟수 {w.restarts})', flush=True)
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
