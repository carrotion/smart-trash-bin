"""1층 기본 동작 — 정해진 거리만큼 곧게 가기(멈춤-확인-이동의 "이동" 부분, docs/motion_foundation.md).

거리 판단은 /odometry/filtered(EKF) 위치를 **출발 방향으로 투영**한 값. 방향 유지는 구동 노드 yaw_hold(w=0 명령)에 맡긴다.
속도: 가속 제한 accel → 순항 v_max → 남은 거리 × kp로 감속(최저 v_min) → 예상 관성 거리 안에 들면 정지(구동 노드 능동 제동).
방향: 출발 방향과의 오차를 작은 회전 명령(±head_w_max)으로 되돌림(구동 노드 yaw_hold는 각속도만 유지).
coast_s 0.17: 0.25면 첫 주행 평균 −1.6 cm(일찍 멈춤), 0.12면 +1.3 cm → 0.17(09-27, 능동 제동 뒤, 회전과 같은 값).
head_kp 3.0: 1.5에선 방향 오차 1.2~1.3°가 가끔, 3.0에서 대부분 0.1~0.6°(달리는 중 작은 회전은 2~5할만 먹힘).
정지 뒤 조용해지면 남은 거리를 다시 읽고, tol 넘게 모자라면 v_min으로 한 번 보정(넘친 경우는 되돌리지 않음 — 뒤 확인 없이 후진하지 않기).
안전: 전체 시간 제한, 1.5초 동안 2 cm도 못 가면 중단(구동 이상), 호출자가 준 clear_fn()이 False면 즉시 정지.

    dd = DriveDistance(pub, spin, get_pose, get_speed, clear_fn=...)
    r = dd.drive_by(0.5)     # r: ok, msg, err_m(+ = 넘침), fixes, secs
"""
import math
import time

from geometry_msgs.msg import Twist


class DriveDistance:
    """get_pose(): (x, y, yaw) EKF, get_speed(): 현재 선속도(m/s, EKF), spin(t), clear_fn(sign)->bool(가는 쪽이 비었나)."""

    def __init__(self, pub, spin, get_pose, get_speed, clear_fn=None, v_max=0.15, v_min=0.06, accel=0.3,
                 kp=1.5, coast_s=0.17, tol_m=0.02, max_fix=1, settle_speed=0.01, settle_s=0.3,
                 head_kp=3.0, head_w_max=0.2):
        self.pub, self.spin, self.get_pose, self.get_speed = pub, spin, get_pose, get_speed
        self.clear_fn = clear_fn or (lambda sgn: True)
        self.v_max, self.v_min, self.accel, self.kp, self.coast_s = v_max, v_min, accel, kp, coast_s
        self.tol, self.max_fix, self.settle_speed, self.settle_s = tol_m, max_fix, settle_speed, settle_s
        self.head_kp, self.head_w_max = head_kp, head_w_max
        self._v = 0.0; self._t = None; self._w = 0.0

    def _cmd(self, v, w=0.0):
        now = time.time()
        if self.accel > 0 and v != 0.0 and abs(v) > abs(self._v) and (self._v == 0.0 or (v > 0) == (self._v > 0)):
            dt = min(0.1, now - self._t) if self._t else 0.02
            v = math.copysign(min(abs(v), abs(self._v) + self.accel * dt), v)
        self._t = now
        c = Twist(); c.linear.x = float(v); c.angular.z = float(w); self.pub.publish(c); self._v = float(v)

    def stop(self, n=5):
        for _ in range(n):
            self._cmd(0.0); self.spin(0.03)

    def _progress(self, start):
        """출발 자세 기준 앞으로 간 거리(m, 출발 방향 투영)와 옆 거리."""
        x0, y0, th0 = start; x, y, _ = self.get_pose()
        dx, dy = x - x0, y - y0
        return dx * math.cos(th0) + dy * math.sin(th0), -dx * math.sin(th0) + dy * math.cos(th0)

    def _settle(self, limit=2.0):
        t0 = time.time(); calm = None
        while time.time() - t0 < limit:
            self.spin(0.02)
            if abs(self.get_speed()) < self.settle_speed:
                calm = calm or time.time()
                if time.time() - calm >= self.settle_s:
                    return True
            else:
                calm = None
        return False

    def _drive(self, start, target, v_cap, time_limit):
        """출발 자세 start 기준 target(m, 부호=방향) 지점까지. (ok, msg)"""
        t0 = time.time(); sgn = 1.0 if target > 0 else -1.0; last_prog = None; t_prog = t0; t_block = None
        while True:
            along, _ = self._progress(start)
            rem = target - along
            v_now = abs(self.get_speed())
            if rem * sgn < max(v_now * self.coast_s, 0.003):
                break
            # 막힘이 0.15초(스캔 두 장) 넘게 이어질 때만 정지 — ⚠️ 09-27: 차체 앞 모서리 바로 바깥 자기반사 점이
            #   스캔 한 장에만 찍혀 0.03 m 만에 멈춘 일(주행기·회귀 시험 각 1회). 판정을 1층 동작 안에 둔다
            if not self.clear_fn(sgn):
                t_block = t_block or time.time()
                if time.time() - t_block > 0.15:
                    self.stop(); return False, '진행 방향 장애물 — 정지'
            else:
                t_block = None
            v = sgn * max(self.v_min, min(v_cap, self.kp * abs(rem)))
            # 출발 방향 유지: 방향 오차 × head_kp를 작은 회전 명령으로(±head_w_max). ⚠️ 09-27 기준 측정에서 0.3 m마다
            #   출발 때 1~2° 틀어진 채 남음 — 구동 노드 yaw_hold는 각속도만 잡아 이미 틀어진 각은 되돌리지 않는다
            herr = (self.get_pose()[2] - start[2] + math.pi) % (2 * math.pi) - math.pi
            w = max(-self.head_w_max, min(self.head_w_max, -self.head_kp * herr))
            self._cmd(v, w); self.spin(0.02)
            now = time.time()
            if last_prog is None or abs(along - last_prog) > 0.02:
                last_prog, t_prog = along, now
            elif now - t_prog > 1.5:
                self.stop(); return False, '1.5초 동안 2 cm도 못 감 — 구동 이상'
            if now - t0 > time_limit:
                self.stop(); return False, f'시간 초과 {time_limit:.1f}초(남은 {rem * 100:+.1f} cm)'
        self.stop()
        return True, ''

    def drive_by(self, dist_m):
        t0 = time.time(); start = self.get_pose()
        limit = abs(dist_m) / self.v_min + 3.0
        ok, msg = self._drive(start, dist_m, self.v_max, limit)
        fixes = 0; main_err = None
        while ok:
            self._settle()
            along, _ = self._progress(start)
            err = along - dist_m                      # + = 넘침(부호는 진행 방향 기준으로 아래서 정리)
            if main_err is None:
                main_err = err * (1 if dist_m > 0 else -1)
            short = (dist_m - along) * (1 if dist_m > 0 else -1)   # + = 모자람
            if short <= self.tol or fixes >= self.max_fix:
                break
            fixes += 1
            ok, msg = self._drive(start, dist_m, self.v_min, short / self.v_min + 3.0)
        along, lat = self._progress(start)
        err = (along - dist_m) * (1 if dist_m > 0 else -1)
        return dict(ok=ok, msg=msg, err_m=err, lat_m=lat, fixes=fixes, secs=time.time() - t0, main_err_m=main_err)
