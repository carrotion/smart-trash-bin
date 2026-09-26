"""EKF 방향 기준 폐루프 제자리 회전 — 목표 각도까지 감속하며 돌고, 멈춘 뒤 다시 재서 보정한다.

왜 필요한가 (2026-09-26):
- 구동 노드는 회전 명령이 0이 되면 출력을 즉시 끊는다 → 멈추는 순간 속도만큼 관성으로 더 돈다.
  그 양은 속도·바닥·배터리에 따라 달라서, 예전 도구처럼 "w×0.12초 앞당겨 멈춤"으로 고정 추정하면
  90°마다 약 2~3° 틀렸다(정사각형 시험 방향 오차 ±7~8°의 대부분).
- 판단 기준을 /odometry/filtered(EKF) 방향으로 두면 로봇이 "믿는" 방향과 실제 돌기가 일치한다.
  EKF 방향 = 자이로 적분이고, 정사각형 시험에서 EKF 방향 추정 오차는 1~4°였다.

방법:
  1) 남은 각 rem에 비례해 감속: |w| = clamp(kp·|rem|, w_min, w_max)
  2) |rem| < 예상 관성 각(|측정 w| × coast_s) 이면 정지 명령
  3) 자이로가 가라앉을 때까지 기다린 뒤 rem을 다시 읽고, |rem| > tol이면 w_min으로 짧게 추가 회전
     (넘쳤으면 반대로). 최대 max_fix번
안전: 전체 시간 제한, 회전 명령 1.5초 뒤에도 자이로 적분이 5° 미만이면 중단(09-26 IMU 재시작 사고).
"""
import math
import time

from geometry_msgs.msg import Twist


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TurnToAngle:
    """get_yaw(): EKF 방향(rad), get_gyro(): 최신 자이로 z(rad/s), spin(t): t초 동안 콜백 처리."""

    def __init__(self, pub, spin, get_yaw, get_gyro, kp=1.5, w_min=0.2, w_max=0.5,
                 coast_s=0.10, tol_deg=1.0, max_fix=3, settle_gyro=0.03, settle_s=0.3):
        self.pub, self.spin, self.get_yaw, self.get_gyro = pub, spin, get_yaw, get_gyro
        self.kp, self.w_min, self.w_max, self.coast_s = kp, w_min, w_max, coast_s
        self.tol, self.max_fix = math.radians(tol_deg), max_fix
        self.settle_gyro, self.settle_s = settle_gyro, settle_s

    def _cmd(self, w):
        c = Twist(); c.angular.z = float(w); self.pub.publish(c)

    def stop(self, n=5):
        for _ in range(n):
            self._cmd(0.0); self.spin(0.03)

    def _settle(self, limit=2.0):
        t0 = time.time(); calm = None
        while time.time() - t0 < limit:
            self.spin(0.02)
            if abs(self.get_gyro()) < self.settle_gyro:
                calm = calm or time.time()
                if time.time() - calm >= self.settle_s:
                    return True
            else:
                calm = None
        return False

    def _drive(self, target, w_cap, time_limit):
        """target 방향으로 감속 회전 후 정지. (정상 여부, 설명)"""
        t0 = time.time(); moved = 0.0; y_prev = self.get_yaw()
        while True:
            rem = wrap(target - self.get_yaw())
            g = self.get_gyro()
            if abs(rem) < max(abs(g) * self.coast_s, math.radians(0.3)):
                break
            w = math.copysign(max(self.w_min, min(w_cap, self.kp * abs(rem))), rem)
            self._cmd(w); self.spin(0.02)
            now = time.time(); y = self.get_yaw()
            moved += abs(wrap(y - y_prev)); y_prev = y
            if now - t0 > time_limit:
                self.stop(); return False, f'시간 초과 {time_limit:.1f}초(남은 각 {math.degrees(rem):+.1f}°)'
            if now - t0 > 1.5 and moved < math.radians(5):
                self.stop(); return False, f'1.5초 동안 {math.degrees(moved):.1f}°만 돎 — 자이로/구동 이상'
        self.stop()
        return True, ''

    def turn_by(self, delta_rad):
        """현재 EKF 방향에서 delta만큼 회전. 결과 dict(ok, msg, err_deg, fixes, secs)."""
        t0 = time.time()
        target = wrap(self.get_yaw() + delta_rad)
        limit = abs(delta_rad) / self.w_min + 3.0
        ok, msg = self._drive(target, self.w_max, limit)
        fixes = 0
        while ok:
            self._settle()
            rem = wrap(target - self.get_yaw())
            if abs(rem) <= self.tol or fixes >= self.max_fix:
                break
            fixes += 1
            ok, msg = self._drive(target, self.w_min, abs(rem) / self.w_min + 3.0)
        rem = wrap(target - self.get_yaw())
        return dict(ok=ok, msg=msg, err_deg=-math.degrees(rem), fixes=fixes, secs=time.time() - t0)
