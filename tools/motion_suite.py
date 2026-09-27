#!/usr/bin/env python3
"""주행 기반 회귀 시험 모음 — 무엇을 고치든 이걸 다시 돌려 다른 동작이 깨지지 않았는지 확인한다(docs/motion_foundation.md 원칙 2).

    python3 tools/motion_suite.py            # 오프라인만(로봇 없이): 정적 검사 + 로직 가상 시험
    python3 tools/motion_suite.py --floor    # + 바닥 실물: ④ 각도 회전, ②' 직진 거리, 정사각형 조합
바닥 모드 조건: 로봇이 바닥(충전 아님), 반경 0.5 m + 앞뒤 0.8 m + 정사각형용 앞·왼쪽 1.1 m 여유, velocity 모드 구동 서비스.
각 시험의 합격 기준은 해당 도구와 motion_foundation.md에 있다. 결과 요약은 docs/calibration/motion_suite_*.json.
"""
import argparse, json, os, re, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CORE = ['tools/turn_to_angle.py', 'tools/drive_distance.py', 'tools/step_navigator.py', 'tools/turn_angle_test.py',
        'tools/drive_distance_test.py', 'tools/square_test.py', 'tools/test_turn_to_angle_sim.py',
        'tools/test_step_navigator_sim.py', 'tools/step_nav_acceptance.py',
        'robot/src/smart_trash_motion/smart_trash_motion/diff_drive_node.py',
        'robot/src/smart_trash_sensors/smart_trash_sensors/imu_publisher_node.py']


def run(name, cmd, timeout, ok_pattern=None):
    t0 = time.time()
    try:
        p = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=timeout)
        out = p.stdout + p.stderr
        ok = p.returncode == 0 if ok_pattern is None else bool(re.search(ok_pattern, out))
    except subprocess.TimeoutExpired:
        out, ok = f'시간 초과 {timeout}s', False
    last = [l for l in out.strip().splitlines() if l.strip() and 'RTPS' not in l][-3:]
    print(f'{"✅" if ok else "❌"} {name} ({time.time() - t0:.0f}s)' + ''.join(f'\n     {l}' for l in last), flush=True)
    return dict(name=name, ok=ok, secs=round(time.time() - t0, 1), tail=last)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--floor', action='store_true', help='바닥 실물 시험까지')
    a = ap.parse_args()
    res = [run('정적 검사(pyflakes, 핵심 파일)', [sys.executable, '-m', 'pyflakes'] + CORE, 60),
           run('④ 각도 회전 로직 가상 시험', [sys.executable, 'tools/test_turn_to_angle_sim.py'], 400),
           run('4층 주행기 로직 가상 시험', [sys.executable, 'tools/test_step_navigator_sim.py'], 60)]
    if a.floor:
        bat = subprocess.run(['bash', '-c', f'tail -1 {REPO}/logs/battery/$(date +%Y%m%d).csv | cut -d, -f3'],
                             capture_output=True, text=True).stdout.strip()
        print(f'배터리 {bat} V', flush=True)
        res += [run('④ 각도 회전(바닥)', [sys.executable, 'tools/turn_angle_test.py', '--label', 'suite'], 300, r'④ 각도 회전: ✅'),
                run("②' 직진 거리(바닥)", [sys.executable, 'tools/drive_distance_test.py', '--reps', '1', '--label', 'suite'], 300, r'직진 거리: ✅'),
                run('조합: 정사각형 좌회전(바닥)', [sys.executable, 'tools/square_test.py', '--side', '0.8', '--label', 'suite'], 300, r'① 실제 복귀')]
    ok = all(r['ok'] for r in res)
    print(f'\n회귀 시험: {"✅ 전부 통과" if ok else "❌ 실패 있음"} ({sum(r["ok"] for r in res)}/{len(res)})')
    out = os.path.join(REPO, 'docs/calibration', f'motion_suite_{time.strftime("%Y%m%d_%H%M")}{"_floor" if a.floor else ""}.json')
    json.dump(dict(floor=a.floor, results=res), open(out, 'w'), ensure_ascii=False, indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
