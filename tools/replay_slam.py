#!/usr/bin/env python3
"""녹화한 가방을 재생해 SLAM만 다시 돌리고 맵 품질을 잰다 — 로봇 없이.

왜 이게 핵심인가
----------------
지금까지 SLAM 파라미터를 바꾸려면 로봇을 들고 집을 한 바퀴 돌아야 했다. 1회에
수십 분, 매번 경로가 달라서 **비교 자체가 불가능**했다. 같은 가방을 재생하면
입력이 완전히 동일하므로, 정렬도 차이가 곧 파라미터 효과다.

    A안 71.2% vs B안 78.4%  ->  논쟁 없이 결론

동작 방식
---------
가방에서 `/scan`, `/odom`, `/tf_static`만 재생한다.
⚠️ `/tf`는 **재생하지 않는다.** 가방의 /tf에는 원래 SLAM이 만든 map->odom이 들어
   있어서, 새로 띄운 SLAM과 같은 변환을 두 노드가 발행하게 된다(TF 트리 오염).
   대신 `/odom`에서 odom->base_link를 다시 만들어 발행한다. base_link->base_laser는
   /tf_static에 있으니 그대로 쓴다.

사용법
------
    python3 tools/replay_slam.py --bag bags/2026...._teleop_tour \\
        --params robot/src/smart_trash_bringup/config/mapper_params_online_async.yaml \\
        --label baseline
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time


def wait_for(cond, timeout, period=0.5):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(period)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--params', required=True)
    ap.add_argument('--label', default='run')
    ap.add_argument('--rate', type=float, default=1.0,
                    help='재생 배속. 올리면 빠르지만 SLAM이 스캔을 놓칠 수 있다')
    ap.add_argument('--overrides', default='',
                    help='"키=값,키=값" 형태로 파라미터 덮어쓰기')
    args = ap.parse_args()

    if not os.path.isdir(args.bag):
        print(f'가방이 없다: {args.bag}'); return 1

    # 파라미터 덮어쓰기: 원본을 건드리지 않고 임시 사본을 만든다
    params = args.params
    tmpdir = None
    if args.overrides:
        tmpdir = tempfile.mkdtemp(prefix='replay_slam_')
        params = os.path.join(tmpdir, 'params.yaml')
        ov = dict(kv.split('=', 1) for kv in args.overrides.split(','))
        with open(args.params) as f:
            lines = f.readlines()
        seen = set()
        out = []
        for ln in lines:
            stripped = ln.strip()
            key = stripped.split(':')[0].strip() if ':' in stripped else None
            if key in ov and not stripped.startswith('#'):
                indent = ln[:len(ln) - len(ln.lstrip())]
                out.append(f'{indent}{key}: {ov[key]}\n')
                seen.add(key)
            else:
                out.append(ln)
        missing = set(ov) - seen
        if missing:
            print(f'⚠️ 파라미터 파일에 없는 키라 덮어쓰지 못했다: {missing}')
            print('   (오타이거나 파일 구조가 달라진 것 — 그대로 진행하면 실험이 무의미하다)')
            return 1
        open(params, 'w').writelines(out)
        print(f'덮어쓴 파라미터: {ov}')

    procs = []

    def spawn(cmd, name):
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, preexec_fn=os.setsid)
        procs.append((name, p))
        return p

    try:
        # ⚠️ 재생에서는 반드시 **모의 시간(use_sim_time)** 을 쓴다.
        # 가방의 타임스탬프는 과거인데 노드가 현재 벽시계를 쓰면, SLAM이 스캔과
        # TF의 시각을 맞추지 못해 **스캔을 전부 조용히 버린다**(실측: 맵이 아예
        # 안 만들어졌는데 에러 로그도 없었다). `ros2 bag play --clock`이 /clock을
        # 발행하고 모든 노드가 그 시계를 따르게 해야 일관된다.
        print('SLAM 기동 (use_sim_time)...')
        spawn(['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
               '--ros-args', '--params-file', params,
               '-p', 'use_sim_time:=true'], 'slam')

        print('odom -> base_link TF 재발행 노드 기동...')
        spawn([sys.executable, os.path.join(os.path.dirname(__file__),
                                            'odom_tf_republisher.py'),
               '--ros-args', '-p', 'use_sim_time:=true'], 'odomtf')
        time.sleep(6)

        print(f'가방 재생 (배속 {args.rate}, --clock)...')
        play = spawn(['ros2', 'bag', 'play', args.bag, '--rate', str(args.rate),
                      '--clock', '200',
                      '--topics', '/scan', '/odom', '/tf_static'], 'play')
        play.wait()
        print('재생 완료 — SLAM 마무리 대기 5초')
        time.sleep(5)

        print('\n=== 맵 품질 ===')
        subprocess.run([sys.executable,
                        os.path.join(os.path.dirname(__file__), 'map_quality.py'),
                        '--split', '--sim-time'])
    finally:
        for name, p in reversed(procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except OSError:
                pass
        time.sleep(2)
        for name, p in reversed(procs):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except OSError:
                    pass
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
