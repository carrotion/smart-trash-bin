#!/usr/bin/env python3
"""지정된 프레임 쌍(기본 map_replay->odom)의 TF를 지정 시간 동안 주기적으로
폴링한다.

slam_replay_sweep.py(pc_tasks.md 11-a)가 재생 세션과 병렬로 띄워서, bag
재생이 끝나기 직전까지 최신 map->odom 변환을 계속 갱신해두는 용도 —
slam_toolbox가 종료되면 TF 발행도 끊기므로, 종료 직전 시점의 값을 잡아야 한다.

⚠️ 설계 변경(2026-08-29 밤): 처음엔 "duration 끝나면 stdout에 한 번 출력"
방식이었는데, 호출부(slam_replay_sweep.py)가 프로세스 정리를 위해 이 프로세스를
SIGKILL할 수도 있는 상황(kill_process_tree가 안전을 위해 finally에서 항상
호출됨)과 맞물려서 **duration이 끝나기 전에 죽으면 print 버퍼가 통째로
날아가 결과가 완전히 유실**되는 문제를 실측으로 겪었다(정상적으로 도는
것처럼 보였는데 stdout 파일이 계속 0바이트). --out 파일에 **매 성공적인
폴링마다 즉시 write+flush**하는 방식으로 바꿔서, 중간에 죽어도 마지막으로
기록된 값이 항상 남도록 한다 — 원자성이 필요 없는 단일 갱신값이라
tmp+rename 같은 복잡한 처리 없이 매번 덮어써도 안전하다.

map_frame을 map_replay로 쓰는 이유: 재생용 slam_toolbox 인스턴스가
mapper_params_online_async.yaml의 기본 map_frame(map)을 그대로 쓰면, bag에
이미 녹화돼 있던 원본 세션의 map->odom TF와 같은 프레임 쌍을 두 발행자가
동시에 내보내게 돼 tf2가 경고를 내고 값이 튈 수 있다 — slam_replay_sweep.py는
항상 map_frame 오버라이드와 함께 이 프로브를 쓴다.

사용법:
    python3 tools/tf_yaw_probe.py --duration 40 --target-frame map_replay \
        --source-frame odom --out /tmp/probe.txt [--use-sim-time]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quat(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=40.0,
                     help='폴링을 계속할 wall-clock 초')
    ap.add_argument('--target-frame', default='map_replay')
    ap.add_argument('--source-frame', default='odom')
    ap.add_argument('--use-sim-time', action='store_true')
    ap.add_argument('--poll-interval', type=float, default=0.2)
    ap.add_argument('--out', required=True,
                     help='매 성공적인 폴링마다 최신값을 덮어쓰는 파일 '
                          '(YAW_DEG=.. / X=.. / Y=.. 세 줄)')
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('tf_yaw_probe',
                           parameter_overrides=[rclpy.parameter.Parameter(
                               'use_sim_time', value=args.use_sim_time)])
    buf = Buffer()
    listener = TransformListener(buf, n)

    # 시작 시점에 "아직 못 잡음"을 파일에 먼저 남겨둔다 — 프로세스가 첫 성공
    # 폴링 전에 죽으면 --out 파일 자체가 없어서 호출부가 구분 못 하는 걸 방지.
    with open(args.out, 'w') as f:
        f.write('YAW_DEG=NONE\nX=NONE\nY=NONE\n')

    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=args.poll_interval)
        try:
            tf = buf.lookup_transform(args.target_frame, args.source_frame,
                                       Time())
            q = tf.transform.rotation
            yaw = yaw_from_quat(q)
            x, y = tf.transform.translation.x, tf.transform.translation.y
            with open(args.out, 'w') as f:
                f.write(f'YAW_DEG={yaw:.3f}\nX={x:.3f}\nY={y:.3f}\n')
                f.flush()
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
