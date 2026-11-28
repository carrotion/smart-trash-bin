#!/usr/bin/env python3
"""저장된 지도에서 위치 잡기용 SLAM 설정 파일을 만든다 — 평소 매핑 설정을 그대로 두고 세 가지만 바꾼 사본.

    mode: localization, map_file_name: <지도 이름(확장자 없이)>, map_start_pose: [x, y, 방향(rad)]
시작 자세는 tools/locate_on_map.py가 스캔으로 찾은 값을 넣는다(사람이 놓은 자리를 믿지 않는다).

    python3 tools/localize_params.py --map ~/maps/home_20261011 --pose=-0.18,0.38,178.5 --out /tmp/slam_localize.yaml [--sim]
"""
import argparse, math, os, sys
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True); ap.add_argument('--pose', required=True, help='x,y(m),방향(도)')
    ap.add_argument('--out', required=True); ap.add_argument('--sim', action='store_true', help='use_sim_time')
    ap.add_argument('--params', default=os.path.join(HERE, '../robot/src/smart_trash_bringup/config/mapper_params_online_async.yaml'))
    a = ap.parse_args()
    base = os.path.expanduser(a.map)
    if not (os.path.isfile(base + '.posegraph') and os.path.isfile(base + '.data')):
        print(f'❌ 위치 잡기용 자료가 없다: {base}.posegraph / .data (10-10 이후에 저장한 지도만 있다)'); return 2
    x, y, deg = (float(v) for v in a.pose.split(','))
    P = yaml.safe_load(open(a.params)); rp = P[next(iter(P))]['ros__parameters']
    rp.update(mode='localization', map_file_name=base, map_start_pose=[x, y, math.radians(deg)])
    rp.pop('map_start_at_dock', None)
    # 11-28: 어뭉이방에서 위치가 2.92 m 튀었다(12:06:27) — 저장 지도와 거짓으로 맞춘 고리 닫기(찾는 범위 3.0 m 바로 안).
    #   녹화 재생 비교(도메인 42, 같은 녹화를 설정만 바꿔 다시 돌림, 2초마다 스캔이 지도 벽 10 cm 안에 드는 비율):
    #     3.0 m(그대로): 튐 재현, 이후 50%대 / 1.0 m·고리 닫기 끔: 안 튀지만 필요한 0.3 m 보정도 못 해 40~69%로 밀림
    #     2.0 m: 안 튀고 보정은 함, 끝 자세 (−1.85, −1.38) = 전체 지도 찾기 (−1.84, −1.37), 80% 미만 1/109장(그대로는 11)
    #     오전 15분 녹화: 3.0 m 94.1% / 2.0 m 94.3%(80% 미만 22 → 20장) — 나빠진 곳 없음
    #   지도를 그릴 때(매핑)는 그대로 3.0 m — 위치 잡기에서만 줄인다
    rp['loop_search_maximum_distance'] = 2.0
    if a.sim:
        rp['use_sim_time'] = True
    yaml.safe_dump(P, open(a.out, 'w'), allow_unicode=True)
    print(f'{a.out}: {os.path.basename(base)} 에서 ({x:+.2f}, {y:+.2f}) m, {deg:+.1f}° 로 시작')
    return 0


if __name__ == '__main__':
    sys.exit(main())
