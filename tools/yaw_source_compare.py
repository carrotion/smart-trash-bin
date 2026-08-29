#!/usr/bin/env python3
"""rf2o / EKF / 휠(odom) 세 오도메트리 소스의 누적 요(yaw)를 비교 (pc_tasks.md
11-b, Track 0 후속 — docs/midterm_review_20260829.md "재수립된 계획" 2번).

`tools/loop_gyro_diag.py`(2026-08-29 밤 확장, rf2o_yaw_deg 컬럼 추가)가 남긴
CSV를 읽어서, 세 소스가 시간축에서 어느 지점부터 갈라지기 시작하는지 찾는다.
목적: "자이로 드리프트 구간"과 "rf2o가 다르게 보는 구간"을 구분해서, 08-29
중간평가에서 확정된 장주기 요 드리프트(자이로 바이어스 방랑 ~3°/min +
회전 스케일 오차)를 rf2o가 실제로 잡아주는지 실측으로 판단하는 근거를 만든다.

각도는 -180~180으로 매 샘플 wrap되어 있어서 그대로 비교하면 경계에서
거짓 발산으로 보이므로, 먼저 언랩(unwrap — 스텝마다 최단 각도 변화만
누적)해서 연속적인 누적 회전량으로 바꾼 뒤 비교한다.

rf2o가 로그에 없던 구간(컬럼이 빈 값)은 자동으로 건너뛰고, 사용 가능한
소스만으로 비교한다 — rf2o 없이 만든 기존 loop_gyro_diag CSV도 그대로
쓸 수 있다(휠 vs EKF만 비교).

사용법:
    # 자체 검증 (합성 데이터, CSV/로봇 불필요)
    python3 tools/yaw_source_compare.py --self-test

    # 실측 CSV 분석
    python3 tools/yaw_source_compare.py --csv docs/calibration/loop_gyro_diag.csv \
        --diverge-threshold-deg 10
"""
import argparse
import csv
import sys


def wrap_deg(a):
    """각도를 -180~180 범위로 정규화."""
    return (a + 180.0) % 360.0 - 180.0


def unwrap_deg(yaws):
    """스텝마다 최단 각도 변화만 누적해서 wrap 경계(±180)를 넘어도 연속적인
    누적 회전량으로 만든다. None(결측)이 섞여 있으면 그 지점은 건너뛴다
    (직전 유효값과의 차이로 계속 누적) — 원소 수는 입력과 동일하게 유지,
    결측 위치는 None 그대로."""
    out = []
    prev_raw = None
    acc = None
    for y in yaws:
        if y is None:
            out.append(None)
            continue
        if acc is None:
            acc = y
        else:
            acc += wrap_deg(y - prev_raw)
        prev_raw = y
        out.append(acc)
    return out


def relative_to_start(series):
    """각 소스를 자기 시작값 기준 상대각(시작=0)으로 맞춘다 — 절대 오프셋
    차이(센서마다 초기 heading 기준이 다를 수 있음)를 제거하고 '변화량'만
    비교하기 위함."""
    start = next((v for v in series if v is not None), None)
    if start is None:
        return series
    return [None if v is None else v - start for v in series]


def load_csv(path):
    t, odom, ekf, rf2o = [], [], [], []

    def parse(v):
        if v is None or v == '':
            return None
        try:
            return float(v)
        except ValueError:
            return None

    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(parse(row.get('t')))
            odom.append(parse(row.get('odom_yaw_deg')))
            ekf.append(parse(row.get('ekf_yaw_deg')))
            rf2o.append(parse(row.get('rf2o_yaw_deg')))
    return t, odom, ekf, rf2o


def find_divergence(t, a, b, threshold_deg):
    """|a-b|가 threshold_deg를 처음 넘는(그리고 그 뒤로 계속 넘는 — 순간
    노이즈 스파이크 배제) 시점을 찾는다. 못 찾으면 None."""
    n = len(t)
    for i in range(n):
        if a[i] is None or b[i] is None:
            continue
        diff = abs(a[i] - b[i])
        if diff >= threshold_deg:
            # 순간 스파이크가 아니라 지속되는 발산인지 확인 — 이후 5개 유효
            # 샘플(또는 남은 전부) 중 과반이 여전히 임계값 이상인지 체크.
            checked, sustained = 0, 0
            for j in range(i, n):
                if a[j] is None or b[j] is None:
                    continue
                checked += 1
                if abs(a[j] - b[j]) >= threshold_deg:
                    sustained += 1
                if checked >= 5:
                    break
            if checked == 0 or sustained / checked >= 0.5:
                return t[i], diff
    return None


def compare(t, odom, ekf, rf2o, threshold_deg):
    odom_u = relative_to_start(unwrap_deg(odom))
    ekf_u = relative_to_start(unwrap_deg(ekf))
    rf2o_u = relative_to_start(unwrap_deg(rf2o))

    def last_valid(series):
        for v in reversed(series):
            if v is not None:
                return v
        return None

    result = {
        'final_odom_deg': last_valid(odom_u),
        'final_ekf_deg': last_valid(ekf_u),
        'final_rf2o_deg': last_valid(rf2o_u),
        'divergence': {},
    }
    # ⚠️ "데이터 없음"(비교 자체가 불가능)과 "발산 없음"(비교했는데 안 벌어짐)은
    # 의미가 다르므로 값을 뭉개지 않는다 — 둘 다 find_divergence()가 None을
    # 반환하는 케이스라, 여기서 구분 안 하면 호출부가 "왜 None인지" 알 방법이
    # 없다(실제로 이 버그로 self-test가 "발산 없음"인지 "데이터 없음"인지
    # 구분 못 한 채 통과했던 적이 있었음 — 반드시 status 태그를 같이 넣는다).
    pairs = [('odom_vs_ekf', odom_u, ekf_u), ('odom_vs_rf2o', odom_u, rf2o_u),
             ('ekf_vs_rf2o', ekf_u, rf2o_u)]
    for name, a, b in pairs:
        if all(v is None for v in a) or all(v is None for v in b):
            result['divergence'][name] = {'status': 'no_data'}
        else:
            found = find_divergence(t, a, b, threshold_deg)
            if found is None:
                result['divergence'][name] = {'status': 'no_divergence'}
            else:
                t_div, diff = found
                result['divergence'][name] = {'status': 'diverged', 't': t_div, 'diff': diff}
    return result, odom_u, ekf_u, rf2o_u


def print_report(result, threshold_deg, t0):
    def fmt(v):
        return f'{v:+.1f}°' if v is not None else 'N/A(데이터 없음)'

    print(f'최종 누적 요(시작 기준 상대각):')
    print(f'  휠(/odom)              : {fmt(result["final_odom_deg"])}')
    print(f'  EKF(/odometry/filtered): {fmt(result["final_ekf_deg"])}')
    print(f'  rf2o(/odom_rf2o)       : {fmt(result["final_rf2o_deg"])}')
    print(f'\n소스 쌍별 발산 시점 (|차이| >= {threshold_deg}° 지속 구간의 시작):')
    for name, d in result['divergence'].items():
        if d['status'] == 'no_data':
            print(f'  {name:16s}: N/A (한쪽 데이터 없음)')
        elif d['status'] == 'diverged':
            print(f'  {name:16s}: t={d["t"] - t0:.1f}s 부근 (그 시점 차이 {d["diff"]:.1f}°)')
        else:  # no_divergence
            print(f'  {name:16s}: 세션 내내 {threshold_deg}° 미만 (발산 없음)')


def run_from_csv(args):
    t, odom, ekf, rf2o = load_csv(args.csv)
    if not t:
        print('❌ CSV가 비어있거나 읽을 수 없습니다'); return 1
    t0 = next((v for v in t if v is not None), t[0])
    result, odom_u, ekf_u, rf2o_u = compare(t, odom, ekf, rf2o, args.diverge_threshold_deg)
    print_report(result, args.diverge_threshold_deg, t0)

    if args.series_out:
        with open(args.series_out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['t', 'odom_rel_deg', 'ekf_rel_deg', 'rf2o_rel_deg'])
            w.writeheader()
            for i in range(len(t)):
                w.writerow({'t': t[i], 'odom_rel_deg': odom_u[i],
                            'ekf_rel_deg': ekf_u[i], 'rf2o_rel_deg': rf2o_u[i]})
        print(f'\n시계열(상대각) 저장: {args.series_out}')
    return 0


def self_test():
    """합성 데이터로 도구 자체를 검증한다 — CSV/로봇 불필요.
    세 소스가 한동안 일치하다가 특정 시점부터 한 소스만 드리프트가 붙는
    시나리오를 만들어서, find_divergence()가 그 지점을 정확히 찾아내는지
    확인한다."""
    failures = []

    def check(name, cond, detail):
        status = 'PASS' if cond else 'FAIL'
        print(f'  [{status}] {name}' + (f' — {detail}' if not cond and detail else ''))
        if not cond:
            failures.append(name)

    # 1) unwrap_deg: 179 -> -179(실제로는 +2도 계속 회전)를 올바르게 이어붙이는지
    raw = [178.0, 179.0, -179.0, -178.0]  # 178->179->181(=-179)->182(=-178)
    u = unwrap_deg(raw)
    check('unwrap_deg가 ±180 경계를 연속 누적으로 처리',
          abs(u[-1] - 182.0) < 1e-6, f'got {u}')

    # 2) relative_to_start: 시작값을 빼서 0에서 시작하는지
    r = relative_to_start([10.0, 15.0, 20.0])
    check('relative_to_start 시작=0', r[0] == 0.0 and abs(r[-1] - 10.0) < 1e-9, f'got {r}')

    # 3) 핵심 시나리오: 100샘플(1Hz), t=0~50은 odom/ekf/rf2o 전부 0도 유지,
    #    t=50부터 odom만 +2도/초로 드리프트, ekf/rf2o는 계속 0도.
    n = 100
    t = [float(i) for i in range(n)]
    odom = [0.0 if i < 50 else (i - 50) * 2.0 for i in range(n)]
    ekf = [0.0] * n
    rf2o = [0.0] * n
    result, *_ = compare(t, odom, ekf, rf2o, threshold_deg=10.0)
    # 임계값 10도를 넘는 시점: (i-50)*2 >= 10 -> i=55
    div = result['divergence']['odom_vs_ekf']
    check('odom_vs_ekf 발산 시점이 t=55 부근(주입 지점 t=50에서 5초 이내)',
          div['status'] == 'diverged' and abs(div['t'] - 55.0) <= 2.0, f'got {div}')
    div2 = result['divergence']['odom_vs_rf2o']
    check('odom_vs_rf2o도 동일하게 발산 검출',
          div2['status'] == 'diverged' and abs(div2['t'] - 55.0) <= 2.0, f'got {div2}')
    div3 = result['divergence']['ekf_vs_rf2o']
    check('ekf_vs_rf2o는 둘 다 0도라 "발산 없음"(no_divergence, 데이터는 있음)',
          div3['status'] == 'no_divergence', f'got {div3}')

    # 4) 결측치(rf2o 컬럼 전부 없음)를 우아하게 처리하는지 — "데이터 없음"과
    #    "발산 없음"을 절대 혼동하면 안 됨(이전 시도에서 실제로 겹쳐서 놓쳤던
    #    버그 — status 태그 없이 둘 다 None으로 반환했을 때는 이 self-test조차
    #    값만 보고 통과시켜서 못 잡았다).
    result2, *_ = compare(t, odom, ekf, [None] * n, threshold_deg=10.0)
    check('rf2o 전부 결측이어도 크래시 없이 처리',
          result2['final_rf2o_deg'] is None, f'got {result2["final_rf2o_deg"]}')
    check('rf2o 결측 시 관련 발산 항목은 no_data(no_divergence와 구분됨)',
          result2['divergence']['odom_vs_rf2o']['status'] == 'no_data',
          f'got {result2["divergence"]["odom_vs_rf2o"]}')

    # 5) 순간 스파이크(노이즈 1샘플만 튐)는 지속 발산으로 오판하지 않는지
    odom_spike = [0.0] * n
    odom_spike[30] = 50.0  # 딱 1샘플만 튀고 바로 원복
    result3, *_ = compare(t, odom_spike, ekf, rf2o, threshold_deg=10.0)
    check('1샘플 스파이크는 지속 발산으로 오판하지 않음(no_divergence)',
          result3['divergence']['odom_vs_ekf']['status'] == 'no_divergence',
          f'got {result3["divergence"]["odom_vs_ekf"]}')

    print()
    if failures:
        print(f'❌ {len(failures)}개 실패: {failures}')
        return 1
    print('✅ 전부 통과')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--self-test', action='store_true',
                     help='합성 데이터로 도구 자체 검증 (CSV/로봇 불필요)')
    ap.add_argument('--csv', help='loop_gyro_diag.py 출력 CSV')
    ap.add_argument('--diverge-threshold-deg', type=float, default=10.0,
                     help='이 값(도) 이상 지속되는 차이를 발산으로 판정 (기본 10)')
    ap.add_argument('--series-out', help='정규화된 상대각 시계열을 CSV로 저장(선택)')
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.csv:
        ap.error('--csv가 필요합니다 (또는 --self-test로 자체 검증만 실행)')
    return run_from_csv(args)


if __name__ == '__main__':
    sys.exit(main())
