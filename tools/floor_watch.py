#!/usr/bin/env python3
"""좁은 곳에 들어가기 전 "후진해서 바닥 보기"가 물을 곳 — 카메라 검출을 거리·옆 위치로 바꾼다 (2026-11-22).

하는 일: 관제 대시보드(8767)의 `/api/detect`에서 사람·물건 상자를 받아, 상자의 **아래 끝**을 바닥 거리로 환산하고
         **가로 위치**를 화면 중심에서 옆 거리로 어림한다. `step_navigator._look_floor`가 부르면 (거리, 옆, 이름) 목록을 돌려준다.

⚠️ 한계: 지금 인식은 COCO 80종만 안다. 슬리퍼·옷·문턱은 그 목록에 없어 **잡지 못한다.** 지금 잡히는 것은 사람·병·의자·
   가방·상자 같은 아는 물건뿐이다. 모르는 물체는 1) 따로 학습한 모델, 2) 3D 깊이 카메라(2027-01)의 몫이다.
   그래서 이 확인은 "아는 물체가 앞에 있나"를 보는 정도로 두고, 결과는 단계 기록(`look_back`)에 남겨 나중에 학습 자료로 쓴다.
"""
import json
import time
import math
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

API = 'http://127.0.0.1:8767'
HFOV_DEG = 50.0          # C270 가로 화각(어림) — 화면 중심에서 옆 거리로 바꿀 때 쓴다


def make_check(api=API, height=480, width=640, keep_names=('person', 'bottle', 'chair', 'backpack', 'handbag',
                                                         'suitcase', 'cup', 'book', 'vase', 'potted_plant',
                                                         'couch', 'bed', 'dining_table', 'tv', 'laptop')):
    """floor_check 함수를 만든다. 대시보드에 닿지 않으면 None을 돌려준다(주행기는 확인 없이 그대로 간다)."""
    try:
        urllib.request.urlopen(api + '/api/detect?peek=1', timeout=1.5).read()     # 있는지만 본다(카메라는 안 켠다)
    except Exception:
        return None
    from camera_detect import row_to_distance
    half = math.tan(math.radians(HFOV_DEG / 2))

    t_wake = {'t': 0.0}

    def wake():
        """후진을 시작할 때 부른다 — 카메라를 켜는 데 3~4초 걸리니 물러나는 동안 데운다(11-25, 카메라는 필요할 때만 켬)."""
        t_wake['t'] = time.time()
        try:
            urllib.request.urlopen(api + '/api/detect', timeout=1.5).read()
        except Exception:
            pass

    def fresh(timeout=8.0):
        """wake 뒤에 새로 한 검출 결과를 기다려 돌려준다. 시간 안에 안 오면 None."""
        end = time.time() + timeout
        while time.time() < end:
            try:
                d = json.load(urllib.request.urlopen(api + '/api/detect', timeout=2))
                if d.get('on') and d.get('t', 0) > t_wake['t'] + 0.3 and d.get('age_s', 9) < 1.5:
                    return d
            except Exception:
                pass
            time.sleep(0.3)
        return None

    def check():
        d = fresh()
        if d is None:
            return []                                   # 새 화면이 안 왔으면 판단하지 않는다
        out = []
        for q in d.get('dets', []):
            if q.get('name') not in keep_names or q.get('score', 0) < 0.45:
                continue
            dist = row_to_distance(q['y2'], height)
            if dist is None:
                continue
            cx = (q['x1'] + q['x2']) / 2
            lateral = (cx - width / 2) / (width / 2) * half * dist     # 화면 중심 기준 옆 거리(m)
            out.append((round(dist, 2), round(lateral, 2), q['name']))
        return out
    def ready(min_brightness=35.0):
        """카메라로 바닥을 볼 수 있나. 11-25 드레스룸(불 꺼짐)에서 화면 평균 밝기가 거의 0이라 검출이 될 수 없었다."""
        d = fresh()
        if d is None:
            return False, '카메라 화면이 안 옴'
        b = d.get('brightness')
        if b is not None and b < min_brightness:
            return False, f'어두움(밝기 {b:.0f})'
        return True, ''
    check.ready = ready
    check.wake = wake
    return check


if __name__ == '__main__':
    c = make_check()
    print('확인 함수:', '없음(카메라 대시보드 응답 없음)' if c is None else '있음')
    if c:
        print('지금 잡힌 것:', c())
