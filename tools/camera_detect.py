#!/usr/bin/env python3
"""카메라 화면에서 사람을 찾는다 — YOLOX-nano(ONNX, 416×416)를 파이 CPU에서 (2026-11-22).

왜: 깊이 카메라(2027-01) 전까지 2D 웹캠으로 "사람이 보인다"를 먼저 붙인다(엣지 우선 설계 7절의 가벼운 일).
    파이에서 61 ms/장이라 3장에 한 번 돌리면 코어 0.2개 안쪽이다.
모델: ~/models/yolox_nano.onnx (Megvii 공개 배포, 3.6 MB). 다른 클래스가 필요하면 keep=로 고른다(COCO 이름 참고).

    python3 tools/camera_detect.py --still ~/camera_shots/x.jpg        # 한 장으로 시험(상자 그려 저장)
    python3 tools/camera_detect.py --test                              # 속도·빈 화면 오검출 확인
"""
import argparse, os, sys, time

import numpy as np

MODEL = os.path.expanduser('~/models/yolox_nano.onnx')
COCO = ('person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign '
        'parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella handbag '
        'tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove skateboard surfboard '
        'tennis_racket bottle wine_glass cup fork knife spoon bowl banana apple sandwich orange broccoli carrot '
        'hot_dog pizza donut cake chair couch potted_plant bed dining_table toilet tv laptop mouse remote keyboard '
        'cell_phone microwave oven toaster sink refrigerator book clock vase scissors teddy_bear hair_drier toothbrush').split()
STRIDES = (8, 16, 32)


class Detector:
    """YOLOX-nano 한 장 판정. 입력은 BGR uint8, 출력은 [[x1,y1,x2,y2,score,class], …](원본 화면 좌표)."""

    def __init__(self, model=MODEL, size=416, conf=0.35, keep=(0,), threads=2):
        import onnxruntime as ort
        so = ort.SessionOptions(); so.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(model, so, providers=['CPUExecutionProvider'])
        self.name = self.sess.get_inputs()[0].name
        self.size, self.conf, self.keep = size, conf, set(keep)
        self.grids = np.concatenate([self._grid(size // s) for s in STRIDES]).astype(np.float32)
        # ⚠️ 11-22: 여기에 size//stride(52·26·13)를 넣어 상자가 4분의 1로 작게 나왔다 — 디코딩에 쓰는 값은 픽셀 스트라이드(8·16·32)다
        self.strides = np.repeat(list(STRIDES), [(size // s) ** 2 for s in STRIDES]).astype(np.float32)

    @staticmethod
    def _grid(n):
        y, x = np.meshgrid(np.arange(n), np.arange(n), indexing='ij')
        return np.stack((x, y), 0).reshape(2, -1).T

    def preprocess(self, frame):
        """YOLOX 방식: 비율 지키고 회색(114)으로 채워 416×416. 늘린 비율을 돌려준다."""
        import cv2
        h, w = frame.shape[:2]
        r = min(self.size / h, self.size / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        img = np.full((self.size, self.size, 3), 114, np.uint8)
        left, top = (self.size - nw) // 2, (self.size - nh) // 2
        img[top:top + nh, left:left + nw] = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = img[:, :, ::-1].astype(np.float32).transpose(2, 0, 1)[None]      # BGR→RGB, HWC→CHW
        return x, r, left, top

    def detect(self, frame):
        x, r, left, top = self.preprocess(frame)
        out = self.sess.run(None, {self.name: x})[0][0]                      # (3549, 85) — obj·cls는 이미 sigmoid
        obj, cls = out[:, 4], out[:, 5:]
        score = obj * cls.max(1); k = (score >= self.conf) & np.isin(cls.argmax(1), list(self.keep))
        if not k.any():
            return []
        p = out[k]; s = score[k]; c = cls[k].argmax(1)
        cx = (p[:, 0] + self.grids[k][:, 0]) * self.strides[k]
        cy = (p[:, 1] + self.grids[k][:, 1]) * self.strides[k]
        bw, bh = np.exp(p[:, 2]) * self.strides[k], np.exp(p[:, 3]) * self.strides[k]
        boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], 1)
        boxes = (boxes - [left, top, left, top]) / r
        keep = []
        for i in np.argsort(-s):
            if all(self._iou(boxes[i], boxes[j]) < 0.45 for j in keep):
                keep.append(i)
        return [[float(boxes[i][0]), float(boxes[i][1]), float(boxes[i][2]), float(boxes[i][3]), float(s[i]), int(c[i])] for i in keep]

    @staticmethod
    def _iou(a, b):
        x1, y1 = max(a[0], b[0]), max(a[1], b[1]); x2, y2 = min(a[2], b[2]), min(a[3], b[3])
        iw, ih = max(0.0, x2 - x1), max(0.0, y2 - y1); inter = iw * ih
        ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
        return inter / ua if ua > 0 else 0.0

    def draw(self, frame, dets):
        """상자·이름·점수를 그려 돌려준다(대시보드 화면용)."""
        import cv2
        for x1, y1, x2, y2, s, c in dets:
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (60, 200, 60), 2)
            cv2.putText(frame, f'{COCO[c]} {s:.2f}', (int(x1), max(12, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 200, 60), 1, cv2.LINE_AA)
        return frame


FLOOR = {'a': 347.0, 'b': 96.0}      # (옛) 줄 = a + b/거리(m) — 2026-11-22 실측. 11-28부터 GEOM 하나로 계산한다(아래)

# 카메라 기하. pitch_up_deg = 광축이 수평보다 **위로** 든 각(도). 11-25 값 9.5°는 그동안 "아래로 9.5°"라고 적었지만 틀린 표기였다 —
#   식(v = cy + f·tan(atan(h/d) + 9.5°))에서 지평선(d→∞)이 347줄, 화면 가운데(240)보다 아래에 오니 광축은 위를 본다(11-28 정정, 값과 계산은 맞았다).
#   11-28 사용자가 카메라를 더 위로 올렸다(서버 쪽 물체 인식이 가구 전체를 못 봄) — 11-25 값은 무효.
#   다시 잴 때까지 pitch_up_deg=None → 거리·바닥 투영을 하지 않는다(틀린 거리보다 "모름"이 낫다).
#   다시 재기: 로봇을 바닥에 두고 `python3 tools/camera_pitch_calibrate.py`(라이다로 잰 물체 거리 + 검출 상자 아래 끝 → 기울기)
#   11-28 12:23 다시 잼: 위로 17.62°(사람 4.3 m 9장, 표준편차 0.22°). 같은 자세 사진 비교 임시값 ≈ 18°와 맞다.
#     이 각도에서 화면 맨 아래 줄 = 바닥 2.9 m, 지평선 = 450줄 → 바닥 거리는 2.9 m~십여 m만 보이고 줄 하나가 수십 cm다
#   그런데 키 170 cm로 확인하니 머리 끝은 10.5°를 냈다 — f=644가 틀렸다(라이다 모서리 맞춤으로 f≈820, 그러면 발끝·머리 끝·문지방이 ≈12.6°로 맞음).
#     → 다시 None. tools/camera_intrinsics_calibrate.py(체스판)로 fx·fy·cx·cy를 재고 기울기를 다시 풀어 함께 넣는다
#   11-28 14:2x 체스판(37장, RMS 0.38 px, fx 흔들림 ±23)으로 렌즈 값을 쟀다: fx 849.5, fy 857.0, cx 316.1, cy 212.3(화각 41.3×31.3°).
#     이 값으로 사람(키 170 cm, 라이다 4.27 m) 발끝 → 14.74°(첫째 섬)·13.87°(둘째 섬), 머리 끝 → 14.65° — f=644 때(16.5 vs 10.5)와 달리 1° 안에서 맞는다 → 14.4°(±0.5).
#     ⚠️ 이 각도에선 바닥 3~10 m가 화면 437~479줄에 몰린다 — 상자 아래 끝이 10 px 흔들리면 4 m가 7 m로 바뀐다. 거리는 라이다로 잴 것
#     'f'는 세로(줄 → 거리)에 쓰는 fy, 'fx'는 가로(열 → 옆 방향). 왜곡(k1 +0.03, k2 +0.37)은 아직 적용하지 않는다(가운데 근처는 1 px 안팎).
#     이 각도에서 화면 맨 아래 줄 = 바닥 약 3.1 m, 지평선 = 437줄
GEOM = {'h': 0.14, 'f': 857.0, 'fx': 849.5, 'pitch_up_deg': 14.4, 'cy': 212.3, 'cx': 316.1,
        'dist': [0.0308, 0.3716, -0.0123, 0.0108, 0.0], 'intrinsics_file': 'docs/calibration/camera_intrinsics_20261128.json'}
# 기울기 이력 — 새 렌즈 값 기준. 11-25(오전 기록 주행까지)는 같은 자세 사진 10쌍의 회전 차이 7.6°와 11-25 바닥 세 점(7.2~8.1°)으로 다시 풀었다
GEOM_HISTORY = [dict(date='2026-11-25', pitch_up_deg=7.3, valid_until='2026-11-28 10:29', file='docs/calibration/camera_geometry_20261125.json',
                     note='옛 표기 9.5°는 f=644 짝 — 새 렌즈 값으로 7.3°(±0.5)'),
                dict(date='2026-11-28', pitch_up_deg=14.4, file='docs/calibration/camera_intrinsics_20261128.json',
                     note='체스판 렌즈 값 + 사람 발끝·머리 끝. 17.62°(f=644 짝)는 무효')]


def geom_ready(geom=None):
    return (geom or GEOM).get('pitch_up_deg') is not None


def row_to_distance(row, height=480, geom=None):
    """화면 줄(물체 상자 아래 끝) → 카메라에서 바닥 접점까지 수평 거리(m). 기울기를 모르면 None.
    식: 수평 아래로 본 각 = atan((줄 − cy)/f) − 위 기울기 → 거리 = h / tan(각). 그 각이 0.5° 이하(지평선 근처·위)면 None."""
    import math
    g = geom or GEOM
    if g.get('pitch_up_deg') is None or row > height:
        return None
    if row >= height - 2:
        return None      # 11-28: 아래 끝이 화면 끝에 잘린 상자 — 실제 접점은 더 가깝고 얼마인지 모른다(17.6°면 2.9 m로 잘못 나왔다)
    down = math.atan((row - g['cy']) / g['f']) - math.radians(g['pitch_up_deg'])
    if down <= math.radians(0.5):
        return None
    return g['h'] / math.tan(down)


def project_floor(x, y, pose, geom=GEOM, x_cam=0.0, width=640, height=480):
    """지도 위 바닥 점(x,y)이 카메라 화면 어디에 찍히나 (u, v). 화면 밖이면 None.
    pose = (로봇 x, 로봇 y, 로봇 yaw°). 카메라는 base_link에서 x_cam만큼 앞, 높이 h, 광축은 위로 pitch_up_deg.
    바닥이 평평하다고 보고 계산한다 — 벽이나 가구 위 점에는 맞지 않는다(그것은 깊이 카메라 몫)."""
    import math
    rx, ry, yaw = pose
    th = math.radians(yaw)
    dx, dy = x - rx, y - ry
    fwd = dx * math.cos(th) + dy * math.sin(th) - x_cam
    left = -dx * math.sin(th) + dy * math.cos(th)
    if fwd <= 0.05 or geom.get('pitch_up_deg') is None:
        return None
    h, f, pitch = geom['h'], geom['f'], math.radians(geom['pitch_up_deg'])   # 광축이 위로 든 각
    alpha = math.atan(h / math.hypot(fwd, left))
    v = geom['cy'] + f * math.tan(alpha + pitch)
    psi = math.atan2(left, fwd)
    # 카메라가 +x를 보면 화면 오른쪽은 로봇의 -y(오른쪽)다 → 로봇 왼쪽(+y)은 화면 왼쪽(u<320). 11-25 실물 확인에서 뒤집혀 있던 것을 고침
    u = geom['cx'] - geom.get('fx', f) * math.tan(psi)
    if not (0 <= u < width and 0 <= v < height):
        return None
    return (round(u), round(v))


def objects_on_floor(dets, height=480, max_m=2.0):
    """검출 상자들 중 '바닥 가까이(아래 끝)에 있고 2 m 안'인 것을 (상자, 거리)로. 낮은 물체 판단의 재료."""
    out = []
    for d in dets:
        dist = row_to_distance(d[3], height)
        if dist is not None and dist <= max_m:
            out.append((d, round(dist, 2)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--still'); ap.add_argument('--test', action='store_true')
    ap.add_argument('--keep', type=int, nargs='*', default=[0], help='찾을 COCO 번호(기본 0=사람). 예: 사람+의자 0 56')
    ap.add_argument('--out', default=os.path.expanduser('~/camera_shots/detect.jpg'))
    a = ap.parse_args()
    import cv2
    d = Detector(keep=tuple(a.keep))
    if a.still:
        fr = cv2.imread(os.path.expanduser(a.still)); t = time.time()
        dets = d.detect(fr); dt = (time.time() - t) * 1000
        print(f'{os.path.basename(a.still)}: {dt:.0f} ms, {len(dets)}개', [f'{COCO[c]} {s:.2f}' for *_, s, c in dets])
        cv2.imwrite(a.out, d.draw(fr.copy(), dets)); print('저장', a.out)
        return 0
    if a.test:
        for _ in range(3):
            d.detect(np.zeros((480, 640, 3), np.uint8))
        t = time.time(); n = 10
        for _ in range(n):
            d.detect(np.zeros((480, 640, 3), np.uint8))
        print(f'추론 {((time.time() - t) / n) * 1000:.0f} ms/장 (416×416, 검은 화면)')
        return 0
    ap.print_help(); return 2


if __name__ == '__main__':
    sys.exit(main())
