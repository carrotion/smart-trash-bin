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
