#!/usr/bin/env python3
"""ATS 서버 쪽 성과물 수신기. 표준 라이브러리만 쓴다 — 설치할 게 없다.

로봇(`tools/artifact_uploader.py`)이 맵/가방/지표/텔레메트리를 올리면 기체별로
보관하고, 받은 내용의 해시를 돌려준다. 로봇은 **그 해시가 일치할 때만** 로컬에서
치우므로, 이 응답이 곧 "안전하게 지워도 된다"는 계약이다.

설계 메모 (`docs/agent_architecture.md`)
----------------------------------------
- 유닛 3대 규모에 큐 브로커 같은 건 필요 없다. **HTTP + 파일 저장 + 정적 페이지면
  충분하다.** 과설계는 그 자체로 비용이다.
- 받은 내용을 **다시 읽어서** 해시를 계산한다. 보낸 쪽 값을 그대로 되돌려주면
  검증이 아무 의미가 없다(전송 중 손상을 못 잡는다).
- 임시 파일에 받고 검증 후 `os.replace`로 옮긴다. 중간에 끊겨도 반쪽짜리 파일이
  정식 이름으로 남지 않는다.

실행 — 윈도우 (ATS 서버)
------------------------
    python ats_receiver.py --root "C:\\Users\\carro\\projects\\smart-trash-bin\\artifacts" --port 8080

    항상 켜두려면 작업 스케줄러에 등록한다(윈도우엔 systemd가 없다):
      트리거: 시스템 시작 시 / 사용자 로그온 시
      동작:   프로그램  python
              인수      "경로\\ats_receiver.py" --root "...\\artifacts" --port 8080
      설정:   "작업이 실패하면 다시 시작" 켜기

    ⚠️ 방화벽에서 8080 인바운드를 허용해야 로봇이 접속할 수 있다.
       (관리자 PowerShell)
       New-NetFirewallRule -DisplayName "ATS receiver" -Direction Inbound \
         -LocalPort 8080 -Protocol TCP -Action Allow -Profile Private

실행 — 리눅스
------------
    python3 ats_receiver.py --root ~/smart-trash-artifacts --port 8080

확인
----
    curl http://192.168.0.170:8080/           # 보관 현황
    curl http://192.168.0.170:8080/api/list   # JSON
"""

import argparse
import hashlib
import secrets
import html
import json
import os
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

# 로봇이 보내는 이름을 그대로 경로에 쓰면 `../../etc/passwd` 같은 게 가능하다.
# 파일명에 쓸 문자를 화이트리스트로 제한한다.
SAFE = re.compile(r'^[A-Za-z0-9._-]{1,200}$')

# ⚠️ 윈도우 예약 장치명. 이 이름으로 파일을 만들면 실패하거나 장치로 열린다.
# 화이트리스트 정규식만으로는 안 걸러진다("NUL"은 통과한다). 서버가 윈도우일 수
# 있으므로 플랫폼과 무관하게 막는다 — 나중에 서버를 옮겨도 동작이 같아야 한다.
WIN_RESERVED = {'CON', 'PRN', 'AUX', 'NUL'} | \
    {f'COM{i}' for i in range(1, 10)} | {f'LPT{i}' for i in range(1, 10)}
MAX_BYTES = 2 * 1024 * 1024 * 1024      # 2GB 상한 — 사고성 대용량 방지
ROOT = None


def safe_component(value, fallback):
    value = (value or '').strip()
    if not SAFE.match(value) or value in ('.', '..'):
        return fallback
    # 확장자를 뗀 본체가 예약 장치명이면 안 된다 ("NUL", "nul.txt" 둘 다 위험).
    if value.split('.')[0].upper() in WIN_RESERVED:
        return fallback
    # 윈도우는 이름 끝의 점/공백을 조용히 잘라내서 다른 파일과 충돌할 수 있다.
    if value != value.rstrip('. '):
        return fallback
    return value


class Handler(BaseHTTPRequestHandler):
    server_version = 'ATSReceiver/1.0'

    def log_message(self, fmt, *a):
        sys.stderr.write(f'{time.strftime("%H:%M:%S")} {self.address_string()} '
                         f'{fmt % a}\n')

    def _json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ---------------- 업로드 ----------------
    def do_POST(self):
        u = urlparse(self.path)
        if u.path != '/upload':
            return self._json(404, {'error': 'unknown path'})
        q = parse_qs(u.query)

        def one(k, default=''):
            return (q.get(k) or [default])[0]

        unit = safe_component(one('unit'), 'unknown-unit')
        kind = safe_component(one('kind'), 'misc')
        # ⚠️ 대체 이름에 난수를 넣는다. 예전엔 초 단위 시각만 썼는데, 거부된 업로드가
        # 같은 초에 여러 건 오면 **전부 같은 이름이 되어 서로 덮어썼다**(실측 확인).
        # 이름이 거부됐다는 건 이미 뭔가 이상하다는 뜻이라, 그 증거를 잃으면 안 된다.
        name = safe_component(one('name'),
                              f'file-{int(time.time())}-{secrets.token_hex(4)}')
        claimed = one('sha256')

        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            return self._json(400, {'error': 'bad Content-Length'})
        if length <= 0:
            return self._json(400, {'error': 'empty body'})
        if length > MAX_BYTES:
            return self._json(413, {'error': f'too large: {length}'})

        dest_dir = os.path.join(ROOT, unit, kind)
        os.makedirs(dest_dir, exist_ok=True)
        final = os.path.join(dest_dir, name)
        tmp = final + '.part'

        h = hashlib.sha256()
        got = 0
        try:
            with open(tmp, 'wb') as f:
                while got < length:
                    chunk = self.rfile.read(min(1 << 20, length - got))
                    if not chunk:
                        break
                    f.write(chunk)
                    h.update(chunk)
                    got += len(chunk)
        except OSError as e:
            self._cleanup(tmp)
            return self._json(500, {'error': f'write failed: {e}'})

        if got != length:
            self._cleanup(tmp)
            return self._json(400, {'error': f'incomplete: {got}/{length}'})

        digest = h.hexdigest()
        if claimed and claimed != digest:
            # 전송 중 손상. 반쪽 파일을 정식 이름으로 남기지 않는다.
            self._cleanup(tmp)
            return self._json(422, {'error': 'checksum mismatch',
                                    'sha256': digest, 'claimed': claimed})

        os.replace(tmp, final)
        self.log_message('받음 %s/%s/%s (%.2f MB)', unit, kind, name, got / 1e6)
        # 로봇은 이 sha256이 자기 값과 같을 때만 로컬에서 지운다.
        return self._json(200, {'ok': True, 'sha256': digest,
                                'bytes': got, 'stored': f'{unit}/{kind}/{name}'})

    def _cleanup(self, path):
        try:
            os.unlink(path)
        except OSError:
            pass

    # ---------------- 조회 ----------------
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == '/api/list':
            return self._json(200, self._inventory())
        if u.path == '/':
            return self._page()
        return self._json(404, {'error': 'unknown path'})

    def _inventory(self):
        out = {}
        if not os.path.isdir(ROOT):
            return out
        for unit in sorted(os.listdir(ROOT)):
            ud = os.path.join(ROOT, unit)
            if not os.path.isdir(ud):
                continue
            out[unit] = {}
            for kind in sorted(os.listdir(ud)):
                kd = os.path.join(ud, kind)
                if not os.path.isdir(kd):
                    continue
                files = []
                for n in sorted(os.listdir(kd)):
                    p = os.path.join(kd, n)
                    if os.path.isfile(p) and not n.endswith('.part'):
                        st = os.stat(p)
                        files.append({'name': n, 'bytes': st.st_size,
                                      'mtime': st.st_mtime})
                out[unit][kind] = files
        return out

    def _page(self):
        inv = self._inventory()
        rows = []
        for unit, kinds in inv.items():
            total = sum(f['bytes'] for fs in kinds.values() for f in fs)
            cnt = sum(len(fs) for fs in kinds.values())
            rows.append(f'<h2>{html.escape(unit)} '
                        f'<small>{cnt}개 · {total/1e6:.1f} MB</small></h2>')
            for kind, files in kinds.items():
                rows.append(f'<h3>{html.escape(kind)}</h3><table>')
                for f in sorted(files, key=lambda x: -x['mtime'])[:50]:
                    ts = time.strftime('%m-%d %H:%M', time.localtime(f['mtime']))
                    rows.append(f'<tr><td>{html.escape(f["name"])}</td>'
                                f'<td>{f["bytes"]/1e6:.2f} MB</td><td>{ts}</td></tr>')
                rows.append('</table>')
        if not rows:
            rows = ['<p>아직 받은 성과물이 없다.</p>']
        body = f'''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<title>ATS 성과물 보관</title><style>
body{{font-family:system-ui,sans-serif;max-width:900px;margin:2rem auto;padding:0 1rem}}
table{{border-collapse:collapse;width:100%;margin-bottom:1rem}}
td{{border-bottom:1px solid #ddd;padding:.35rem .5rem;font-variant-numeric:tabular-nums}}
small{{color:#666;font-weight:400}} h3{{margin:.6rem 0 .3rem;color:#555;font-size:.95rem}}
</style></head><body><h1>ATS 성과물 보관</h1>{"".join(rows)}</body></html>'''
        raw = body.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def main():
    global ROOT
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.expanduser('~/smart-trash-artifacts'))
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--bind', default='0.0.0.0')
    args = ap.parse_args()

    ROOT = os.path.abspath(os.path.expanduser(args.root))
    os.makedirs(ROOT, exist_ok=True)
    print(f'보관 위치: {ROOT}')
    print(f'수신 대기: http://{args.bind}:{args.port}/')
    ThreadingHTTPServer((args.bind, args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
