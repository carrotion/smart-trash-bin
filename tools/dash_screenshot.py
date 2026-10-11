#!/usr/bin/env python3
"""대시보드 화면을 헤드리스 브라우저로 찍는다 — 파이에는 브라우저가 없어 화면을 못 본 채 고치던 것을 막는 도구(2026-10-10).

배치가 창 크기에 따라 겹치는지 보려고 여러 크기로 한 번에 찍는다. 요소가 서로 겹치는지도 기계로 검사한다
(--check: 같은 부모 아래 형제 요소의 영역이 겹치거나, 글자가 칸 밖으로 넘치면 목록으로 출력).
스크립트 오류(pageerror)는 --check 없이도 항상 잡아 실패로 친다.

준비(한 번):  sudo apt-get install -y python3.12-venv && python3 -m venv ~/.venvs/shot
              ~/.venvs/shot/bin/pip install playwright && ~/.venvs/shot/bin/playwright install chromium-headless-shell
              sudo ~/.venvs/shot/bin/playwright install-deps chromium
실행:  ~/.venvs/shot/bin/python tools/dash_screenshot.py http://localhost:8769 --out /tmp/shots --sizes 1920x950 1366x700 --check
"""
import argparse, os, sys
from playwright.sync_api import sync_playwright

CHECK_JS = r"""() => {
  const out = [];
  const vis = e => { const r = e.getBoundingClientRect(); return r.width > 1 && r.height > 1 && getComputedStyle(e).visibility !== 'hidden'; };
  // 1) 같은 부모 아래 형제끼리 영역이 겹치는가(절대 배치 요소는 제외)
  for (const p of document.querySelectorAll('body *')) {
    const kids = [...p.children].filter(c => vis(c) && !['absolute', 'fixed'].includes(getComputedStyle(c).position));
    for (let i = 0; i < kids.length; i++) for (let j = i + 1; j < kids.length; j++) {
      const a = kids[i].getBoundingClientRect(), b = kids[j].getBoundingClientRect();
      const w = Math.min(a.right, b.right) - Math.max(a.left, b.left), h = Math.min(a.bottom, b.bottom) - Math.max(a.top, b.top);
      if (w > 2 && h > 2) out.push(`겹침 ${w.toFixed(0)}x${h.toFixed(0)}px: <${kids[i].tagName.toLowerCase()}#${kids[i].id}> "${kids[i].textContent.trim().slice(0, 18)}" ↔ <${kids[j].tagName.toLowerCase()}#${kids[j].id}> "${kids[j].textContent.trim().slice(0, 18)}"`);
    }
  }
  // 2) 글자가 칸 밖으로 넘치거나 잘리는가
  for (const e of document.querySelectorAll('body *')) {
    if (!vis(e) || e.children.length || !e.textContent.trim()) continue;
    if (e.scrollWidth > e.clientWidth + 1 && getComputedStyle(e).display !== 'inline')
      out.push(`잘림: <${e.tagName.toLowerCase()}#${e.id}> "${e.textContent.trim().slice(0, 30)}" (${e.scrollWidth} > ${e.clientWidth}px)`);
  }
  if (document.documentElement.scrollWidth > innerWidth + 1) out.push(`가로 스크롤 생김: ${document.documentElement.scrollWidth} > ${innerWidth}px`);
  out.push(`문서 높이 ${document.documentElement.scrollHeight}px / 창 ${innerHeight}px`);
  return out;
}"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('url')
    ap.add_argument('--out', default='.')
    ap.add_argument('--sizes', nargs='+', default=['1920x950', '1366x700', '1100x800'])
    ap.add_argument('--wait', type=float, default=3.0, help='데이터가 들어와 그려질 때까지 기다리는 시간(초)')
    ap.add_argument('--check', action='store_true', help='겹침·잘림을 기계로 검사해 출력(문제가 있으면 종료 코드 1)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True); bad = 0
    with sync_playwright() as p:
        b = p.chromium.launch()
        for s in a.sizes:
            w, h = (int(x) for x in s.split('x'))
            pg = b.new_page(viewport=dict(width=w, height=h)); errs = []
            # ⚠️ 10-11: 지도 대시보드의 스크립트가 문법 오류로 통째로 죽어 있었는데(문자열 안 줄바꿈) 배치 검사만 해서 못 잡았다
            #   — 화면이 "멀쩡해 보이는 빈 화면"이었다. 페이지 오류가 하나라도 있으면 실패로 친다
            pg.on('pageerror', lambda e: errs.append(str(e)[:200]))
            pg.goto(a.url); pg.wait_for_timeout(int(a.wait * 1000))
            for e in errs:
                print('    스크립트 오류:', e); bad += 1
            f = os.path.join(a.out, f'shot_{s}.png'); pg.screenshot(path=f, full_page=True); print(f)
            if a.check:
                for ln in pg.evaluate(CHECK_JS):
                    print('   ', ln); bad += ln.startswith(('겹침', '잘림', '가로'))
            pg.close()
        b.close()
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
