"""대시보드 공통 파비콘.
브라우저 탭에 표시할 아이콘. 대시보드가 5개라 각 파일에 따로 넣으면 나중에
하나만 바뀌어 어긋난다. 여기 한 곳만 고치면 전부 반영된다.

도안: 라이다 스캔 호(파랑) + 쓰레기통 뚜껑/몸통(초록) + 바퀴 2개.
      16x16으로 줄어도 초록 몸통 덩어리가 남아 식별된다.

경고: base64로 넣는다. SVG를 그대로 data URI에 쓰면 색상의 # 을 퍼센트
      인코딩해야 하는데, 그 퍼센트 기호가 파이썬 %-포매팅이나 f-string과
      충돌할 수 있다(이 파일을 만들 때 실제로 겪었다). base64는 #, %,
      중괄호가 없어 어떤 문자열 리터럴에 넣어도 안전하다.

사용법:
    from dashboard_icon import FAVICON_TAG
    ... <head> 안에 FAVICON_TAG 삽입
"""

FAVICON_DATA_URI = (
    "data:image/svg+xml;base64,"
    "PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCA2NCA2NCI+PHBhdGggZD0iTTEwIDI0IEEyNCAyNCAwIDAgMSA1NCAyNCIgZmlsbD0ibm9uZSIgc3Ryb2tlPSIjNGVhM2ZmIiBzdHJva2Utd2lkdGg9IjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgb3BhY2l0eT0iLjk1Ii8+PHJlY3QgeD0iMTQiIHk9IjI2IiB3aWR0aD0iMzYiIGhlaWdodD0iNyIgcng9IjMuNSIgZmlsbD0iIzM4NDU1YSIvPjxwYXRoIGQ9Ik0xOCAzNSBoMjggbC0zLjUgMTcgYTMgMyAwIDAgMSAtMyAyLjYgSDI0LjUgYTMgMyAwIDAgMSAtMyAtMi42IHoiIGZpbGw9IiMzYWQwN2YiLz48cGF0aCBkPSJNMjggMzkgdjExIE0zNiAzOSB2MTEiIHN0cm9rZT0iIzJhYTU2MyIgc3Ryb2tlLXdpZHRoPSIyLjYiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPjxjaXJjbGUgY3g9IjIyIiBjeT0iNTgiIHI9IjUuNSIgZmlsbD0iIzM4NDU1YSIvPjxjaXJjbGUgY3g9IjQyIiBjeT0iNTgiIHI9IjUuNSIgZmlsbD0iIzM4NDU1YSIvPjwvc3ZnPg=="
)

FAVICON_TAG = '<link rel="icon" href="' + FAVICON_DATA_URI + '">'
LOGO_IMG_TAG = '<img src="' + FAVICON_DATA_URI + '" width="24" height="24" style="vertical-align:-4px; margin-right:6px;" alt="">'
HOME_ICON_TAG = '<img src="' + FAVICON_DATA_URI + '" width="16" height="16" style="vertical-align:-2px; margin-right:4px;" alt="">'


