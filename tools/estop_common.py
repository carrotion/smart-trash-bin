#!/usr/bin/env python3
"""비상정지 공통 모듈 — 자율주행 프로세스를 죽인다.

⚠️ **여러 대시보드가 이 코드를 공유해야 한다.** 복사해 두면 한쪽만 고쳐져
   어긋나고, 하필 비상정지에서 그러면 위험하다.

왜 "프로세스를 죽이는 것"이 진짜 정지인가
------------------------------------------
cmd_vel에 0을 보내는 것만으로는 부족하다. 자율주행 스택(explore/Nav2)이
돌고 있으면 **다음 주기에 즉시 덮어쓴다.** 명령을 만들어내는 쪽을 멈춰야 한다.
diff_drive_node는 cmd_vel이 0.5초만 끊겨도 스스로 정지하므로(cmd_vel_timeout),
소스를 죽이면 0.5초 안에 반드시 선다.
"""
import os
import signal


# 전체에 대한 부분문자열 매칭이었는데, 그러면 "bt_navigator" 같은 흔한 문자열이
# 들어간 **무관한 프로세스까지 죽인다**. 실제로 진단용 셸(`pgrep -f
# 'nav2_bt_navigator|...'`)이 비상정지에 휘말려 죽는 걸 확인했다 — 실전에서는
# 사용자의 SSH 세션이나 모니터링 명령이 같은 이유로 죽을 수 있어서 위험하다.
AUTONOMOUS_EXECUTABLES = {
    "controller_server",   # Nav2 컨트롤러 — cmd_vel을 직접 발행하는 주범
    "planner_server",
    "bt_navigator",
    "behavior_server",     # 복구 동작(회전/후진)도 cmd_vel을 냄
    "explore",             # explore_lite의 실행파일 이름
}
# 런치 프로세스는 argv[0]이 python이라 실행파일명으로는 못 잡는다 — 인자로 판별.
AUTONOMOUS_LAUNCH_FILE = "autonomous_mapping.launch.py"


def kill_autonomous_processes():
    """자율주행 프로세스를 /proc 직접 스캔 + os.kill로 죽인다. 목록을 반환.

    ⚠️ 예전엔 `subprocess.run(["pkill", "-f", p], capture_output=True)`를 썼는데,
    **죽일 프로세스가 실제로 있을 때만 응답이 안 돌아오는** 현상을 재현 확인했다
    (죽일 게 없으면 0.4초, 있으면 무한 대기). 비상정지 경로에서 서브프로세스와
    파이프에 의존하는 것 자체가 위험하다고 판단해, 외부 프로세스를 아예 안 쓰는
    방식으로 바꿨다. /proc 읽기와 os.kill은 블로킹될 여지가 없다.

    자기 자신(대시보드)은 반드시 제외한다 — 패턴이 자기 명령줄에 걸리면
    비상정지를 누르는 순간 대시보드가 자살한다.
    """
    me = os.getpid()
    killed = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                # cmdline은 인자들이 NUL로 구분돼 있음 — 리스트로 복원
                args = f.read().decode('utf-8', 'replace').split('\0')
        except OSError:
            continue  # 그 사이 종료됐거나 권한 없음
        args = [a for a in args if a]
        if not args:
            continue  # 커널 스레드

        # 파이썬 노드는 argv[0]가 python3이고 실제 이름은 argv[1]에 있다 —
        # 지금 대상은 전부 C++ 바이너리지만, 나중에 파이썬 노드가 대상에
        # 추가돼도 조용히 안 잡히는 일이 없도록 여기서도 같이 처리한다.
        exe = os.path.basename(args[0])
        if exe.startswith('python'):
            for a in args[1:]:
                if not a.startswith('-'):
                    exe = os.path.basename(a)
                    break
        if exe in AUTONOMOUS_EXECUTABLES:
            match = exe
        elif any(a.endswith(AUTONOMOUS_LAUNCH_FILE) for a in args):
            match = AUTONOMOUS_LAUNCH_FILE
        else:
            continue

        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(f'{match}({pid})')
        except OSError:
            pass
    return killed


