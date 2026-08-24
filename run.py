#!/usr/bin/env python3
'''크로스 플랫폼 실행 런처 — Windows/macOS/Linux 공통.

    python run.py [web_operate.py 옵션...]

제어 서버는 표준 라이브러리만 쓴다(외부 패키지 0개). venv도 pip install도 필요 없고,
이 파일이 하는 일은 web_operate.py의 flat import(`from udp_link import ...`)가
동작하도록 sys.path를 맞춰주고 브라우저를 띄우는 것뿐이다.
'''

import runpy
import sys
import threading
import webbrowser
from pathlib import Path

if sys.version_info < (3, 8):
    sys.exit(f"Python 3.8+ required, got {sys.version.split()[0]}")

SRC = Path(__file__).resolve().parent / "python" / "q8bot"
ENTRY = SRC / "web_operate.py"
sys.path.insert(0, str(SRC))

# --port 값을 web_operate와 중복 파싱한다 — 브라우저 URL에만 쓰는 값이라
# argparse를 통째로 가져오는 것보다 이쪽이 싸다.
port = "8080"
argv = sys.argv[1:]
for i, a in enumerate(argv):
    if a == "--port" and i + 1 < len(argv):
        port = argv[i + 1]
    elif a.startswith("--port="):
        port = a.split("=", 1)[1]

if "--no-browser" in argv:
    argv.remove("--no-browser")
else:
    threading.Timer(1.5, webbrowser.open, [f"http://localhost:{port}/"]).start()

sys.argv = [str(ENTRY), *argv]
runpy.run_path(str(ENTRY), run_name="__main__")
