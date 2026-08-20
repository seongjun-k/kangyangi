#!/usr/bin/env python3
"""시리얼 로그를 타임스탬프 붙여 파일에 남긴다.

pio device monitor는 보드가 리셋되면 USB CDC가 재열거되면서 그대로 죽는다 —
정작 재부팅 원인을 봐야 하는 순간에 로그가 끊긴다. 이 스크립트는 포트가 사라지면
기다렸다 다시 붙고, 끊긴 사실 자체도("PORT LOST") 기록한다. 재부팅 여부는 이 줄과
바로 뒤의 [BOOT] 줄로 판별한다.

표준 라이브러리만 사용한다(pyserial 불필요) — stty로 포트를 설정하고 바이너리로 읽는다.

사용법: python3 tools/seriallog.py [포트] [로그파일]
"""
import glob
import os
import subprocess
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else None
logfile = sys.argv[2] if len(sys.argv) > 2 else "serial.log"
BAUD = "115200"


def find_port():
    if port:
        return port if os.path.exists(port) else None
    found = sorted(glob.glob("/dev/ttyACM*"))
    return found[0] if found else None


def stamp():
    return time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"


def emit(log, text):
    line = f"[{stamp()}] {text}"
    print(line, flush=True)
    log.write(line + "\n")
    log.flush()  # 다음 리셋에 버퍼째 날아가지 않도록 매 줄 flush


def main():
    with open(logfile, "a", buffering=1) as log:
        emit(log, f"=== seriallog 시작 (로그: {logfile}) ===")
        while True:
            dev = find_port()
            if not dev:
                time.sleep(0.2)
                continue
            try:
                # -hupcl: 닫을 때 DTR을 내려 보드를 리셋시키지 않는다
                subprocess.run(
                    ["stty", "-F", dev, BAUD, "raw", "-echo", "-hupcl"], check=True
                )
                emit(log, f"=== 연결됨: {dev} ===")
                with open(dev, "rb", buffering=0) as f:
                    buf = b""
                    while True:
                        chunk = f.read(256)
                        if not chunk:  # EOF = 보드가 사라짐
                            raise OSError("EOF")
                        buf += chunk
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            emit(log, raw.decode("utf-8", "replace").rstrip("\r"))
            except (OSError, subprocess.CalledProcessError) as e:
                emit(log, f"*** PORT LOST ({e}) — 재연결 대기 ***")
                time.sleep(0.2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
