#!/usr/bin/env python3
"""로봇이 UDP로 브로드캐스트하는 진단 로그를 받아 타임스탬프와 함께 남긴다.

모터 전원이 켜져 있으면 부스트 레일이 XIAO의 VUSB를 붙들어 USB가 열거되지 않는다
(firmware/kangyangi/include/klog.h 주석 참고). 구동 중 로그를 볼 방법이 이것뿐이다.

한계: 브라운아웃으로 보드가 리셋되면 WiFi 스택이 함께 죽어 직전 몇 줄이 유실될 수
있다. "마지막 줄이 없다"를 근거로 결론내지 말 것 — 재부팅 여부는 로그가 끊긴 뒤
새로 오는 [BOOT] 줄로 판별한다.

사용법: python3 tools/udplog.py [로그파일]   (노트북이 로봇 AP에 접속된 상태여야 함)
"""
import socket
import sys
import time

PORT = 9999  # firmware klog.h의 KLOG_PORT와 반드시 일치
logfile = sys.argv[1] if len(sys.argv) > 1 else "udp.log"


def stamp():
    return time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(1.0)

    with open(logfile, "a", buffering=1) as log:
        def emit(text):
            line = f"[{stamp()}] {text}"
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        emit(f"=== udplog 대기 중 (포트 {PORT}, 로그: {logfile}) ===")
        last = time.time()
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                # 5초 이상 조용하면 알린다 - 로봇이 죽은 건지 그냥 유휴인지
                # 구분이 안 되면 로그를 신뢰할 수 없다. 텔레메트리는 1초 주기다.
                if time.time() - last > 5:
                    emit("--- 5초 이상 수신 없음 ---")
                    last = time.time()
                continue
            last = time.time()
            for raw in data.decode("utf-8", "replace").splitlines():
                if raw.strip():
                    emit(f"{addr[0]} | {raw}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
