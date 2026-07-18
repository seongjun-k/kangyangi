'''
가짜 펌웨어(mock firmware). 실물 XIAO ESP32-S3 없이 UDP 프로토콜/안전정지 로직을
검증하기 위한 시뮬레이터. 패킷 포맷·안전 규칙은 docs/protocol.md(SSoT)를 그대로 재현한다.
'''

import socket
import struct
import threading
import time

MOTION_LEN = 19
CMD_LEN = 3
CMD_MAGIC = 0xFF
CMD_TORQUE_OFF = 0
CMD_TORQUE_ON = 1
CMD_JUMP = 4
NO_RECV_TIMEOUT = 0.5  # 500ms 무수신 시 torque off (protocol.md 안전 규칙)


def _xor_checksum(data):
    checksum = 0
    for b in data:
        checksum ^= b
    return checksum


class MockRobot:
    '''UDP 8888 수신 -> 패킷 검증 -> 관절/토크 상태 갱신. 스레드로 백그라운드 실행.'''

    def __init__(self, ip="127.0.0.1", port=8888):
        self.ip = ip
        self.port = port
        self.joint_ticks = [2048] * 8  # ID 11-18 순서, ZERO_OFFSET=2048 기본값
        self.torque_on = False
        self.last_seq = None
        self.last_recv_time = None
        self.packet_count = 0
        self.checksum_errors = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.1)  # stop 플래그를 주기적으로 확인하기 위한 타임아웃
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self.sock.close()

    def _run(self):
        while not self._stop.is_set():
            try:
                data, _addr = self.sock.recvfrom(64)
            except socket.timeout:
                self._check_safety_timeout()
                continue
            self._handle_packet(data)
            self._check_safety_timeout()

    def _check_safety_timeout(self):
        # 펌웨어와 동일 규칙: 마지막 수신 후 500ms 지나면 torque off
        with self._lock:
            if self.torque_on and self.last_recv_time is not None:
                if time.monotonic() - self.last_recv_time > NO_RECV_TIMEOUT:
                    self.torque_on = False
                    print("[mock_robot] 500ms 무수신 -> torque off (안전정지)")

    def _handle_packet(self, data):
        n = len(data)
        if n == MOTION_LEN:
            self._handle_motion(data)
        elif n == CMD_LEN:
            self._handle_cmd(data)
        else:
            # 길이 불일치 패킷 폐기 (protocol.md)
            print(f"[mock_robot] 길이 불일치 폐기 (len={n})")
            return

    def _handle_motion(self, data):
        body, checksum = data[:18], data[18]
        if _xor_checksum(body) != checksum:
            self.checksum_errors += 1
            print("[mock_robot] 모션 패킷 체크섬 불일치 -> 폐기")
            return
        seq, *ticks = struct.unpack("<H8H", body)
        with self._lock:
            if self.last_seq is not None and seq == self.last_seq:
                # 중복 패킷 무시 (protocol.md)
                self.last_recv_time = time.monotonic()
                return
            self.last_seq = seq
            self.joint_ticks = ticks
            self.last_recv_time = time.monotonic()
            self.packet_count += 1
        print(f"[mock_robot] motion seq={seq} ticks={ticks}")

    def _handle_cmd(self, data):
        body, checksum = data[:2], data[2]
        if _xor_checksum(body) != checksum:
            self.checksum_errors += 1
            print("[mock_robot] 커맨드 패킷 체크섬 불일치 -> 폐기")
            return
        magic, cmd = body[0], body[1]
        if magic != CMD_MAGIC:
            print("[mock_robot] 커맨드 매직 불일치 -> 폐기")
            return
        with self._lock:
            self.last_recv_time = time.monotonic()
            self.packet_count += 1
            if cmd == CMD_TORQUE_ON:
                self.torque_on = True
            elif cmd == CMD_TORQUE_OFF:
                self.torque_on = False
        print(f"[mock_robot] cmd={cmd} torque_on={self.torque_on}")

    def snapshot(self):
        '''viz/검사용 상태 스냅샷 (스레드 세이프).'''
        with self._lock:
            return list(self.joint_ticks), self.torque_on, self.packet_count, self.checksum_errors


if __name__ == "__main__":
    # 단독 실행: operate.py --ip 127.0.0.1 통합 테스트용 가짜 로봇
    import time
    robot = MockRobot()
    robot.start()
    print("[mock_robot] listening on 0.0.0.0:8888 (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot.stop()
