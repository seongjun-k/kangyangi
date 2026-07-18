'''
Written by yufeng.wu0902@gmail.com (원본 espnow.py 기반)

espnow.py(시리얼)를 대체. Q8bot 노트북 제어 측에서 XIAO ESP32-S3(AP 192.168.4.1:8888)로
UDP를 통해 모션/커맨드 패킷을 전송한다. 패킷 포맷은 docs/protocol.md(SSoT)를 따른다.
공개 API(enable_torque, disable_torque, move_all, move_mirror, send_jump 등)는
q8_espnow과 동일하게 유지해 operate.py 등 호출부 수정을 최소화한다.
'''

import json
import socket
import struct
from pathlib import Path

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

ROBOT_IP = "192.168.4.1"
ROBOT_PORT = 8888

# 커맨드 패킷 매직 바이트 (protocol.md)
CMD_MAGIC = 0xFF
CMD_TORQUE_OFF = 0
CMD_TORQUE_ON = 1
CMD_JUMP = 4

# deg -> Dynamixel raw tick(0-8191, extended position mode) 변환 계수.
# 원본 espnow.py의 deg2dxl 로직을 이식했으나 GEAR_RATIO 값은
# control_config.py/helpers.py 어디에도 정의되어 있지 않았음(원본에서도 미사용 dead code).
# ZERO_OFFSET=4096은 펌웨어 q8Dynamixel.h:51 HOMING_OFFSET(_zeroOffset)과 일치(SSoT)이자
# calibration.json 미존재 시(실측 보정 전) 기본값.
GEAR_RATIO = 1.0
ZERO_OFFSET = 4096

# 캘리브레이션 마법사(web_operate.py /calib)가 저장하는 관절별 실측 오프셋 파일.
CALIBRATION_FILE = Path(__file__).parent / "calibration.json"


def load_zero_offsets():
    '''calibration.json이 있으면 관절별(ID 11-18 순서) 실측 오프셋을, 없으면 전 관절 ZERO_OFFSET을 반환.'''
    if CALIBRATION_FILE.exists():
        try:
            data = json.loads(CALIBRATION_FILE.read_text())
            offsets = data.get("zero_offsets")
            if isinstance(offsets, list) and len(offsets) == 8:
                return list(offsets)
        except (json.JSONDecodeError, OSError):
            pass
    return [ZERO_OFFSET] * 8


class q8_udp:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT, joint_list=DEFAULT_JOINTLIST):
        self.ip = ip
        self.port = port
        self.JOINTS = joint_list
        self.torque_on = False
        self.seq = 0
        self.zero_offsets = load_zero_offsets()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _next_seq(self):
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF  # uint16 랩어라운드
        return seq

    def _send(self, payload):
        try:
            self.sock.sendto(payload, (self.ip, self.port))
        except OSError:
            return False
        return True

    def _send_cmd(self, cmd):
        # 커맨드 패킷(3B): magic, cmd, checksum
        body = bytes([CMD_MAGIC, cmd])
        checksum = 0
        for b in body:
            checksum ^= b
        return self._send(body + bytes([checksum]))

    def enable_torque(self):
        self.torque_on = True
        return self._send_cmd(CMD_TORQUE_ON)

    def disable_torque(self):
        self.torque_on = False
        return self._send_cmd(CMD_TORQUE_OFF)

    def send_jump(self):
        return self._send_cmd(CMD_JUMP)

    def move_all(self, joints_pos, dur=0, record=True):
        # Expects 8 positions in deg. dur(프로파일 시간)/record는 새 프로토콜에 없어 무시.
        try:
            ticks = [self.deg2dxl(p, i) for i, p in enumerate(joints_pos)]
        except (struct.error, ValueError):
            return False
        return self.send_raw_ticks(ticks)

    def move_mirror(self, joint_pos, dur=0):
        # Expects a pair of pos for one leg, which will be mirrored 4times.
        mirrored_pos = []
        for i in range(4):
            mirrored_pos.append(joint_pos[0])
            mirrored_pos.append(joint_pos[1])
        return self.move_all(mirrored_pos, dur, False)

    def send_raw_ticks(self, ticks):
        '''이미 계산된 tick 8개(ID 11-18 순서)를 그대로 모션 패킷으로 송신.
        캘리브레이션 마법사가 deg2dxl을 거치지 않고 직접 tick을 보낼 때 사용.'''
        try:
            seq = self._next_seq()
            body = struct.pack("<H8H", seq, *ticks)  # seq(2B) + tick*8(16B)
            checksum = 0
            for b in body:
                checksum ^= b
            self._send(body + bytes([checksum]))
        except struct.error:
            return False
        return True

    def deg2dxl(self, angle_friendly, joint_index=0):
        # joint_index(0-7)로 관절별 실측 zero offset(calibration.json)을 적용.
        friendly_per_dxl = 360.0 / 4096.0 / GEAR_RATIO
        angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5) + self.zero_offsets[joint_index]
        return max(0, min(8191, angle_dxl))  # extended position mode tick 유효범위는 0-8191
