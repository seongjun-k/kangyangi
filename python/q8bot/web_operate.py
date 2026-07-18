'''
operate.py(pygame)를 대체하는 웹 UI 엔트리. 표준 라이브러리만 사용(pygame 불필요).

제어 흐름은 operate.py와 동일: 브라우저 키 입력 -> gait_manager -> q8.move_all(UDP).
pygame 키코드 대신 문자열 키("w","a",...)로 매핑하며 의미는 control_config.py
KEYBOARD_MAPPING과 동일하게 유지한다.
'''

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from kinematics_solver import k_solver
from udp_link import q8_udp
from helpers import Q8Logger
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet
from control_config import JOYSTICK_CONFIG, apply_deadzone, get_joystick_direction

CENTER_DIST = 19.5
L1 = 25
L2 = 40

SPEED = 200  # 제어 루프 tick rate(Hz), operate.py와 동일
HEARTBEAT_TIMEOUT = 0.5  # 하트비트 끊김 판정(펌웨어 500ms 안전정지와 일관)

WEB_DIR = Path(__file__).parent / "web"

# 웹용 키 매핑: control_config.py KEYBOARD_MAPPING과 동일 의미, 문자열 키 사용
WEB_KEY_MAPPING = {
    'movement': {
        'forward': 'w', 'backward': 's', 'left': 'a', 'right': 'd',
        'forward_left': 'q', 'forward_right': 'e',
    },
    'actions': {
        'greet': 'h', 'battery': 'b', 'switch_gait': 'g', 'jump': 'j',
        'reset': 'r', 'record': 'z', 'show_range': 'c',
    },
}

# Xbox 컨트롤러 게임패드 매핑: control_config.py JOYSTICK_CONFIG의 "Xbox Series X Controller"
# 항목을 그대로 사용(SSoT). 브라우저 Gamepad API는 Xbox 컨트롤러를 "standard" 매핑으로 노출하므로
# 버튼 인덱스가 pygame 버전의 physical position 해석과 동일하게 맞아떨어진다.
_XBOX_BUTTONS = JOYSTICK_CONFIG['controllers']['Xbox Series X Controller']['buttons']
GAMEPAD_DEADZONE = JOYSTICK_CONFIG['controllers']['Xbox Series X Controller']['axes']['deadzone']
GAMEPAD_ACTIONS = {
    action: _XBOX_BUTTONS[position]
    for action, position in JOYSTICK_CONFIG['action_mapping'].items()
    if position in _XBOX_BUTTONS
}


def get_movement_direction(keys, axes=None):
    '''input_handler.py InputHandler.get_movement_direction()의 의미를 보존:
    게임패드 아날로그 스틱이 있으면 12방향 아날로그 매핑을 우선 사용하고,
    입력이 없으면(또는 게임패드 미연결) 키보드 6방향 매핑으로 대체한다.'''
    if axes:
        x = apply_deadzone(axes.get('x', 0.0), GAMEPAD_DEADZONE)
        y = apply_deadzone(axes.get('y', 0.0), GAMEPAD_DEADZONE)
        direction = get_joystick_direction(x, y, analog_mode=True)
        if direction:
            return direction

    m = WEB_KEY_MAPPING['movement']
    if m['forward'] in keys:
        return 'f'
    if m['backward'] in keys:
        return 'b'
    if m['left'] in keys:
        return 'l'
    if m['right'] in keys:
        return 'r'
    if m['forward_left'] in keys:
        return 'fl_0.75'
    if m['forward_right'] in keys:
        return 'fr_0.75'
    return None


def is_action_pressed(action_name, keys, buttons):
    '''키보드 키 또는 게임패드 버튼 중 하나라도 눌려있으면 액션 발동(마지막 입력 우선이 아닌 OR 조건 — 단순함 우선).'''
    action_names = WEB_KEY_MAPPING['actions']
    if action_name in action_names and action_names[action_name] in keys:
        return True
    return GAMEPAD_ACTIONS.get(action_name) in buttons


class KeyState:
    '''브라우저가 POST /keys로 보낸 눌린 키/게임패드 상태를 보관. 하트비트 끊기면 전체 해제.'''

    def __init__(self):
        self._lock = threading.Lock()
        self._keys = set()
        self._axes = None      # {"x":..,"y":..} 또는 None(게임패드 미연결)
        self._buttons = set()
        self._last_update = time.monotonic()

    def update(self, keys, axes=None, buttons=None):
        with self._lock:
            self._keys = set(keys)
            self._axes = axes
            self._buttons = set(buttons) if buttons else set()
            self._last_update = time.monotonic()

    def get(self):
        with self._lock:
            if time.monotonic() - self._last_update > HEARTBEAT_TIMEOUT:
                return set(), None, set()
            return set(self._keys), self._axes, set(self._buttons)


class RobotState:
    '''SSE 상태 push용 공유 상태(제어 스레드가 갱신, HTTP 스레드가 읽음).'''

    def __init__(self, q8, gait_manager):
        self.q8 = q8
        self.gait_manager = gait_manager
        self._lock = threading.Lock()
        self._send_count = 0
        self._rate = 0.0

    def note_send(self):
        with self._lock:
            self._send_count += 1

    def compute_rate(self, dt):
        with self._lock:
            self._rate = self._send_count / dt if dt > 0 else 0.0
            self._send_count = 0

    def snapshot(self):
        with self._lock:
            rate = self._rate
        return {
            "torque_on": self.q8.torque_on,
            "gait": self.gait_manager.current_gait,
            "seq": self.q8.seq,
            "rate_hz": round(rate, 1),
        }


def control_loop(key_state, robot_state, q8, leg, gait_manager, gait_names, pos_ref, log, stop_event):
    '''operate.py 메인 루프의 pygame 비의존 로직을 이식: 키 상태 -> gait 갱신 -> UDP 송신.'''

    def move_xy(x, y, dur=0):
        q1, q2, _ = leg.ik_solve(x, y, True, 1)
        q8.move_mirror([q1, q2], dur)
        robot_state.note_send()

    movement = False
    tick_interval = 1.0 / SPEED

    while not stop_event.is_set():
        loop_start = time.monotonic()
        keys, axes, buttons = key_state.get()

        if movement:
            direction = get_movement_direction(keys, axes)
            if direction:
                if gait_manager.start_movement(direction):
                    pos = gait_manager.tick()
                    if pos:
                        q8.move_all(pos, 0, False)
                        robot_state.note_send()
                else:
                    movement = False
            else:
                move_xy(pos_ref[0], pos_ref[1], 0)
                gait_manager.stop()
                movement = False
        else:
            if get_movement_direction(keys, axes) is not None:
                movement = True
            elif is_action_pressed('reset', keys, buttons):
                log.info("Gait Reset")
                move_xy(pos_ref[0], pos_ref[1], 500)
                time.sleep(0.2)
            elif is_action_pressed('jump', keys, buttons):
                log.info("Jump")
                q8.send_jump()
                time.sleep(5)
                move_xy(pos_ref[0], pos_ref[1], 500)
            elif is_action_pressed('switch_gait', keys, buttons):
                gait_names.append(gait_names.pop(0))
                new_gait = gait_names[0]
                if gait_manager.load_gait(new_gait):
                    pos_ref[0], pos_ref[1] = GAITS[new_gait][1], GAITS[new_gait][2]
                    move_xy(pos_ref[0], pos_ref[1], 500)
                    log.info(f"Switched to {new_gait}")
                else:
                    log.error(f"Failed to load gait: {new_gait}")
                    gait_names.insert(0, gait_names.pop())
                time.sleep(0.2)
            elif is_action_pressed('record', keys, buttons):
                log.debug("Record next movement")
                time.sleep(0.2)
            elif is_action_pressed('show_range', keys, buttons):
                log.info("Show Range")
                show_range(q8)
                time.sleep(0.2)
            elif is_action_pressed('greet', keys, buttons):
                log.info("Greet")
                greet(q8)
                move_xy(pos_ref[0], pos_ref[1], 1000)
                time.sleep(0.2)
            # battery: no-op (udp_link.check_battery)

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, tick_interval - elapsed))


def status_stream_body(robot_state):
    '''SSE 이벤트 제너레이터: 5Hz push.'''
    interval = 0.2
    while True:
        time.sleep(interval)
        robot_state.compute_rate(interval)
        data = json.dumps(robot_state.snapshot())
        yield f"data: {data}\n\n".encode()


def make_handler(key_state, robot_state, q8, leg, pos_ref, robot_ip):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # 표준 stderr 접근 로그 억제(콘솔 소음 방지)

        def _send_json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                index = (WEB_DIR / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(index)))
                self.end_headers()
                self.wfile.write(index)
            elif self.path == "/config":
                self._send_json({"robot_ip": robot_ip})
            elif self.path == "/status":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    for chunk in status_stream_body(robot_state):
                        self.wfile.write(chunk)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                self._send_json({"error": "bad json"}, 400)
                return

            if self.path == "/keys":
                keys = payload.get("keys", [])
                axes = payload.get("axes")  # 하위호환: 게임패드 미연결 시 없음
                buttons = payload.get("buttons", [])
                key_state.update(keys, axes, buttons)
                self._send_json({"ok": True})
            elif self.path == "/cmd":
                cmd = payload.get("cmd")
                if cmd == "torque_on":
                    q8.enable_torque()
                elif cmd == "torque_off":
                    q8.disable_torque()
                elif cmd == "jump":
                    q8.send_jump()
                    # 5초 후 정지 자세로 복귀(operate.py send_jump 처리와 동일)
                    def _settle():
                        time.sleep(5)
                        q1, q2, _ = leg.ik_solve(pos_ref[0], pos_ref[1], True, 1)
                        q8.move_mirror([q1, q2], 500)
                    threading.Thread(target=_settle, daemon=True).start()
                else:
                    self._send_json({"error": "unknown cmd"}, 400)
                    return
                self._send_json({"ok": True})
            else:
                self.send_error(404)

    return Handler


def main():
    parser = argparse.ArgumentParser(description='Q8bot web control server')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    parser.add_argument('--ip', default=None, help='Robot IP override (e.g. 127.0.0.1 for mock_robot)')
    parser.add_argument('--port', type=int, default=8080, help='Web server port')
    args = parser.parse_args()

    log = Q8Logger(debug=args.debug)

    leg = k_solver(CENTER_DIST, L1, L2, L1, L2)
    q8 = q8_udp(ip=args.ip) if args.ip else q8_udp()
    q8.enable_torque()

    gait_names = list(GAITS.keys())
    gait_manager = GaitManager(leg, GAITS)

    first_gait_params = GAITS[gait_names[0]]
    pos_ref = [first_gait_params[1], first_gait_params[2]]
    q1, q2, _ = leg.ik_solve(pos_ref[0], pos_ref[1], True, 1)
    q8.move_mirror([q1, q2], 1000)

    if not gait_manager.load_gait(gait_names[0]):
        log.error(f"Failed to load default gait: {gait_names[0]}")
        return

    time.sleep(2)

    key_state = KeyState()
    robot_state = RobotState(q8, gait_manager)
    stop_event = threading.Event()

    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(key_state, robot_state, q8, leg, gait_manager, gait_names, pos_ref, log, stop_event),
        daemon=True,
    )
    ctrl_thread.start()

    handler = make_handler(key_state, robot_state, q8, leg, pos_ref, q8.ip)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    log.info(f"Web UI: http://0.0.0.0:{args.port}/  (robot={q8.ip}:{q8.port})")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        q8.disable_torque()
        server.shutdown()


if __name__ == "__main__":
    main()
