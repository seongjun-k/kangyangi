'''
웹 UI 제어 엔트리. 표준 라이브러리 http.server와 Gamepad API 사용.
브라우저 키 입력/게임패드 입력 -> gait_manager -> q8.move_all(UDP) 흐름.
'''

import argparse
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from kinematics_solver import k_solver
from udp_link import q8_udp, ROBOT_IP
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet, paw

CENTER_DIST = 19.5
L1 = 25
L2 = 40

SPEED = 200  # 제어 루프 tick rate(Hz)
HEARTBEAT_TIMEOUT = 0.5  # 하트비트 끊김 판정(펌웨어 500ms 안전정지와 일관)

WEB_DIR = Path(__file__).parent / "web"

# 웹용 키 매핑: 문자열 키 사용
WEB_KEY_MAPPING = {
    'movement': {
        'forward': 'w', 'backward': 's', 'left': 'a', 'right': 'd',
        'forward_left': 'q', 'forward_right': 'e',
    },
    'actions': {
        'greet': 'h', 'switch_gait': 'g', 'jump': 'j',
        'reset': 'r', 'show_range': 'c', 'paw': 'p',
    },
}

# Xbox 컨트롤러 매핑: 브라우저 Gamepad API의 "standard" 매핑에 따른 버튼 인덱스.
_XBOX_BUTTONS = {"top": 3, "bottom": 0, "left": 2, "right": 1, "side1": 4, "side2": 7}
GAMEPAD_DEADZONE = 0.1
GAMEPAD_ACTIONS = {
    "greet": _XBOX_BUTTONS["top"],
    "battery": _XBOX_BUTTONS["right"],
    "switch_gait": _XBOX_BUTTONS["bottom"],
    "jump": _XBOX_BUTTONS["left"],
    "reset": _XBOX_BUTTONS["side1"],
    "exit": _XBOX_BUTTONS["side2"],
}


def apply_deadzone(value, deadzone):
    '''조이스틱 축 값에 데드존 적용. 임계값 미만이면 0.0.'''
    return 0.0 if abs(value) < deadzone else value


def get_joystick_direction(axis0, axis1):
    '''아날로그 스틱 12방향 해석. 임계값(0.25/0.6/0.85/0.4)과 반환 문자열은
    gait 궤적 키(GAITS)와 직결되므로 절대 변경 금지.

    NOTE: axis0은 좌우, axis1은 전후(음수=전진/좌)
    '''
    if axis0 == 0 and axis1 == 0:
        return None

    abs_a0 = abs(axis0)
    abs_a1 = abs(axis1)

    if abs_a0 >= 0.85 and abs_a1 <= 0.4:
        return 'l' if axis0 < 0 else 'r'

    if abs_a1 > abs_a0:
        base_dir = 'f' if axis1 < 0 else 'b'
        turn_dir = 'l' if axis0 < 0 else 'r'

        if abs_a0 < 0.25:
            return base_dir
        elif abs_a0 < 0.6:
            return f"{base_dir}{turn_dir}_0.75"
        else:
            return f"{base_dir}{turn_dir}_0.5"
    else:
        return 'l' if axis0 < 0 else 'r'


def get_movement_direction(keys, axes=None):
    '''게임패드 아날로그 스틱이 있으면 12방향 매핑을 우선, 입력 없으면 키보드 6방향으로 대체.'''
    if axes:
        x = apply_deadzone(axes.get('x', 0.0), GAMEPAD_DEADZONE)
        y = apply_deadzone(axes.get('y', 0.0), GAMEPAD_DEADZONE)
        direction = get_joystick_direction(x, y)
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


class ControlSuppress:
    '''jump/greet 실행 중 control_loop의 gait 프레임 송신을 스킵시키는 공유 플래그(락 보호).
    두 명령이 겹칠 일은 거의 없지만 순차 실행 대비 카운터로 관리(중첩 set/clear 안전).'''

    def __init__(self):
        self._lock = threading.Lock()
        self._count = 0

    def set(self):
        with self._lock:
            self._count += 1

    def clear(self):
        with self._lock:
            self._count = max(0, self._count - 1)

    def is_active(self):
        with self._lock:
            return self._count > 0


def move_stance(q8, leg, x, y, dur):
    '''IK 계산 후 move_mirror 송신. ik_solve는 실패 시 이전 각도를 반환하므로
    (kinematics_solver.py prev_ik) 실패 프레임은 송신하지 않고 스킵한다.'''
    q1, q2, ok = leg.ik_solve(x, y, True, 1)
    if not ok:
        logging.getLogger("kangyangi").warning(f"IK failed at ({x}, {y}); move skipped")
        return False
    q8.move_mirror([q1, q2], dur)
    return True


def _run_suppressed(q8, leg, pos_ref, suppress, action, dur):
    '''스레드 + suppress.set/clear + action() + move_stance 복귀 공통 구조.
    control_loop 블로킹 방지, suppress로 감싸 control_loop의 gait 송신과 겹치지 않게 한다.'''
    def _run():
        suppress.set()
        try:
            action()
            move_stance(q8, leg, pos_ref[0], pos_ref[1], dur)
        finally:
            suppress.clear()
    threading.Thread(target=_run, daemon=True).start()


def _jump_action(q8):
    q8.send_jump()
    # 펌웨어 jump()가 7.3s 블로킹(q8Dynamixel.cpp) — 그보다 짧으면 jump 후반에 gait 패킷이 겹친다.
    time.sleep(7.5)


def run_jump(q8, leg, pos_ref, suppress):
    _run_suppressed(q8, leg, pos_ref, suppress, lambda: _jump_action(q8), 500)


def run_greet(q8, leg, pos_ref, suppress, dur=1000):
    _run_suppressed(q8, leg, pos_ref, suppress, lambda: greet(q8), dur)


def run_paw(q8, leg, pos_ref, suppress, dur=1000):
    _run_suppressed(q8, leg, pos_ref, suppress, lambda: paw(q8), dur)


class CalibState:
    '''캘리브레이션 모드 상태(락 보호). active 동안 calib_send_loop가 calib_ticks를
    계속 송신한다(500ms 안전정지 워치독 keepalive 겸용 — 모션 패킷 재수신이 곧 torque 유지 조건).'''

    def __init__(self, q8):
        self._lock = threading.Lock()
        self._q8 = q8
        self.active = False
        self.ticks = list(q8.zero_offsets)

    def enter(self):
        '''새로 활성화됐으면 True, 이미 활성 상태였으면 False(중복 enter 시 suppress 카운터가
        남아 보행이 영구 억제되는 것을 호출측에서 막기 위해 반환).'''
        with self._lock:
            was_active = self.active
            self.active = True
            self.ticks = list(self._q8.zero_offsets)
            return not was_active

    def exit(self):
        '''직전까지 활성 상태였으면 True(호출측에서 이때만 suppress.clear() 하도록).'''
        with self._lock:
            was_active = self.active
            self.active = False
            return was_active

    def nudge(self, joint, delta):
        with self._lock:
            if not self.active or not (0 <= joint < 8):
                return None
            self.ticks[joint] = max(0, min(8191, self.ticks[joint] + delta))
            return self.ticks[joint]

    def snapshot(self):
        with self._lock:
            return self.active, list(self.ticks)


def calib_send_loop(calib_state, q8, stop_event):
    '''캘리브레이션 모드 중 calib_ticks를 2~5Hz로 계속 송신하는 스레드.
    모션 패킷 송신 자체가 펌웨어 500ms 무수신 워치독을 만족시키므로 keepalive를 겸한다.'''
    interval = 1.0 / 3  # 3Hz -> 500ms 워치독보다 충분히 촘촘
    while not stop_event.is_set():
        active, ticks = calib_state.snapshot()
        if active:
            q8.send_raw_ticks(ticks)
        time.sleep(interval)


def load_saved_offsets():
    from udp_link import CALIBRATION_FILE
    if CALIBRATION_FILE.exists():
        try:
            data = json.loads(CALIBRATION_FILE.read_text())
            offsets = data.get("zero_offsets")
            if isinstance(offsets, list) and len(offsets) == 8:
                return offsets
        except (json.JSONDecodeError, OSError):
            pass
    return None


def save_calibration(ticks):
    from udp_link import CALIBRATION_FILE
    CALIBRATION_FILE.write_text(json.dumps({"zero_offsets": ticks}))


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
        data = {
            "torque_on": self.q8.torque_on,
            "gait": self.gait_manager.current_gait,
            "seq": self.q8.seq,
            "rate_hz": round(rate, 1),
        }
        return data


def control_loop(key_state, robot_state, q8, leg, gait_manager, gait_names, pos_ref, log, stop_event,
                  suppress):
    '''키 상태 -> gait 갱신 -> UDP 송신 루프.'''

    def move_xy(x, y, dur=0):
        if move_stance(q8, leg, x, y, dur):
            robot_state.note_send()

    movement = False
    prev_jump = prev_greet = prev_paw = False
    tick_interval = 1.0 / SPEED

    while not stop_event.is_set():
        loop_start = time.monotonic()

        if suppress.is_active():
            # jump/greet 실행 중 — gait 프레임 송신을 스킵(패킷 충돌 방지). 하트비트/상태는 별도 경로라 영향 없음.
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, tick_interval - elapsed))
            continue

        keys, axes, buttons = key_state.get()

        # jump/greet/paw는 홀드 시 반복 발동 방지 — 눌리는 순간(rising edge)에만 트리거.
        jump_now = is_action_pressed('jump', keys, buttons)
        greet_now = is_action_pressed('greet', keys, buttons)
        paw_now = is_action_pressed('paw', keys, buttons)
        jump_edge = jump_now and not prev_jump
        greet_edge = greet_now and not prev_greet
        paw_edge = paw_now and not prev_paw
        prev_jump, prev_greet, prev_paw = jump_now, greet_now, paw_now

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
            elif jump_edge:
                log.info("Jump")
                run_jump(q8, leg, pos_ref, suppress)  # 별도 스레드 -> control_loop 블로킹 없음
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
            elif is_action_pressed('show_range', keys, buttons):
                log.info("Show Range")
                show_range(q8)
                time.sleep(0.2)
            elif greet_edge:
                log.info("Greet")
                run_greet(q8, leg, pos_ref, suppress)
            elif paw_edge:
                log.info("Paw")
                run_paw(q8, leg, pos_ref, suppress)

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


def make_handler(key_state, robot_state, q8, leg, pos_ref, robot_ip, suppress,
                  calib_state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass  # 표준 stderr 접근 로그 억제(콘솔 소음 방지)

        def _send_json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_file(self, name):
            body = (WEB_DIR / name).read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self._send_file("index.html")
            elif self.path == "/config":
                self._send_json({"robot_ip": robot_ip})
            elif self.path in ("/calib", "/calib.html"):
                self._send_file("calib.html")
            elif self.path == "/calib/state":
                active, ticks = calib_state.snapshot()
                self._send_json({"active": active, "ticks": ticks, "saved_offsets": load_saved_offsets()})
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

            if self.path == "/calib":
                action = payload.get("action")
                if action == "enter":
                    # enter()가 새로 활성화된 경우에만 suppress.set() - 중복 enter 시 카운터가
                    # 쌓여 exit 한 번으로 해제되지 않고 보행이 영구 억제되는 버그 방지
                    if calib_state.enter():
                        suppress.set()  # gait 프레임 송신 억제(jump/greet와 동일 메커니즘 재사용)
                elif action == "exit":
                    if calib_state.exit():
                        suppress.clear()
                else:
                    self._send_json({"error": "unknown action"}, 400)
                    return
                self._send_json({"ok": True})
            elif self.path == "/calib/nudge":
                joint = payload.get("joint")
                delta = payload.get("delta")
                if not isinstance(joint, int) or not isinstance(delta, int):
                    self._send_json({"error": "bad joint/delta"}, 400)
                    return
                new_tick = calib_state.nudge(joint, delta)
                if new_tick is None:
                    self._send_json({"error": "not in calib mode or bad joint"}, 400)
                    return
                self._send_json({"ok": True, "tick": new_tick})
            elif self.path == "/calib/save":
                active, ticks = calib_state.snapshot()
                if not active:
                    self._send_json({"error": "not in calib mode"}, 400)
                    return
                save_calibration(ticks)
                q8.zero_offsets = list(ticks)  # 저장 즉시 반영, 재시작 없이 적용
                self._send_json({"ok": True, "zero_offsets": ticks})
            elif self.path == "/keys":
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
                    run_jump(q8, leg, pos_ref, suppress)  # jump/greet와 동일한 suppress 경로로 통일
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

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG if args.debug else logging.INFO)
    log = logging.getLogger("kangyangi")

    leg = k_solver(CENTER_DIST, L1, L2, L1, L2)
    q8 = q8_udp(ip=args.ip or ROBOT_IP)
    q8.enable_torque()

    gait_names = list(GAITS.keys())
    gait_manager = GaitManager(leg)

    first_gait_params = GAITS[gait_names[0]]
    pos_ref = [first_gait_params[1], first_gait_params[2]]
    move_stance(q8, leg, pos_ref[0], pos_ref[1], 1000)

    if not gait_manager.load_gait(gait_names[0]):
        log.error(f"Failed to load default gait: {gait_names[0]}")
        return

    time.sleep(2)

    key_state = KeyState()
    suppress = ControlSuppress()
    calib_state = CalibState(q8)
    robot_state = RobotState(q8, gait_manager)
    stop_event = threading.Event()

    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(key_state, robot_state, q8, leg, gait_manager, gait_names, pos_ref, log, stop_event,
              suppress),
        daemon=True,
    )
    ctrl_thread.start()

    calib_thread = threading.Thread(target=calib_send_loop, args=(calib_state, q8, stop_event), daemon=True)
    calib_thread.start()

    handler = make_handler(key_state, robot_state, q8, leg, pos_ref, q8.ip, suppress,
                            calib_state)
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
