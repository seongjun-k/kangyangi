'''
operate.py(pygame)를 대체하는 웹 UI 엔트리. 표준 라이브러리만 사용(pygame 불필요).

제어 흐름은 operate.py와 동일: 브라우저 키 입력 -> gait_manager -> q8.move_all(UDP).
pygame 키코드 대신 문자열 키("w","a",...)로 매핑하며 의미는 control_config.py
KEYBOARD_MAPPING과 동일하게 유지한다.
'''

import argparse
import json
import logging
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from kinematics_solver import k_solver
from udp_link import q8_udp
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet
from control_config import JOYSTICK_CONFIG, apply_deadzone, get_joystick_direction
import voice

CENTER_DIST = 19.5
L1 = 25
L2 = 40

SPEED = 200  # 제어 루프 tick rate(Hz), operate.py와 동일
HEARTBEAT_TIMEOUT = 0.5  # 하트비트 끊김 판정(펌웨어 500ms 안전정지와 일관)

VOICE_PORT = 81  # 로봇 오디오 스트림 포트(카메라 MJPEG는 기본 80번 포트)
VOICE_SAMPLE_RATE = 16000
VOICE_MAX_SECONDS = 5  # 최대 녹음 길이
VOICE_MOVE_DURATION = 2.0  # 이동 명령 유지 시간(음성은 순간 명령이므로)

# 부분 일치 명령 테이블: 인식 텍스트에 키워드가 하나라도 포함되면 매칭.
# "가" 단독 키워드는 삭제(거의 모든 문장에 우연히 포함되어 오탐 유발).
VOICE_COMMANDS = [
    (("앞으로", "전진"), "forward"),
    (("뒤로", "후진"), "backward"),
    (("왼쪽",), "left"),
    (("오른쪽",), "right"),
    (("멈춰", "정지"), "stop"),
    (("인사",), "greet"),
    (("앉아", "쉬어"), "torque_off"),
    (("일어나", "준비"), "torque_on"),
]

# jump는 부분 포함이 아닌 단어 단위 정확 일치만 허용(예: "점프해줘"는 매칭, "점프타운"은 매칭 안 됨).
JUMP_WORDS = ("점프", "뛰어")

# 부정어가 포함된 문장은 전체 무시(예: "점프하지 마" -> jump 오발동 방지).
NEGATION_PATTERNS = ("하지마", "하지 마", "안 ", "말고", "말아")


def _has_negation(text):
    return any(p in text for p in NEGATION_PATTERNS) or text.rstrip().endswith(("마", "말"))


def match_voice_command(text):
    '''인식 텍스트 -> (action, ignore_reason). 부정어 감지 시 (None, "negation").
    매칭은 테이블 순서가 아니라 가장 긴 키워드를 우선한다(짧은 키워드의 우연한 부분일치 방지).'''
    if _has_negation(text):
        return None, "negation"

    words = text.split()
    if any(w in JUMP_WORDS for w in words):
        return "jump", None

    best_action, best_len = None, 0
    for keywords, action in VOICE_COMMANDS:
        for kw in keywords:
            if kw in text and len(kw) > best_len:
                best_action, best_len = action, len(kw)
    return best_action, None


class VoiceState:
    '''음성 녹음/인식 상태(HTTP 스레드 <-> 녹음 스레드 공유). 모든 상태 변경은 _lock으로 보호.

    세대 토큰(_generation): start()마다 증가. 녹음 스레드는 자신의 로컬 버퍼에만 쓰고,
    종료 시 자기 세대가 여전히 최신일 때만 voice_state.buffer에 커밋한다 —
    잔존(stale) 스레드가 그 사이 새로 시작된 녹음의 버퍼를 덮어쓰는 것을 방지.'''

    def __init__(self):
        self._lock = threading.Lock()
        self.recording = False
        self.buffer = bytearray()
        self.last_text = ""
        self.last_command = None
        self.last_ignore_reason = None
        self.mic_connected = None  # None=미시도, True/False=최근 연결 결과
        self.stop_event = None
        self.thread = None
        self._generation = 0

    def snapshot(self):
        with self._lock:
            return {
                "voice_recording": self.recording,
                "voice_mic_connected": self.mic_connected,
                "voice_text": self.last_text,
                "voice_command": self.last_command,
                "voice_ignore_reason": self.last_ignore_reason,
            }

    def try_start(self):
        '''락 안 check-then-set. 이미 recording이면 None(호출측에서 409 처리).'''
        with self._lock:
            if self.recording:
                return None
            self.recording = True
            self.buffer = bytearray()
            self.stop_event = threading.Event()
            self._generation += 1
            return self._generation, self.stop_event

    def set_thread(self, thread):
        with self._lock:
            self.thread = thread

    def commit_recording(self, gen, buf):
        '''녹음 스레드가 스스로(5초 상한/연결 실패/스트림 종료) 또는 stop 요청으로 끝날 때 호출.
        gen이 낡았으면(그새 새 녹음이 시작됨) 폐기 — stale 스레드 결과가 새 세션을 오염시키지 않음.'''
        with self._lock:
            if gen != self._generation:
                return
            self.buffer = buf
            self.recording = False

    def stop_and_collect(self):
        '''사용자 stop 요청: 스레드 join 후 버퍼 복사까지 전부 락으로 보호.'''
        with self._lock:
            if not self.recording:
                return None
            stop_event = self.stop_event
            thread = self.thread
        stop_event.set()
        if thread is not None:
            thread.join(timeout=2)
        with self._lock:
            # 스레드가 이미 자체 종료로 commit_recording을 호출했을 수 있음(정상 케이스).
            self.recording = False
            return bytes(self.buffer)


def _record_voice_audio(voice_state, robot_ip, stop_event, gen):
    '''로봇 :81 raw PCM 스트림에서 최대 VOICE_MAX_SECONDS초 버퍼링(로컬 버퍼 사용).
    5초 상한/연결 실패/스트림 종료 등 어떤 이유로 끝나든 commit_recording으로 자체 정리한다.'''
    local_buf = bytearray()
    url = f"http://{robot_ip}:{VOICE_PORT}/"
    try:
        resp = urllib.request.urlopen(url, timeout=3)
    except OSError:
        with voice_state._lock:
            if gen == voice_state._generation:
                voice_state.mic_connected = False
        voice_state.commit_recording(gen, local_buf)
        return
    with voice_state._lock:
        if gen == voice_state._generation:
            voice_state.mic_connected = True
    max_bytes = VOICE_SAMPLE_RATE * 2 * VOICE_MAX_SECONDS  # s16le mono
    try:
        while not stop_event.is_set() and len(local_buf) < max_bytes:
            chunk = resp.read(4096)
            if not chunk:
                break
            local_buf.extend(chunk)
    except OSError:
        pass
    finally:
        resp.close()
    voice_state.commit_recording(gen, local_buf)

WEB_DIR = Path(__file__).parent / "web"

# 웹용 키 매핑: control_config.py KEYBOARD_MAPPING과 동일 의미, 문자열 키 사용
WEB_KEY_MAPPING = {
    'movement': {
        'forward': 'w', 'backward': 's', 'left': 'a', 'right': 'd',
        'forward_left': 'q', 'forward_right': 'e',
    },
    'actions': {
        'greet': 'h', 'battery': 'b', 'switch_gait': 'g', 'jump': 'j',
        'reset': 'r', 'show_range': 'c',
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


class VoiceOverride:
    '''음성 이동 명령의 유효 방향 + 만료 시각(락 보호). control_loop가 매 tick 참조한다.
    브라우저 하트비트(KeyState)와 완전 독립 — 하트비트 만료로 지워지지 않는다.'''

    def __init__(self):
        self._lock = threading.Lock()
        self._direction = None
        self._expires_at = 0.0

    def set(self, direction, duration):
        with self._lock:
            self._direction = direction
            self._expires_at = time.monotonic() + duration

    def clear(self):
        with self._lock:
            self._direction = None

    def get(self):
        with self._lock:
            if self._direction is not None and time.monotonic() < self._expires_at:
                return self._direction
            return None


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


def run_jump(q8, leg, pos_ref, suppress):
    '''jump 실행 + 5초 후 정지 자세 복귀를 별도 스레드에서 수행(control_loop 블로킹 방지).
    suppress로 감싸 control_loop의 gait 송신과 겹치지 않게 한다.'''
    def _run():
        suppress.set()
        try:
            q8.send_jump()
            time.sleep(5)
            q1, q2, _ = leg.ik_solve(pos_ref[0], pos_ref[1], True, 1)
            q8.move_mirror([q1, q2], 500)
        finally:
            suppress.clear()
    threading.Thread(target=_run, daemon=True).start()


def run_greet(q8, leg, pos_ref, suppress, dur=1000):
    def _run():
        suppress.set()
        try:
            greet(q8)
            q1, q2, _ = leg.ik_solve(pos_ref[0], pos_ref[1], True, 1)
            q8.move_mirror([q1, q2], dur)
        finally:
            suppress.clear()
    threading.Thread(target=_run, daemon=True).start()


class RobotState:
    '''SSE 상태 push용 공유 상태(제어 스레드가 갱신, HTTP 스레드가 읽음).'''

    def __init__(self, q8, gait_manager, voice_state):
        self.q8 = q8
        self.gait_manager = gait_manager
        self.voice_state = voice_state
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
        data.update(self.voice_state.snapshot())
        return data


def execute_voice_command(action, q8, leg, pos_ref, voice_override, suppress):
    '''인식된 명령 실행. 이동 명령은 key_state 주입이 아니라 voice_override(방향+만료시각)를
    세팅해 control_loop가 매 tick 참조하도록 한다 — 브라우저 하트비트와 완전 독립.'''

    move_dirs = {"forward": "f", "backward": "b", "left": "l", "right": "r"}
    if action in move_dirs:
        voice_override.set(move_dirs[action], VOICE_MOVE_DURATION)
    elif action == "stop":
        voice_override.clear()
    elif action == "jump":
        run_jump(q8, leg, pos_ref, suppress)
    elif action == "greet":
        run_greet(q8, leg, pos_ref, suppress)
    elif action == "torque_off":
        q8.disable_torque()
    elif action == "torque_on":
        q8.enable_torque()
    # action이 None(무매칭)이면 아무 동작도 하지 않음


def control_loop(key_state, robot_state, q8, leg, gait_manager, gait_names, pos_ref, log, stop_event,
                  voice_override, suppress):
    '''operate.py 메인 루프의 pygame 비의존 로직을 이식: 키 상태 -> gait 갱신 -> UDP 송신.'''

    def move_xy(x, y, dur=0):
        q1, q2, _ = leg.ik_solve(x, y, True, 1)
        q8.move_mirror([q1, q2], dur)
        robot_state.note_send()

    def effective_direction(keys, axes):
        # 사용자 실키/게임패드 입력이 있으면 그쪽 우선, 없을 때만 음성 오버라이드를 사용한다
        # (사용자가 로봇을 직접 조작 중이면 음성 이동이 끼어들지 않도록).
        direction = get_movement_direction(keys, axes)
        if direction is not None:
            return direction
        return voice_override.get()

    movement = False
    tick_interval = 1.0 / SPEED

    while not stop_event.is_set():
        loop_start = time.monotonic()

        if suppress.is_active():
            # jump/greet 실행 중 — gait 프레임 송신을 스킵(패킷 충돌 방지). 하트비트/상태는 별도 경로라 영향 없음.
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, tick_interval - elapsed))
            continue

        keys, axes, buttons = key_state.get()

        if movement:
            direction = effective_direction(keys, axes)
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
            if effective_direction(keys, axes) is not None:
                movement = True
            elif is_action_pressed('reset', keys, buttons):
                log.info("Gait Reset")
                move_xy(pos_ref[0], pos_ref[1], 500)
                time.sleep(0.2)
            elif is_action_pressed('jump', keys, buttons):
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
            elif is_action_pressed('greet', keys, buttons):
                log.info("Greet")
                run_greet(q8, leg, pos_ref, suppress)
                time.sleep(0.2)

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


def make_handler(key_state, robot_state, q8, leg, pos_ref, robot_ip, voice_state, voice_override, suppress):
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
            elif self.path == "/voice/status":
                self._send_json(voice_state.snapshot())
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
                    run_jump(q8, leg, pos_ref, suppress)  # jump/greet/음성과 동일한 suppress 경로로 통일
                else:
                    self._send_json({"error": "unknown cmd"}, 400)
                    return
                self._send_json({"ok": True})
            elif self.path == "/voice":
                action = payload.get("action")
                if action == "start":
                    started = voice_state.try_start()
                    if started is None:
                        self._send_json({"error": "already recording"}, 400)
                        return
                    gen, stop_event = started
                    thread = threading.Thread(
                        target=_record_voice_audio,
                        args=(voice_state, robot_ip, stop_event, gen),
                        daemon=True,
                    )
                    voice_state.set_thread(thread)
                    thread.start()
                    self._send_json({"ok": True})
                elif action == "stop":
                    buf = voice_state.stop_and_collect()
                    if buf is None:
                        self._send_json({"error": "not recording"}, 400)
                        return
                    text = voice.recognize(buf)
                    command, ignore_reason = match_voice_command(text) if text else (None, None)
                    with voice_state._lock:
                        voice_state.last_text = text
                        voice_state.last_command = command
                        voice_state.last_ignore_reason = ignore_reason
                    execute_voice_command(command, q8, leg, pos_ref, voice_override, suppress)
                    self._send_json({"ok": True, "text": text, "command": command, "ignore_reason": ignore_reason})
                else:
                    self._send_json({"error": "unknown action"}, 400)
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
    voice_state = VoiceState()
    voice_override = VoiceOverride()
    suppress = ControlSuppress()
    robot_state = RobotState(q8, gait_manager, voice_state)
    stop_event = threading.Event()

    voice.preload_model_async()  # 첫 PTT 블로킹 방지(백그라운드 선로딩)

    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(key_state, robot_state, q8, leg, gait_manager, gait_names, pos_ref, log, stop_event,
              voice_override, suppress),
        daemon=True,
    )
    ctrl_thread.start()

    handler = make_handler(key_state, robot_state, q8, leg, pos_ref, q8.ip, voice_state, voice_override, suppress)
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
