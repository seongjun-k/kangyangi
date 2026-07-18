'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot. Run this script on your laptop,
connected to the XIAO ESP32-S3 AP(192.168.4.1) over WiFi, to control the robot
via keyboard/joystick.
'''

import time
import pygame
import sys
import argparse
from kinematics_solver import k_solver
from udp_link import q8_udp
from helpers import Q8Logger
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet
from input_handler import InputHandler, detect_and_init_joystick
from camera_client import CameraClient
from helpers import draw_camera_view, draw_status_panel, draw_controls_help

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

# Pygame config
SPEED = 200
res = 0.2

# 레이아웃(다크 배경 GUI): 좌측 카메라 320x240, 우측 상태 패널, 하단 조작 안내
WINDOW_SIZE = (900, 420)
CAM_POS, CAM_SIZE = (10, 10), (320, 240)
PANEL_POS, PANEL_SIZE = (340, 10), (550, 240)
CONTROLS_POS, CONTROLS_SIZE = (10, 260), (880, 150)
BG_COLOR = (15, 15, 17)


def main():
    # Helper Functions
    def move_xy(x, y, dur=0, deg=True):
        """Move robot legs to specific x,y position."""
        q1, q2, success = leg.ik_solve(x, y, deg, 1)
        q8.move_mirror([q1, q2], dur)
        return success

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Q8bot control script')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()

    # Initialize logger
    log = Q8Logger(debug=args.debug)

    # Flags for main loop
    movement = False
    record = False
    request = "none"

    # Start pygame instance
    pygame.init()
    window = pygame.display.set_mode(WINDOW_SIZE)
    clock = pygame.time.Clock()

    # Set up pygame surface for logger
    Q8Logger.set_pygame_surface(window)

    # Detect and initialize input device (joystick or keyboard)
    use_joystick, joystick, joystick_mapping = detect_and_init_joystick()
    input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

    # 카메라 라이브 뷰(별도 스레드, GUI 블로킹 없음)
    camera = CameraClient()

    # 하단 조작 안내: 실제 키 바인딩(control_config.py KEYBOARD_MAPPING)과 일치시킴
    if use_joystick:
        controls_text = [
            "Joystick: Move stick to walk (12-direction analog)",
            "Buttons: mapped per controller (see control_config.py JOYSTICK_CONFIG)",
        ]
    else:
        controls_text = [
            "Move: W/A/S/D   Turn: Q(fwd-left) / E(fwd-right)",
            "H: Greet   B: Battery   G: Switch gait   J: Jump   R: Reset   Z: Record   C: Show range",
            "ESC: Exit",
        ]

    # Initialize kinamatics solver and Q8bot UDP link
    leg = k_solver(CENTER_DIST, L1, L2, L1, L2)
    q8 = q8_udp()
    q8.enable_torque()

    # Initialize GaitManager
    gait_names = list(GAITS.keys())
    gait_manager = GaitManager(leg, GAITS)

    # Starting location of leg end effector in x and y
    first_gait_params = GAITS[gait_names[0]]
    pos_x = first_gait_params[1]
    pos_y = first_gait_params[2]
    move_xy(pos_x, pos_y, 1000)

    # Pre-calculate trajectories for default gait
    if not gait_manager.load_gait(gait_names[0]):
        log.error(f"Failed to load default gait: {gait_names[0]}")
        sys.exit(1)

    time.sleep(2)

    while True:
        clock.tick(SPEED)
        events = pygame.event.get()
        if any(e.type == pygame.QUIT for e in events):
            break

        # 화면 그리기: 좌측 카메라 / 우측 상태 패널 / 하단 조작 안내
        window.fill(BG_COLOR)

        draw_camera_view(window, camera.get_frame(), CAM_POS, CAM_SIZE)

        status = {
            "ip": f"{q8.ip}:{q8.port}",
            "torque_on": q8.torque_on,
            "gait": gait_manager.current_gait,
            "seq": q8.seq,
        }
        draw_status_panel(window, PANEL_POS, PANEL_SIZE, status)
        # 상태 패널 하단에 로그 메시지 표시(상태 라인 4개 아래)
        Q8Logger.render_pygame_messages(PANEL_POS[0] + 12, PANEL_POS[1] + 140)

        draw_controls_help(window, CONTROLS_POS, CONTROLS_SIZE, controls_text)

        pygame.display.flip()

        if movement:
            # Get requested direction from input handler
            requested_direction = input_handler.get_movement_direction()

            if requested_direction:
                # Start or switch movement direction
                if gait_manager.start_movement(requested_direction):
                    # Execute current trajectory
                    pos = gait_manager.tick()
                    if pos:
                        q8.move_all(pos, 0, record)
                else:
                    # Failed to start movement
                    movement = False
            else:
                # No movement input - transition to idle
                move_xy(pos_x, pos_y, 0)
                q8.finish_recording()
                record = False
                gait_manager.stop()
                movement = False
        else:
            # Check for movement input
            if input_handler.is_movement_input():
                movement = True
            # Check action inputs using generalized interface
            elif input_handler.is_action_pressed('reset'):
                log.info("Gait Reset")
                move_xy(pos_x, pos_y, 500)
                time.sleep(0.2)
            elif input_handler.is_action_pressed('jump'):
                log.info("Jump")
                q8.send_jump()
                time.sleep(5)
                move_xy(pos_x, pos_y, 500)
            elif input_handler.is_action_pressed('switch_gait'):
                # Cycle to next gait
                gait_names.append(gait_names.pop(0))
                new_gait = gait_names[0]

                # Load new gait
                if gait_manager.load_gait(new_gait):
                    # Update position to match new gait
                    pos_x, pos_y = GAITS[new_gait][1], GAITS[new_gait][2]
                    move_xy(pos_x, pos_y, 500)
                    log.info(f"Switched to {new_gait}")
                else:
                    log.error(f"Failed to load gait: {new_gait}")
                    gait_names.insert(0, gait_names.pop())  # Revert gait change
                time.sleep(0.2)
            elif input_handler.is_action_pressed('battery'):
                q8.check_battery()
                request = "battery"
                time.sleep(0.2)
            elif input_handler.is_action_pressed('record'):
                log.debug("Record next movement")
                record = True
                request = "data"
                time.sleep(0.2)
            elif input_handler.is_action_pressed('show_range'):
                log.info("Show Range")
                show_range(q8)
                time.sleep(0.2)
            elif input_handler.is_action_pressed('greet'):
                log.info("Greet")
                greet(q8)
                move_xy(pos_x, pos_y, 1000)  # Return to rest position
                time.sleep(0.2)
            elif input_handler.is_action_pressed('exit'):
                break
            else:
                # 배터리/기록 응답 수신(시리얼 readline)은 UDP 전환 및 no-op화로 제거.
                request = "none"

    q8.disable_torque()
    camera.stop()
    if joystick:
        joystick.quit()
    pygame.quit()


if __name__ == "__main__":
    main()
