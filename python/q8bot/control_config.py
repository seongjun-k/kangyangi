'''
Written by yufeng.wu0902@gmail.com

Control mappings configuration for Q8bot.
Defines keyboard and joystick control schemes using a two-layer mapping system.
'''

# 키보드 매핑은 pygame GUI(operate.py) 삭제와 함께 제거됨.
# 웹 UI의 문자열 키 매핑은 web_operate.py WEB_KEY_MAPPING 참조(동일 의미 유지).

# =============================================================================
# JOYSTICK CONFIGURATION
# =============================================================================

# Joystick configuration embedded as Python dictionary
# This configuration defines controller mappings and action bindings
JOYSTICK_CONFIG = {
    "controllers": {
        "Nintendo Switch Joy-Con (R)": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.2
            },
            "buttons": {
                "top": 2,
                "bottom": 1,
                "left": 3,
                "right": 0,
                "side1": 16,
                "side2": 18
            }
        },
        "Controller for PC": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.1
            },
            "buttons": {
                "top": 0,
                "bottom": 2,
                "left": 3,
                "right": 1,
                "side1": 4,
                "side2": 9
            }
        },
        "Xbox Series X Controller": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.1
            },
            "buttons": {
                "top": 3,
                "bottom": 0,
                "left": 2,
                "right": 1,
                "side1": 4,
                "side2": 7
            }
        },
        "Generic Controller": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.2
            },
            "buttons": {
                "top": 0,
                "bottom": 1,
                "left": 2,
                "right": 3,
                "side1": 4,
                "side2": 5
            }
        }
    },
    "requirements": {
        "min_axes": 2,
        "min_buttons": 6
    },
    "action_mapping": {
        "greet": "top",
        "battery": "right",
        "switch_gait": "bottom",
        "jump": "left",
        "reset": "side1",
        "exit": "side2"
    }
}

# Movement control settings (universal for all joysticks)
JOYSTICK_MOVEMENT = {
    'forward_backward_axis': 1,      # axis[1]: -1 = forward, +1 = backward
    'left_right_axis': 0,             # axis[0]: -1 = left, +1 = right
    'analog_mode': True,              # Analog stride control enabled
}

# =============================================================================
# JOYSTICK HELPER FUNCTIONS
# =============================================================================

def apply_deadzone(value, deadzone):
    """
    Apply deadzone to joystick axis value.

    Args:
        value: float, axis value
        deadzone: float, deadzone threshold

    Returns:
        float: 0.0 if within deadzone, otherwise original value
    """
    return 0.0 if abs(value) < deadzone else value


def get_joystick_direction(axis0, axis1, analog_mode=False):
    """
    Determine movement direction from joystick axes.
    Returns recognized command strings matching trajectory names.

    NOTE: axis0 is left/right, axis1 is forward/backward

    Mapping scheme (analog mode):
    - Straight: abs(a[0]) < 0.25 → 'f' or 'b'
    - Moderate turn: 0.25 <= abs(a[0]) < 0.6 → 'fl_0.75', 'fr_0.75', 'bl_0.75', 'br_0.75'
    - Strong turn: 0.6 <= abs(a[0]) < 0.85 → 'fl_0.5', 'fr_0.5', 'bl_0.5', 'br_0.5'
    - Full left/right: abs(a[0]) >= 0.85 and abs(a[1]) <= 0.4 → 'l', 'r'

    Args:
        axis0: float, left/right axis value (negative = left, positive = right)
        axis1: float, forward/backward axis value (negative = forward, positive = backward)
        analog_mode: bool, if True returns command strings, else simple directions

    Returns:
        str: Command string ('f', 'b', 'l', 'r', 'fl_0.75', 'fl_0.5', etc.) or None
    """
    if not analog_mode:
        # Binary mode: simple 4-direction control
        if axis1 < 0:
            return 'f'  # Forward
        elif axis1 > 0:
            return 'b'  # Backward
        elif axis0 < 0:
            return 'l'  # Left turn
        elif axis0 > 0:
            return 'r'  # Right turn
        else:
            return None
    else:
        # Analog mode: returns command strings matching trajectory names
        # Check for no input
        if axis0 == 0 and axis1 == 0:
            return None

        abs_a0 = abs(axis0)
        abs_a1 = abs(axis1)

        # Full turn mode: high horizontal, low vertical (tightened window)
        if abs_a0 >= 0.85 and abs_a1 <= 0.4:
            # Only very extreme positions are pure left/right
            return 'l' if axis0 < 0 else 'r'

        # Forward/backward with turning
        if abs_a1 > abs_a0:  # Forward/backward dominant
            base_dir = 'f' if axis1 < 0 else 'b'
            turn_dir = 'l' if axis0 < 0 else 'r'

            # Determine turn intensity (expanded 0.5 window)
            if abs_a0 < 0.25:  # Straight movement
                return base_dir
            elif abs_a0 < 0.6:  # Moderate turn (75% stride)
                return f"{base_dir}{turn_dir}_0.75"
            else:  # Strong turn (50% stride) - now abs_a0 >= 0.6
                return f"{base_dir}{turn_dir}_0.5"
        else:
            # Turning is dominant - use full turn mode
            return 'l' if axis0 < 0 else 'r'
