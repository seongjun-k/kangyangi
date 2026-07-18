'''
Written by yufeng.wu0902@gmail.com

Helper functions for operate.py.
'''

import logging
import sys


class Q8Logger:
    """
    Logging wrapper for Q8bot application.

    Provides consistent logging across all modules with configurable log levels.
    Default level is INFO, use debug mode for detailed output.
    INFO level messages are also mirrored to pygame window if configured.
    """

    _instance = None
    _logger = None
    _pygame_surface = None
    _pygame_font = None
    _message_log = []
    _max_messages = 5

    def __init__(self, debug=False):
        """
        Initialize Q8Logger.

        Args:
            debug: bool, if True sets level to DEBUG, otherwise INFO
        """
        if Q8Logger._logger is None:
            Q8Logger._logger = logging.getLogger('q8bot')
            Q8Logger._logger.setLevel(logging.DEBUG if debug else logging.INFO)

            # Create console handler
            handler = logging.StreamHandler(sys.stdout)
            handler.setLevel(logging.DEBUG if debug else logging.INFO)

            # Create formatter
            formatter = logging.Formatter('%(levelname)s: %(message)s')
            handler.setFormatter(formatter)

            # Add handler
            Q8Logger._logger.addHandler(handler)

    @staticmethod
    def set_pygame_surface(surface, font=None):
        """
        Set pygame surface for rendering INFO messages.

        Args:
            surface: pygame.Surface object to draw messages on
            font: pygame.font.Font object (optional, will create default if None)
        """
        Q8Logger._pygame_surface = surface
        if font is None:
            import pygame
            Q8Logger._pygame_font = pygame.font.Font(None, 24)
        else:
            Q8Logger._pygame_font = font

    @staticmethod
    def _render_to_pygame(msg, level='INFO'):
        """
        Render message to pygame window.

        Args:
            msg: Message string to display
            level: Log level (INFO, WARNING, ERROR)
        """
        if Q8Logger._pygame_surface is None or Q8Logger._pygame_font is None:
            return

        # Add message to log with timestamp
        import time
        timestamp = time.strftime('%H:%M:%S')
        Q8Logger._message_log.append((timestamp, level, msg))

        # Keep only last N messages
        if len(Q8Logger._message_log) > Q8Logger._max_messages:
            Q8Logger._message_log.pop(0)

    @staticmethod
    def render_pygame_messages(x_offset=10, y_offset=10):
        """
        Render all logged messages to pygame surface.
        This should be called in the main game loop.

        Args:
            x_offset, y_offset: 새 레이아웃(상태 패널 내부)에 맞춰 배치 위치 지정 가능.
        """
        if Q8Logger._pygame_surface is None or Q8Logger._pygame_font is None:
            return

        # Define colors
        WHITE = (255, 255, 255)
        YELLOW = (255, 255, 0)
        RED = (255, 0, 0)

        line_height = 22

        # Render each message
        for i, (timestamp, level, msg) in enumerate(Q8Logger._message_log):
            y_pos = y_offset + (i * line_height)

            # Choose color based on level
            if level == 'WARNING':
                color = YELLOW
            elif level == 'ERROR':
                color = RED
            else:
                color = WHITE

            # Render text: [timestamp] level: message
            text = f"[{timestamp}] {level}: {msg}"
            text_surface = Q8Logger._pygame_font.render(text, True, color)
            Q8Logger._pygame_surface.blit(text_surface, (x_offset, y_pos))

    @staticmethod
    def get_logger():
        """Get the logger instance."""
        if Q8Logger._logger is None:
            Q8Logger()
        return Q8Logger._logger

    @staticmethod
    def debug(msg):
        """Log debug message."""
        Q8Logger.get_logger().debug(msg)

    @staticmethod
    def info(msg):
        """Log info message and mirror to pygame window."""
        Q8Logger.get_logger().info(msg)
        Q8Logger._render_to_pygame(msg, 'INFO')

    @staticmethod
    def warning(msg):
        """Log warning message and mirror to pygame window."""
        Q8Logger.get_logger().warning(msg)
        Q8Logger._render_to_pygame(msg, 'WARNING')

    @staticmethod
    def error(msg):
        """Log error message and mirror to pygame window."""
        Q8Logger.get_logger().error(msg)
        Q8Logger._render_to_pygame(msg, 'ERROR')


def draw_camera_view(surface, frame, pos, size):
    """카메라 프레임을 그리거나, 수신 전/실패 시 NO SIGNAL 플레이스홀더 표시."""
    import pygame

    x, y = pos
    w, h = size
    pygame.draw.rect(surface, (0, 0, 0), (x, y, w, h))
    pygame.draw.rect(surface, (70, 70, 75), (x, y, w, h), 1)
    if frame is not None:
        scaled = frame if frame.get_size() == size else pygame.transform.scale(frame, size)
        surface.blit(scaled, (x, y))
    else:
        font = pygame.font.Font(None, 32)
        text = font.render("NO SIGNAL", True, (150, 150, 150))
        surface.blit(text, text.get_rect(center=(x + w // 2, y + h // 2)))


def draw_status_panel(surface, pos, size, status):
    """연결 IP / torque 상태 / 현재 gait / 마지막 송신 seq 표시."""
    import pygame

    x, y = pos
    w, h = size
    pygame.draw.rect(surface, (25, 25, 28), (x, y, w, h))
    pygame.draw.rect(surface, (70, 70, 75), (x, y, w, h), 1)

    font = pygame.font.Font(None, 26)
    torque_text = "Torque: ON" if status['torque_on'] else "Torque: OFF"
    lines = [
        (f"Robot: {status['ip']}", (235, 235, 235)),
        (torque_text, (100, 220, 120) if status['torque_on'] else (220, 90, 90)),
        (f"Gait: {status['gait']}", (235, 235, 235)),
        (f"Last seq: {status['seq']}", (235, 235, 235)),
    ]
    for i, (line, color) in enumerate(lines):
        text = font.render(line, True, color)
        surface.blit(text, (x + 12, y + 12 + i * 30))


def draw_controls_help(surface, pos, size, lines):
    """하단 키 조작 안내 텍스트 표시."""
    import pygame

    x, y = pos
    w, h = size
    pygame.draw.rect(surface, (20, 20, 22), (x, y, w, h))
    pygame.draw.rect(surface, (70, 70, 75), (x, y, w, h), 1)

    font = pygame.font.Font(None, 22)
    for i, line in enumerate(lines):
        text = font.render(line, True, (200, 200, 200))
        surface.blit(text, (x + 12, y + 10 + i * 22))


# XiaoPortFinder(시리얼 COM 포트 탐지)는 UDP 아키텍처 전환으로 불필요해져 제거.

