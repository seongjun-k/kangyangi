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
    """

    _instance = None
    _logger = None

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
        """Log info message."""
        Q8Logger.get_logger().info(msg)

    @staticmethod
    def warning(msg):
        """Log warning message."""
        Q8Logger.get_logger().warning(msg)

    @staticmethod
    def error(msg):
        """Log error message."""
        Q8Logger.get_logger().error(msg)


# XiaoPortFinder(시리얼 COM 포트 탐지)는 UDP 아키텍처 전환으로 불필요해져 제거.
# pygame GUI(operate.py) 삭제와 함께 draw_camera_view/draw_status_panel/draw_controls_help도 제거.

