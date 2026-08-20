'''
Written by yufeng.wu0902@gmail.com

Gait management module for Q8bot.
This module manages gait trajectories, movement state, and direction switching
for cyclic locomotion patterns.
'''

from gait_generator import generate_gait_trajectories, generate_crawl_trajectories


# Gait parameters dictionary
# Format: 'NAME': [STACKTYPE, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]
GAITS = {
    'TROT':      ['trot', 9.75, 43.36, 40, 20, 0, 15, 30],
    'TROT_HIGH': ['trot', 9.75, 60, 20, 10, 0, 15, 30],
    'TROT_LOW':  ['trot', 9.75, 25, 20, 10, 0, 15, 30],
    'TROT_FAST': ['trot', 9.75, 43.36, 50, 20, 0, 12, 24],
    'WALK':      ['walk', 9.75, 43.36, 30, 20, 0, 20, 140],
    'CRAWL':     ['crawl', 9.75, 40, 10, 20, 0, 50, 25],
    'BOUND':     ['bound', 9.75, 33.36, 40, 0, 20, 50, 10],
    'PRONK':     ['pronk', 9.75, 33.36, 40, 0, 20, 60, 10],
}


class GaitManager:
    """
    Manages gait trajectories, movement state, and direction switching.

    This class encapsulates all state related to cyclic locomotion:
    - Pre-calculated trajectory storage
    - Phase tracking across direction changes
    - Fallback logic for limited movement types
    - Movement state management
    """

    # Fallback mapping for gaits with limited movement types
    FALLBACK_MAP = {
        'fl_0.5': ['fl_0.75', 'fl', 'f'],
        'fl_0.75': ['fl', 'f'],
        'fr_0.5': ['fr_0.75', 'fr', 'f'],
        'fr_0.75': ['fr', 'f'],
        'bl_0.5': ['bl_0.75', 'bl', 'b'],
        'bl_0.75': ['bl', 'b'],
        'br_0.5': ['br_0.75', 'br', 'b'],
        'br_0.75': ['br', 'b'],
        'fl': ['f'],
        'fr': ['f'],
        'bl': ['b'],
        'br': ['b'],
    }

    # stacktype -> 생성기. trot/walk/bound/pronk는 gait_generator의 테이블 주도 공용 생성기 하나로 처리된다.
    GENERATORS = {
        'trot': generate_gait_trajectories,
        'walk': generate_gait_trajectories,
        'bound': generate_gait_trajectories,
        'pronk': generate_gait_trajectories,
        'crawl': generate_crawl_trajectories,
    }

    def __init__(self, leg):
        """
        Initialize the GaitManager.

        Args:
            leg: Kinematics solver instance
        """
        self.leg = leg
        self.current_trajectories = None
        self.current_gait = None
        self.current_direction = None
        self.phase_index = 0
        self.ongoing = False
        self.current_trajectory = None

    def load_gait(self, gait_name):
        """
        Pre-calculate and load trajectories for a given gait.

        Args:
            gait_name: Name of the gait (e.g., 'TROT', 'WALK')

        Returns:
            bool: True if successful, False otherwise
        """
        if gait_name not in GAITS:
            return False

        gait_params = GAITS[gait_name]
        generator = self.GENERATORS.get(gait_params[0])
        if generator is None:
            return False

        trajectories = generator(self.leg, gait_params)
        if trajectories is None:
            return False

        self.current_trajectories = trajectories
        self.current_gait = gait_name
        return True

    def start_movement(self, direction):
        """
        Start or switch to a new movement direction.

        Args:
            direction: Direction string (e.g., 'f', 'b', 'fl_0.75')

        Returns:
            bool: True if movement started, False if trajectory not found
        """
        if self.current_trajectories is None:
            return False

        gait_trajectories = self.current_trajectories

        # Try exact match first
        if direction in gait_trajectories:
            self.current_trajectory = gait_trajectories[direction]
            self.current_direction = direction
            self.ongoing = True
            return True

        # Try fallback logic
        if direction in self.FALLBACK_MAP:
            for fallback in self.FALLBACK_MAP[direction]:
                if fallback in gait_trajectories:
                    self.current_trajectory = gait_trajectories[fallback]
                    self.current_direction = direction  # Remember requested direction
                    self.ongoing = True
                    return True

        # No suitable trajectory found
        return False

    def tick(self):
        """
        Get the next position in the current trajectory.

        Returns:
            list: Joint positions for all motors, or None if no movement active
        """
        if not self.ongoing or self.current_trajectory is None:
            return None

        # Get current position using phase index
        current_index = self.phase_index % len(self.current_trajectory)
        pos = self.current_trajectory[current_index]

        # Increment phase for next tick
        self.phase_index = (self.phase_index + 1) % len(self.current_trajectory)

        return pos

    def stop(self):
        """Stop current movement and reset state."""
        self.ongoing = False
        self.current_direction = None
        self.current_trajectory = None
        self.phase_index = 0
