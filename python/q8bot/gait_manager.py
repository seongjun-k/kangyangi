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

    def tick(self, steps=1):
        """
        Get the next position in the current trajectory.

        Args:
            steps: 진행할 위상 스텝 수. 호출자가 경과 시간으로 정한다(advance_phase) —
                   제어 루프가 목표 레이트를 못 지켜도 보행 속도가 느려지지 않게.

        Returns:
            list: Joint positions for all motors, or None if no movement active
        """
        if not self.ongoing or self.current_trajectory is None:
            return None

        # Get current position using phase index
        current_index = self.phase_index % len(self.current_trajectory)
        pos = self.current_trajectory[current_index]

        # Increment phase for next tick
        self.phase_index = (self.phase_index + steps) % len(self.current_trajectory)

        return pos

    def stop(self):
        """Stop current movement and reset state."""
        self.ongoing = False
        self.current_direction = None
        self.current_trajectory = None
        self.phase_index = 0


# 한 번에 따라잡을 수 있는 최대 위상 스텝. 긴 정지 후 복귀나 스케줄러 지연으로
# 다리가 한꺼번에 튀는 것을 막는다.
MAX_CATCHUP_STEPS = 4


def advance_phase(accum, dt, rate):
    """경과 시간 dt를 위상 스텝 수로 환산. 남은 소수는 다음 호출로 이월한다.

    Returns:
        (steps, 이월된 잔여 시간)
    """
    accum += min(dt, MAX_CATCHUP_STEPS / rate)
    steps = int(accum * rate)
    return steps, accum - steps / rate


if __name__ == "__main__":
    # python3 python/q8bot/gait_manager.py 로 실행되는 자체 점검.
    RATE = 200.0
    steps, rest = advance_phase(0.0, 1.0 / RATE, RATE)
    assert steps == 1 and abs(rest) < 1e-9, (steps, rest)

    # 루프가 빨라 아직 한 스텝이 안 찼으면 0 — 잔여는 이월된다.
    steps, rest = advance_phase(0.0, 0.5 / RATE, RATE)
    assert steps == 0 and abs(rest - 0.5 / RATE) < 1e-9, (steps, rest)
    steps, rest = advance_phase(rest, 0.5 / RATE, RATE)
    assert steps == 1, (steps, rest)

    # 루프가 밀렸으면 그만큼 건너뛴다 — 보행 속도 유지의 핵심.
    steps, _ = advance_phase(0.0, 3.0 / RATE, RATE)
    assert steps == 3, steps

    # 아무리 오래 밀려도 MAX_CATCHUP_STEPS를 넘지 않는다.
    steps, _ = advance_phase(0.0, 10.0, RATE)
    assert steps == MAX_CATCHUP_STEPS, steps

    # 잔여 이월 덕에 장기 평균은 rate와 일치한다(누적 드리프트 없음).
    accum, total = 0.0, 0
    for _ in range(1000):
        s, accum = advance_phase(accum, 1.0 / 150.0, RATE)   # 150Hz로 밀린 루프
        total += s
    assert total == int(1000 * RATE / 150.0), total

    print("ok")
