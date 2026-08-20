'''
Written by yufeng.wu0902@gmail.com

Trajectory generation module for Q8bot gaits.
This module contains functions for generating and managing gait trajectories.
'''

import math
# Note: No imports from kinematics_solver needed - leg is passed as parameter

def append_pos_list(list_1, list_2, list_3, list_4):
    """다리별(FL, FR, BL, BR) 궤적을 프레임 단위로 합쳐 하나의 관절각 리스트로 만든다."""
    return [p1 + p2 + p3 + p4 for p1, p2, p3, p4 in zip(list_1, list_2, list_3, list_4)]


def _gen_base_scales(leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, scales):
    """
    주어진 stride_scale 리스트 각각에 대해 _generate_base_trajectories를 호출.
    하나라도 실패(None)하면 전체를 None 리스트로 반환 (호출부는 results[0] is None으로 판별).
    """
    results = [
        _generate_base_trajectories(leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=scale)
        for scale in scales
    ]
    if any(r is None for r in results):
        return [None] * len(scales)
    return results


# trot/walk/bound/pronk 4개 gait는 모두 "base 궤적 생성 -> 다리별 위상 시프트 -> append_pos_list"
# 동일 구조라 테이블로 접는다. 다리 순서는 항상 [FL, FR, BL, BR].
# - scales: base 궤적을 미리 만들어둘 stride_scale 목록 (부호=전/후진, 크기=stride 비율)
# - leg_mult: 다리별 위상 시프트 배수(shift_unit의 몇 배를 밀지) — gait 고유의 다리 배치를 결정
# - shift_unit_fn(s1, s2): 시프트 1배 단위 계산식. gait마다 미묘히 달라(len_factor 사용 여부 등)
#   식 그대로 보존 — 대수적으로 같아 보여도 부동소수점 중간값이 달라질 수 있어 통일 금지.
# - directions: {방향이름: [FL,FR,BL,BR] 각 다리가 쓸 scale 값}
_LEN_FACTOR_HALF = lambda s1, s2: int(s1 * ((s1 + s2) / s1) / 2)
_LEN_FACTOR_QUARTER = lambda s1, s2: int(s1 * ((s1 + s2) / s1) / 4)
_SUM_QUARTER = lambda s1, s2: int((s2 + s1) / 4)
_NO_SHIFT = lambda s1, s2: 0

GAIT_SPECS = {
    'trot': dict(
        scales=[1.0, 0.75, 0.5, -1.0, -0.75, -0.5],
        leg_mult=[0, 1, 1, 0],
        shift_unit_fn=_LEN_FACTOR_HALF,
        directions={
            'f':  [1.0, 1.0, 1.0, 1.0],
            'b':  [-1.0, -1.0, -1.0, -1.0],
            'l':  [-1.0, 1.0, -1.0, 1.0],
            'r':  [1.0, -1.0, 1.0, -1.0],
            'fl_0.75': [0.75, 1.0, 0.75, 1.0],
            'fl_0.5':  [0.5, 1.0, 0.5, 1.0],
            'fr_0.75': [1.0, 0.75, 1.0, 0.75],
            'fr_0.5':  [1.0, 0.5, 1.0, 0.5],
            'bl_0.75': [-0.75, -1.0, -0.75, -1.0],
            'bl_0.5':  [-0.5, -1.0, -0.5, -1.0],
            'br_0.75': [-1.0, -0.75, -1.0, -0.75],
            'br_0.5':  [-1.0, -0.5, -1.0, -0.5],
        },
    ),
    'walk': dict(
        scales=[1.0, -1.0],
        leg_mult=[0, 1, 2, 3],
        shift_unit_fn=_LEN_FACTOR_QUARTER,
        directions={
            'f': [1.0, 1.0, 1.0, 1.0],
            'b': [-1.0, -1.0, -1.0, -1.0],
            'l': [-1.0, 1.0, -1.0, 1.0],
            'r': [1.0, -1.0, 1.0, -1.0],
        },
    ),
    'bound': dict(
        scales=[1.0, -1.0],
        leg_mult=[0, 0, 1, 1],
        shift_unit_fn=_SUM_QUARTER,
        directions={
            'f': [1.0, 1.0, 1.0, 1.0],
            'b': [-1.0, -1.0, -1.0, -1.0],
        },
    ),
    'pronk': dict(
        scales=[1.0, -1.0],
        leg_mult=[0, 0, 0, 0],
        shift_unit_fn=_NO_SHIFT,
        directions={
            'f': [1.0, 1.0, 1.0, 1.0],
            'b': [-1.0, -1.0, -1.0, -1.0],
        },
    ),
}


def generate_gait_trajectories(leg, gait_params):
    """trot/walk/bound/pronk 공용 생성기. gait_params[0](stacktype)로 GAIT_SPECS를 찾아 조립한다."""
    stacktype, x0, y0, xrange, yrange, yrange2, s1_count, s2_count = gait_params
    spec = GAIT_SPECS[stacktype]

    bases = _gen_base_scales(leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, spec['scales'])
    if bases[0] is None:
        return None
    base_map = dict(zip(spec['scales'], bases))

    shift_unit = spec['shift_unit_fn'](s1_count, s2_count)
    leg_mult = spec['leg_mult']

    def leg_traj(scale_key, leg_idx):
        traj = base_map[scale_key]
        shift = leg_mult[leg_idx] * shift_unit
        return traj[shift:] + traj[:shift]

    return {
        name: append_pos_list(*(leg_traj(scale_keys[i], i) for i in range(4)))
        for name, scale_keys in spec['directions'].items()
    }

def _linear_interpolate(start, end, y0, steps):
    """
    Linear interpolation from start to end x position at constant y.

    Args:
        start: Starting x position
        end: Ending x position
        y0: Constant y position
        steps: Number of interpolation steps

    Returns:
        List of [x, y] coordinate pairs
    """
    if steps <= 1:
        return [[end, y0]]

    trajectory = []
    x_step = (end - start) / steps

    for i in range(steps):
        x = start + (i + 1) * x_step
        trajectory.append([x, y0])

    return trajectory

def _sine_interpolate(start, end, y0, yrange, steps):
    """
    Sinusoidal lift interpolation following the lift phase pattern.
    Matches the lift phase from _generate_base_trajectories().

    Args:
        start: Starting x position
        end: Ending x position
        y0: Base y position
        yrange: Height of the lift
        steps: Number of interpolation steps

    Returns:
        List of [x, y] coordinate pairs
    """
    if steps <= 1:
        return [[end, y0]]

    trajectory = []
    x_step = (end - start) / steps
    freq = math.pi / steps

    for i in range(steps):
        x = start + (i + 1) * x_step
        y = y0 - math.sin((i + 1) * freq) * yrange
        trajectory.append([x, y])

    return trajectory

def _update_leg_positions(leg_lists, x_legs, dx_legs, y0, yrange, s1_count, s2_count, lift_idx=None):
    """
    한 스텝에서 다리 4개(FL,FR,BL,BR 순) 위치를 갱신하고 궤적 리스트에 추가.
    lift_idx로 지정된 다리 하나만 sine(들어올림) 보간, 나머지는 linear(지면) 보간.

    Returns:
        새 x 위치 리스트 [x_FL, x_FR, x_BL, x_BR]
    """
    x_new = [x + dx for x, dx in zip(x_legs, dx_legs)]
    for i, (lst, x, x_n) in enumerate(zip(leg_lists, x_legs, x_new)):
        lst.extend(_sine_interpolate(x, x_n, y0, yrange, s2_count) if i == lift_idx
                   else _linear_interpolate(x, x_n, y0, s1_count))
    return x_new

def generate_crawl_trajectories(leg, gait_params):
    """
    Generate trajectories for CRAWL gait.

    Pseudocode:
    s = xrange
    1. FL: (x0, y0) -> (x0-s, y0), FR: (x0, y0) -> (x0-s, y0),
       BL: (x0, y0) -> (x0-s, y0), BR: (x0, y0) -> (x0-s, y0).
    2. FL, FR, BL stay, BR lift: (x0-s, y0) -> (x0-s + 4*s, y0).
    3. FL, BL, BR stay, FR lift: (x0-s, y0) -> (x0-s + 4*s, y0).
    4. FL: (x0-s, y0) -> (x0-s - 2*s, y0), FR: (x0-s + 4*s, y0) -> (x0-s + 4*s - 2*s, y0),
       BL: (x0-s, y0) -> (x0-s - 2*s, y0), BR: (x0-s + 4*s, y0) -> (x0-s + 4*s - 2*s, y0).
    5. FL, FR, BR stay, BL lift: (x0-s - 2*s, y0) -> (x0-s - 2*s + 4*s, y0).
    6. FR, BL, BR stay, FL lift: (x0-s - 2*s, y0) -> (x0-s - 2*s + 4*s, y0).
    7. FL: (x0-s - 2*s + 4*s, y0) -> (x0-s - 2*s + 4*s - s, y0),
       FR: (x0-s + 4*s - 2*s, y0) -> (x0-s + 4*s - 2*s - s, y0),
       BL: (x0-s - 2*s + 4*s, y0) -> (x0-s - 2*s + 4*s - s, y0),
       BR: (x0-s + 4*s - 2*s, y0) -> (x0-s + 4*s - 2*s - s, y0).
    8. Repeat from step 1.
    """

    stacktype, x0, y0, s, yrange, yrange2, s1_count, s2_count = gait_params

    # Define crawl gait steps: [dx_FL, dx_FR, dx_BL, dx_BR, lift_leg_index]
    # lift_leg_index: 0=FL, 1=FR, 2=BL, 3=BR, None=no lift
    steps_forward = [
        [-s, -s, -s, -s, None],      # Step 1: All legs move to initial position
        [0, 0, 0, 4*s, 3],            # Step 2: BR lifts
        [0, 4*s, 0, 0, 1],            # Step 3: FR lifts
        [-2*s, -2*s, -2*s, -2*s, None],  # Step 4: All legs shift
        [0, 0, 4*s, 0, 2],            # Step 5: BL lifts
        [4*s, 0, 0, 0, 0],            # Step 6: FL lifts
        [-s, -s, -s, -s, None],       # Step 7: All legs shift back
    ]

    # For backward, reverse the leg order: FL<->BR, FR<->BL
    steps_backward = [
        [s, s, s, s, None],           # Step 1: All legs move backward
        [-4*s, 0, 0, 0, 0],            # Step 2: FL lifts (was BR)
        [0, 0, -4*s, 0, 2],            # Step 3: BL lifts (was FR)
        [2*s, 2*s, 2*s, 2*s, None],   # Step 4: All legs shift
        [0, -4*s, 0, 0, 1],            # Step 5: FR lifts (was BL)
        [0, 0, 0, -4*s, 3],            # Step 6: BR lifts (was FL)
        [s, s, s, s, None],           # Step 7: All legs shift back
    ]

    trajectories = {}

    # Generate forward and backward trajectories
    for direction, steps in [('f', steps_forward), ('b', steps_backward)]:
        x_legs = [x0, x0, x0, x0]  # FL, FR, BL, BR
        leg_lists = [[], [], [], []]

        # Execute each step
        for *dx_legs, lift_idx in steps:
            x_legs = _update_leg_positions(leg_lists, x_legs, dx_legs, y0, yrange, s1_count, s2_count, lift_idx)

        # Convert to joint angles
        joint_trajectories = []
        for fl, fr, bl, br in zip(*leg_lists):
            q1_FL, q2_FL, _ = leg.ik_solve(fl[0], fl[1], True, 1)
            q1_FR, q2_FR, _ = leg.ik_solve(fr[0], fr[1], True, 1)
            q1_BL, q2_BL, _ = leg.ik_solve(bl[0], bl[1], True, 1)
            q1_BR, q2_BR, _ = leg.ik_solve(br[0], br[1], True, 1)
            joint_trajectories.append([q1_FL, q2_FL, q1_FR, q2_FR, q1_BL, q2_BL, q1_BR, q2_BR])

        trajectories[direction] = joint_trajectories

    return trajectories


def _generate_base_trajectories(leg, x0, y0, xrange, yrange, yrange2, s1_count, s2_count, stride_scale=1.0):
    """
    단일 다리의 base 궤적(들어올림 -> 내려놓음 1사이클) 생성. stride_scale로 보폭을 조절.

    Returns:
        List of [q1, q2] joint angles, or None if IK/작업범위 실패
    """
    move_trajectory = []
    x_start = x0 - (xrange * stride_scale) / 2
    x_lift_step = (xrange * stride_scale) / s1_count
    x_down_step = (xrange * stride_scale) / s2_count
    x = x_start

    # Check physical limits
    if y0 - yrange < 5:
        return None

    # Generate trajectory points for complete gait cycle
    for i in range(s1_count + s2_count):
        if i < s1_count:
            # Lift phase: sinusoidal lift trajectory
            x += x_lift_step
            freq = math.pi / s1_count
            y = y0 - math.sin((i + 1) * freq) * yrange
        else:
            # Down phase: sinusoidal down trajectory
            x = x - x_down_step
            freq = math.pi / s2_count
            y = y0 + math.sin((i - s1_count + 1) * freq) * yrange2

        # Solve inverse kinematics
        q1, q2, check = leg.ik_solve(x, y, True, 1)

        # Validate IK solution
        # check=False면 ik_solve가 실패해 이전 각도를 반환한 것 — 궤적에 넣지 말고 workspace를 줄여 재시도.
        if not check:
            xr_new, yr_new = xrange - 1, yrange - 1
            if xr_new > 0 and yr_new > 0:
                return _generate_base_trajectories(
                    leg, x0, y0, xr_new, yr_new, yrange2, s1_count, s2_count, stride_scale
                )
            return None

        move_trajectory.append([q1, q2])

    return move_trajectory
