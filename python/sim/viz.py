'''
matplotlib 애니메이션: mock_robot의 관절 상태(tick)를 받아 4다리 5바 링크 측면도를 그린다.

좌표계는 kinematics_solver.k_solver._fk_calc()와 동일:
  - 모터2(q2, l1p) 피벗 = (0, 0)
  - 모터1(q1, l1)  피벗 = (d, 0)
  - 두 크랭크(l1, l1p) 끝(elbow)에서 각각 l2, l2p 길이 링크가 발(foot)에서 만남.
측면도이므로 좌/우 다리는 동일 평면에 겹쳐 그리되 앞/뒤 다리만 x축으로 떨어뜨려 구분한다
(원본 q8bot 코드에 몸통 길이 스펙이 없어 BODY_LENGTH는 시각 구분용 추정값 - ponytail).
'''

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "q8bot"))
from kinematics_solver import k_solver  # noqa: E402
from udp_link import ZERO_OFFSET, GEAR_RATIO  # noqa: E402 (SSoT: python/q8bot/udp_link.py)

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 기본 폰트(DejaVu Sans)에 한글 글리프가 없어 제목이 깨짐 — CJK 폰트가 있으면 지정
from matplotlib import font_manager as _fm
if any("Noto Sans CJK KR" in f.name for f in _fm.fontManager.ttflist):
    plt.rcParams["font.family"] = "Noto Sans CJK KR"

# tick -> deg 역변환 계수. ZERO_OFFSET/GEAR_RATIO는 udp_link.py에서 import(SSoT).
_DEG_PER_TICK = 360.0 / 4096.0 / GEAR_RATIO

BODY_LENGTH = 120  # 앞/뒤 다리 시각 구분용 x offset(mm), 실측 스펙 없음 - ponytail 추정값

# 다리 순서(ID 11-18): FL(q1,q2), FR(q1,q2), BL(q1,q2), BR(q1,q2)
# gait_generator append_pos_list 순서와 동일. 측면도용 (x_offset, color)
LEG_LAYOUT = [
    ("FL", BODY_LENGTH / 2, "tab:blue"),
    ("FR", BODY_LENGTH / 2, "tab:orange"),
    ("BL", -BODY_LENGTH / 2, "tab:green"),
    ("BR", -BODY_LENGTH / 2, "tab:red"),
]


def tick2deg(tick):
    return (tick - ZERO_OFFSET) * _DEG_PER_TICK


def leg_points(leg, q1_deg, q2_deg, x_offset):
    '''pivot1(q1), pivot2(q2), elbow1, elbow2, foot 좌표를 x_offset만큼 이동해 반환.'''
    q1 = leg._deg2rad(q1_deg)
    q2 = leg._deg2rad(q2_deg)
    p1 = (leg.d + x_offset, 0)
    p2 = (0 + x_offset, 0)
    e1 = (leg.l1 * math.cos(q1) + p1[0], leg.l1 * math.sin(q1))
    e2 = (leg.l1p * math.cos(q2) + p2[0], leg.l1p * math.sin(q2))
    foot = leg.fk_solve(q1_deg, q2_deg, deg=True)
    foot = (foot[0] + x_offset, foot[1])
    return p1, p2, e1, e2, foot


def run(get_state, interval_ms=100):
    '''
    get_state: () -> (joint_ticks(list of 8), torque_on(bool)) 를 반환하는 콜백.
    호출측(run_sim.py)에서 mock_robot.snapshot()을 래핑해 전달.
    '''
    leg = k_solver()
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-BODY_LENGTH, BODY_LENGTH)
    ax.set_ylim(90, -20)  # y는 모터 기준 아래로 증가하는 좌표라 반전해서 표시
    ax.set_title("Kangyangi 보행 시뮬레이터 (측면도)")
    lines = []
    for name, _x, color in LEG_LAYOUT:
        (line,) = ax.plot([], [], "o-", color=color, label=name)
        lines.append(line)
    status_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    ax.legend(loc="lower right")

    def update(_frame):
        ticks, torque_on, pkt_count, csum_err = get_state()
        for i, (name, x_offset, _color) in enumerate(LEG_LAYOUT):
            q1_deg = tick2deg(ticks[i * 2])
            q2_deg = tick2deg(ticks[i * 2 + 1])
            try:
                p1, p2, e1, e2, foot = leg_points(leg, q1_deg, q2_deg, x_offset)
            except Exception:
                continue  # IK/FK 실패 프레임은 이전 형상 유지
            xs = [p1[0], e1[0], foot[0], e2[0], p2[0]]
            ys = [p1[1], e1[1], foot[1], e2[1], p2[1]]
            lines[i].set_data(xs, ys)
        status_text.set_text(f"torque={'ON' if torque_on else 'OFF'} pkt={pkt_count} csum_err={csum_err}")
        return lines + [status_text]

    anim = FuncAnimation(fig, update, interval=interval_ms, blit=False)
    plt.show()
    return anim
