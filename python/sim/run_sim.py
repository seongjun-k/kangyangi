'''
Kangyangi 보행 시뮬레이터 엔트리.
mock_robot(가짜 펌웨어, UDP 8888 수신 스레드) + gait_generator(보행 궤적) +
udp_link.q8_udp(실제 송신부, protocol.md 포맷 그대로) 를 연결해
IK/보행/UDP 전 구간을 실물 로봇 없이 검증한다.

--headless: 애니메이션 없이 N걸음 돌리고 수신 패킷 수/체크섬 오류 0/torque 상태를 assert.
그 외: viz.py로 실시간 애니메이션 재생.
'''

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "q8bot"))
from kinematics_solver import k_solver  # noqa: E402
from gait_generator import generate_trot_trajectories  # noqa: E402
from udp_link import q8_udp  # noqa: E402
from gait_manager import GAITS  # noqa: E402

from mock_robot import MockRobot  # noqa: E402

SIM_IP = "127.0.0.1"
SIM_PORT = 8888


def build_trajectory():
    leg = k_solver()
    trajectories = generate_trot_trajectories(leg, GAITS["TROT"])
    if trajectories is None:
        raise RuntimeError("TROT 궤적 생성 실패 (IK 해 없음)")
    return trajectories["f"]  # 전진 1사이클 (n x 8, deg)


def send_gait_cycles(udp, trajectory, n_cycles, step_delay=0.02):
    udp.enable_torque()
    time.sleep(step_delay)
    for _ in range(n_cycles):
        for pose in trajectory:
            udp.move_all(pose)
            time.sleep(step_delay)


def main():
    parser = argparse.ArgumentParser(description="Kangyangi 보행 시뮬레이터")
    parser.add_argument("--headless", action="store_true", help="애니메이션 없이 자체 검사만 실행")
    parser.add_argument("--target-ip", default=SIM_IP, help="송신 대상 IP (기본: 127.0.0.1 mock_robot)")
    parser.add_argument("--cycles", type=int, default=2, help="보행 사이클 반복 횟수")
    args = parser.parse_args()

    robot = MockRobot(ip=SIM_IP, port=SIM_PORT).start()
    time.sleep(0.1)  # 수신 스레드 기동 대기

    udp = q8_udp()
    # udp_link.py 생성자에 대상 IP 인자가 없어(기본 192.168.4.1 고정) 인스턴스 속성을
    # 직접 덮어써서 시뮬레이터 대상(127.0.0.1)으로 리다이렉트한다 (udp_link.py 미수정).
    udp.ip = args.target_ip
    udp.port = SIM_PORT

    trajectory = build_trajectory()

    try:
        if args.headless:
            send_gait_cycles(udp, trajectory, args.cycles)
            time.sleep(0.1)  # 마지막 패킷 처리 대기
            ticks, torque_on, pkt_count, csum_err = robot.snapshot()

            assert csum_err == 0, f"체크섬 오류 발생: {csum_err}"
            assert pkt_count > 0, "수신 패킷 없음"
            expected_min = args.cycles * len(trajectory)  # cmd 1개 + motion N개 이상
            assert pkt_count >= expected_min, f"수신 패킷 수 부족: {pkt_count} < {expected_min}"
            assert torque_on is True, "torque가 켜져 있어야 함(연속 송신 중)"

            print(f"[run_sim] headless OK: pkt_count={pkt_count} csum_err={csum_err} torque_on={torque_on}")

            udp.disable_torque()
            time.sleep(0.05)
            _ticks2, torque_off, _pkt2, _csum2 = robot.snapshot()
            assert torque_off is False, "disable_torque 이후 torque off가 반영되지 않음"
            print("[run_sim] torque off 명령 반영 확인 OK")
        else:
            import threading

            from viz import run as viz_run

            sender = threading.Thread(
                target=send_gait_cycles, args=(udp, trajectory, max(args.cycles, 10_000)), daemon=True
            )
            sender.start()
            viz_run(lambda: robot.snapshot())
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
