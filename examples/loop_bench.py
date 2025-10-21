import time
from datetime import timedelta

import numpy as np
from reachy_mini_motor_controller import FullBodyPosition, ReachyMiniPyControlLoop


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Reachy Mini Control Loop Benchmark")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port to use for communication with the Reachy Mini",
    )
    parser.add_argument(
        "--read-frequency",
        type=float,
        required=True,
        help="Frequency for reading positions (in Hz)",
    )
    parser.add_argument(
        "--write-frequency",
        type=float,
        required=True,
        help="Frequency for writing commands (in Hz)",
    )
    args = parser.parse_args()

    control_loop = ReachyMiniPyControlLoop(
        args.port,
        timedelta(seconds=1.0 / args.read_frequency),
        5,
        timedelta(seconds=1),
        timedelta(seconds=30),
    )
    write_period = 1.0 / args.write_frequency

    control_loop.enable_torque()

    last_stats_tick = time.time()
    sin_t0 = time.time()

    while True:
        t = time.time() - sin_t0
        target = np.sin(2 * np.pi * 0.25 * t) * np.deg2rad(10)
        pos = FullBodyPosition(
            body_yaw=target, stewart=[target] * 6, antennas=[target] * 2
        )

        control_loop.set_all_goal_positions(pos)

        if time.time() - last_stats_tick > 1.0:
            stats = control_loop.get_stats()
            print(stats)
            last_stats_tick = time.time()

        time.sleep(write_period)


if __name__ == "__main__":
    main()
