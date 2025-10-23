import numpy as np
import time
from reachy_mini_motor_controller import ReachyMiniMotorController

def main():
    c = ReachyMiniMotorController(serialport="/dev/tty.usbmodem58FA0959031")

    c.enable_torque()

    amp = np.deg2rad(30.0)
    freq = 0.25

    t0 = time.time()

    while True:
        t = time.time() - t0
        pos = amp * np.sin(2 * np.pi * freq * t)

        c.set_all_goal_positions([pos] * 9)

        cur = c.read_all_positions()

        errors = np.abs(np.array(cur) - pos)
        print(f"Current position: {cur}, Goal position: {pos}, Errors: {errors}")

        time.sleep(0.01)


if __name__ == "__main__":
    main()