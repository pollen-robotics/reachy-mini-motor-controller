from datetime import timedelta
from reachy_mini_motor_controller import ReachyMiniPyControlLoop
import time
import numpy as np

MOTOR_ID = 10
SERIAL_PORT = "/dev/tty.usbmodem58FA0959031"

TYPE_FROM_LENGTH = {
    1 : np.int8,
    2 : np.int16,
    4 : np.int32
}

def main():

    control_loop = ReachyMiniPyControlLoop(
        SERIAL_PORT,
        timedelta(seconds=1.0 / 100.0),
        5,
        timedelta(seconds=1),
        timedelta(seconds=30),
    )

    # The loop is actually running in background, but you can run
    # 31 = Temperature limit register address
    data = control_loop.async_read_raw_bytes(
        id=MOTOR_ID,
        addr=31, 
        length=1,
    )
    initial_temperature_limit = np.frombuffer(data, dtype=TYPE_FROM_LENGTH[1])
    print(f"Initial temperature limit : {initial_temperature_limit}")
    control_loop.async_write_raw_bytes(
        id=MOTOR_ID,
        addr=31,
        data=(initial_temperature_limit + 5).tobytes()
    )
    data = control_loop.async_read_raw_bytes(
        id=MOTOR_ID,
        addr=31,
        length=1,
    )
    modified_temperature_limit = np.frombuffer(data, dtype=TYPE_FROM_LENGTH[1])
    print(f"Modified temperature limit (+5): {modified_temperature_limit}")
    control_loop.async_write_raw_bytes(
        id=MOTOR_ID,
        addr=31,
        data=(initial_temperature_limit).tobytes()
    )

    # Or in a loop
    # 132 = Present position address
    for _ in range(10):
        data = control_loop.async_read_raw_bytes(
            id=MOTOR_ID,
            addr=132,
            length=4,
        )
        present_position = np.frombuffer(data, dtype=TYPE_FROM_LENGTH[4])
        print(f"Pesent position : {present_position}")
        time.sleep(0.1)

if __name__ == "__main__":
    main()