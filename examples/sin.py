import numpy as np
import time
import signal
import sys
from reachy_mini_motor_controller import ReachyMiniMotorController

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\n\nInterrupted! Disabling motors...')
    if 'controller' in globals():
        controller.disable_torque()
    sys.exit(0)

def main():
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Try common serial ports if not specified
    serial_ports = [
        "/dev/tty.usbmodem58FA0959031",
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
    ]
    
    controller = None
    for port in serial_ports:
        try:
            print(f"Trying port {port}...")
            controller = ReachyMiniMotorController(serialport=port)
            print(f"Connected to {port}")
            break
        except Exception as e:
            continue
    
    if controller is None:
        print("Failed to connect to any serial port")
        return
    
    # Make controller global for signal handler
    globals()['controller'] = controller
    
    controller.enable_torque()
    print("Running sine wave motion. Press Ctrl+C to stop.")
    
    amp = np.deg2rad(30.0)
    freq = 0.25
    t0 = time.time()
    
    try:
        while True:
            t = time.time() - t0
            pos = amp * np.sin(2 * np.pi * freq * t)
            
            # Set all motors to same position
            controller.set_all_goal_positions([pos] * 9)
            
            # Read current positions
            cur = controller.read_all_positions()
            errors = np.abs(np.array(cur) - pos)
            max_error = np.max(errors)
            mean_error = np.mean(errors)
            
            # Display status
            print(f"\rTime: {t:6.2f}s | Goal: {np.rad2deg(pos):6.1f}° | "
                  f"Max Error: {np.rad2deg(max_error):5.2f}° | "
                  f"Mean Error: {np.rad2deg(mean_error):5.2f}°", end='')
            
            time.sleep(0.01)
            
    except Exception as e:
        print(f"\n\nError: {e}")
    finally:
        print("\n\nDisabling motors...")
        controller.disable_torque()
        print("Done!")

if __name__ == "__main__":
    main()
