# Changelog

### v1.6.0

* Cap the average motor voltage at 6 V (Goal PWM written from the supply voltage, refreshed every second and after each operating mode change).
* Refuse PWM operating mode (16), incompatible with the voltage cap.

### v1.5.6

* Fix control stats memory infinite growth.

### v1.5.5

* Fix a bug in async_read_raw_bytes that caused the whole loop to hang.

### v1.5.4

* Avoid lock poisoning when run loop panic.

### v1.5.3

* Improve error handling in the control loop to prevent panics.

### v1.5.2

* Do not reboot on input voltage error to avoid the robot going limp.

### v1.5.1

* Improve serial error handling
 https://github.com/pollen-robotics/reachy-mini-motor-controller/pull/34 by @brainwavecoder9

## v1.5.0

* Add a raw write method to directly send bytes on the serial bus. 

### v1.4.1

* Fix a bug in the reboot on error status.

## v1.4.0

* Add support for setting motor position PID gains through the control loop interface.

### v1.3.1

Fix port check for Windows.

## v1.3.0

Improve error messages.

### v1.2.0

* Added motor ID to name mapping in the control loop for easier identification of motors.

### v1.1.0

* Added async_read_raw_bytes and async_write_raw_bytes methods to read and write raw bytes to/from a motor by its ID.

## v1.0.0

* Update IDs to match beta release of the hardware (10-18)

## v0.6.0

* Added new methods for querying motor control state (torque enabled status and Stewart platform operating mode) on both Rust and Python sides.

### v0.6.1

* Add a close method to the control loop to ensure proper thread termination (also called when the control loop is dropped).

## v0.5.0

* Added a full control loop implementation on the Rust side. The Python bindings were updated accordingly. All calls are now asynchronous and should returns instantly.

## v0.4.0

* Operating mode
* Current target