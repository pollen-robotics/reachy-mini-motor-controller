# Changelog

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