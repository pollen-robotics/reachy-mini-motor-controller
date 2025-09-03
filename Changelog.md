# Changelog

## v0.6.0

* Added new methods for querying motor control state (torque enabled status and Stewart platform operating mode) on both Rust and Python sides.

## v0.5.0

* Added a full control loop implementation on the Rust side. The Python bindings were updated accordingly. All calls are now asynchronous and should returns instantly.

## v0.4.0

* Operating mode
* Current target