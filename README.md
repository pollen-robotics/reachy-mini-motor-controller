# Reachy mini motor controller

Handles communication with the stewart platform (6x XL330), the base rotation (XC330) and the antennas (XL330).
Also provides a python binding available via pip.

Used by the [Reachy Mini](https://github.com/pollen-robotics/reachy_mini) project.

## To install locally 
```bash
pip install maturin
```

## To build the wheel
```bash
pip install -e . --verbose
```

## To install the wheel

```bash
cd `target/wheels`
pip install reachy_mini_motor_controller...
```

