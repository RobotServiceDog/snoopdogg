# Servo Utils

This directory contains utility modules for controlling servos in the SnoopDogg robot. It includes configuration parameters, hardware interface classes, and calibration data necessary for precise servo operation.

## Modules
- `config.py`: Contains configuration parameters for the servos, including pin assignments, frequency settings, and pulse width limits.
- `hardware_interface.py`: Implements the hardware interface for controlling the servos using PWM signals.
- `servo_calibration.py`: Provides calibration data and functions to convert between angles and PWM signals.
- `tests/`: Contains test scripts to validate the functionality of the hardware interface and servo control

## Usage

**Note**: Make sure to enable pigpio daemon before running any scripts that utilize the hardware interface.
```bash
sudo pigpiod
```

To use the servo utilities, import the necessary modules and create instances of the hardware interface and configuration classes.


### `move.py`
The file `move.py` allows for testing servo movements straight from the command line.
Example:
```bash
python3 move.py
``` 

Commands:
- "w": Move forward
- "s": Move backward
- "a": Left most
- "d": Right most

### Tests

To run the test scripts, navigate to the main directory and execute the desired test file. For example:
```bash
python3 -m servo_utils.tests.test_hardware_interface
```
