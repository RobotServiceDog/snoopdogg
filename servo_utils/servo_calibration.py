
import numpy as np


MICROS_PER_RAD = 11.3333 * 180.0 / np.pi
NEUTRAL_ANGLE_DEGREES = np.array(
[[  0.,  0.,  0.,  0.],
 [  0.,  0.,  0.,  0.],
 [  0.,  0.,  0.,  0.]])



# [min_pwm, mid_pwm, max_pwm]
SERVO_PWM_RANGES = np.array(
    # LF
    [0, 0, 0],
    [680, 1340, 2320],
    [800, 1550, 2320],
    
    # RF
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    
    # LH
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    
    # RH
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
)
    

