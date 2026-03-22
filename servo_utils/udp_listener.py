
import pigpio
import socket
import struct
import sys

import numpy as np

# --- SERVO SPECIFICATIONS & LIMITS ---
REFRESH_RATE_HZ = 250       # The desired PWM refresh rate (frequency)

# --- VERIFIED SYMMETRICAL LIMITS ---
CENTER_PULSE = 1500         # Symmetrical center pulse width (micro-seconds)
MIN_PULSE = 680             # The lowest non-binding pulse width.
MAX_PULSE = 1900            # The highest non-binding pulse width.
MICROS_PER_DEG = 11.3333

# --- ROBOT CONSTANTS ---
NUM_AXES = 3
NUM_LEGS = 4

# --- UDP SETTINGS ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
# Converntion for servos: [LF_hip, LF_thigh, LF_knee, RF_hip, RF_thigh, RF_knee, LH_hip, LH_thigh, LH_knee, RH_hip, RH_thigh, RH_knee]
# PINS = [2,3, 4, 14, 15, 17, 18, 27, 22, 23, 24, 25]
CENTER_PULSES=[1500, 1400, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
PINS = [-1, 3, 4, -1, 15, 17, -1, 27, 22, -1, 23, 25]
# NEUTRAL_ANGLE_DEGREES = [0., -45, -0, 0., -45., 0, 0., 0., -0, 0., -45., 0,]
NEUTRAL_ANGLE_DEGREES = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# STARTING_ANGLE_DEGREES = [0., -45, -0, 0., -45., 0, 0., 0., -0, 0., -45., 0,]
STARTING_ANGLE_DEGREES = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MULTIPLIERS = [0, 1, -1, 0, -1, 1, 0, 0, -0, 0, -1, 1]
                
def send_servo_command(angle, pin, index):
    """Converts a desired angle to PWM and sends the command to the servo."""
    pwm_value = angle_to_pwm(angle, index)
    print(f"{index=}, {MULTIPLIERS[index]=}, {PINS[index]=}, {pwm_value=}")
    
    # Verify PWM is within bounds
    if not (MIN_PULSE <= pwm_value <= MAX_PULSE):
        raise ValueError(
            f"Angle {angle} corresponding to PWM value {pwm_value} for leg {index} is out of bounds."
        )
    
    print(f"Setting leg: {index} at pin {pin} to pwm: {pwm_value} ")
    pi.set_servo_pulsewidth(pin, pwm_value)
    print("Success")
        
def angle_to_pwm(angle, index):
    """Converts a joint angle (in radians) to a PWM pulse width (in microseconds)."""
    neutral_angle = NEUTRAL_ANGLE_DEGREES[index]

    print(CENTER_PULSES[index], MICROS_PER_DEG, angle, neutral_angle)
    pwm_value = int(
        CENTER_PULSES[index] + MULTIPLIERS[index] * MICROS_PER_DEG * (angle - neutral_angle)
    )
    return pwm_value


print(f"Attempting to connect to pigpio daemon...")
try:
    pi = pigpio.pi() 
    if not pi.connected:
        print("Error: Could not connect to pigpio daemon.")
        print("Please ensure the pigpio daemon is running (e.g., 'sudo pigpiod')")
        sys.exit(1)
    
    print("Testing right now - Connection successful.")

    # 1. Configure the servo pin
    for i, pin in enumerate(PINS):
        if pin != -1:            
            print(f"{pin}: PWM frequency set to {pi.get_PWM_frequency(pin)} Hz.")
            pi.set_PWM_frequency(pin, REFRESH_RATE_HZ)
            send_servo_command(STARTING_ANGLE_DEGREES[i], pin, i)
        
    
    # 3. setting up UDP listener
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT}...")

    # # --- Manual Control Loop ---
    print("\n--- Servo Control (listening to software stack) ---")

    prev_angles = [-1] * 12
    while True:
        data, addr = sock.recvfrom(1024) # buffer size    

        angles = struct.unpack("12d", data)

        print(f"Received from {addr}: {angles}")
        
        for i, angle in enumerate(angles):
            if angles[i] != prev_angles[i] and PINS[i] != -1:
                send_servo_command(angle * 180 / np.pi, PINS[i], i)

        prev_angles = angles        
    # stop_servo(pi)
        
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    # if 'pi' in locals():
        # stop_servo(pi)
    sys.exit(1)