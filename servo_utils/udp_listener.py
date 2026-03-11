
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
MAX_PULSE = 2320            # The highest non-binding pulse width.
MICROS_PER_DEG = 11.3333

# --- ROBOT CONSTANTS ---
NUM_AXES = 3
NUM_LEGS = 4

# --- UDP SETTINGS ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
PINS = [-1, -1, -1, -1, 2, 3, -1, -1, 9, 10, -1, -1]
NEUTRAL_ANGLE_DEGREES = [0., 0., 0., 0., 45., 45., 45., 45., -45., -45., -45., -45.]
                
def send_servo_command(angle, index):
    """Converts a desired angle to PWM and sends the command to the servo."""
    pwm_value = angle_to_pwm(angle, index)
    
    # Verify PWM is within bounds
    if not (MIN_PULSE <= pwm_value <= MAX_PULSE):
        raise ValueError(
            f"Angle {angle} corresponding to PWM value {pwm_value} for leg {index} is out of bounds."
        )
    
    print(f"Setting leg: {index} to pwm: {pwm_value} ")
    pi.set_servo_pulsewidth(index, pwm_value)
    print("Success")
        
def angle_to_pwm(angle, index):
    """Converts a joint angle (in radians) to a PWM pulse width (in microseconds)."""
    neutral_angle = NEUTRAL_ANGLE_DEGREES[index]

    print(CENTER_PULSE, MICROS_PER_DEG, angle, neutral_angle)
    pwm_value = int(
        CENTER_PULSE + MICROS_PER_DEG * (angle - neutral_angle)
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
                send_servo_command(angle, PINS[i])

        prev_angles = angles        
    # stop_servo(pi)
        
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    # if 'pi' in locals():
        # stop_servo(pi)
    sys.exit(1)