
# import pigpio
import socket
import struct

import numpy as np

# --- SERVO SPECIFICATIONS & LIMITS ---
SERVO_PIN = 2               # The GPIO pin connected to the servo signal wire (BCM pin 2)
REFRESH_RATE_HZ = 250       # The desired PWM refresh rate (frequency)

# --- VERIFIED SYMMETRICAL LIMITS ---
CENTER_PULSE = 1500         # Symmetrical center pulse width (micro-seconds)
MIN_PULSE = 680             # The lowest non-binding pulse width.
MAX_PULSE = 2320            # The highest non-binding pulse width.
MICROS_PER_RAD = 11.3333 * 180.0 / np.pi

# --- ROBOT CONSTANTS ---
NUM_AXES = 3
NUM_LEGS = 4

# --- UDP SETTINGS ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
PINS = np.array([[2, 14, 18, 23], [3, 15, 27, 24], [4, 17, 22, 25]])
NEUTRAL_ANGLE_DEGREES = np.array([[0., 0., 0., 0.],[45., 45., 45., 45.], [-45., -45., -45., -45.]])
                
def send_servo_command(angle, axis, leg):
    """Converts a desired angle to PWM and sends the command to the servo."""
    print(angle, axis, leg)
    pwm_value = angle_to_pwm(angle, axis, leg)
    
    # Verify PWM is within bounds
    if not (MIN_PULSE <= pwm_value <= MAX_PULSE):
        raise ValueError(
            f"Angle {angle} corresponding to PWM value {pwm_value} for axis {axis}, leg {leg} is out of bounds."
        )
    
    # pi.set_servo_pulsewidth(pwm_params.pins[axis][leg], pwm_value)
        
def angle_to_pwm(angle, axis, leg):
    """Converts a joint angle (in radians) to a PWM pulse width (in microseconds)."""
    neutral_angle = NEUTRAL_ANGLE_DEGREES[axis][leg]

    pwm_value = int(
        CENTER_PULSE + MICROS_PER_RAD * (angle - neutral_angle)
    )
    return pwm_value


print(f"Attempting to connect to pigpio daemon...")
try:
    # pi = pigpio.pi() 
    # if not pi.connected:
    #     print("Error: Could not connect to pigpio daemon.")
    #     print("Please ensure the pigpio daemon is running (e.g., 'sudo pigpiod')")
    #     sys.exit(1)
    
    print("Testing right now - Connection successful.")

    # 1. Configure the servo pin
    # pi.set_PWM_frequency(SERVO_PIN, REFRESH_RATE_HZ)
    # print(f"PWM Frequency set to {pi.get_PWM_frequency(SERVO_PIN)} Hz.")
    
    # 2. Go to the symmetrical center position
    # print(f"Setting initial pulse: {CURRENT_PULSE} us...")
    # pi.set_servo_pulsewidth(SERVO_PIN, CURRENT_PULSE)
    # time.sleep(1) 
    
    # 3. setting up UDP listener
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT}...")

    # # --- Manual Control Loop ---
    # print("\n--- Servo Control (listening to software stack) ---")
    # print(f"Current Pulse Width: {CURRENT_PULSE} us (Center)")

    while True:
        data, addr = sock.recvfrom(1024) # buffer size    

        angles = struct.unpack("12d", data)

        print(f"Received from {addr}: {angles}")
        
        for i, angle in enumerate(angles):
            send_servo_command(angle, i // NUM_LEGS, i % NUM_LEGS)
            

    stop_servo(pi)
        
except Exception as e:
    print(f"An unexpected error occurred: {e}")