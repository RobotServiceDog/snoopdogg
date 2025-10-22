# servo_sweep.py - Manual control script for high-speed digital servo.
# Use 'w' and 's' to adjust the pulse width within the verified, safe limits (680us - 2320us).
# Run this script after starting the pigpio daemon: 'sudo pigpiod'
# Press 'q' or Ctrl+C to stop and safely disable the servo signal.

import pigpio
import time
import signal
import sys
import termios
import tty

# --- SERVO SPECIFICATIONS & LIMITS ---
SERVO_PIN = 2               # The GPIO pin connected to the servo signal wire (BCM pin 2)
REFRESH_RATE_HZ = 250       # The desired PWM refresh rate (frequency)

# Verified symmetrical limits
CENTER_PULSE = 1500         # Symmetrical center pulse width (micro-seconds)
MIN_PULSE = 680             # The lowest non-binding pulse width.
MAX_PULSE = 2320            # The highest non-binding pulse width.

# --- CONTROL CONSTANTS ---
ADJUSTMENT_STEP = 10        # Pulse width change per key press (micro-seconds)
CURRENT_PULSE = CENTER_PULSE # Start at center


# --- Utility Functions for Non-Blocking Keyboard Input ---

def getch():
    """Gets a single character input from the user without echoing."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # Set terminal to raw mode (non-canonical and no echo)
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def stop_servo(pi_instance):
    """Sets the servo pulse width to 0 to disable signal and stop movement."""
    try:
        pi_instance.set_servo_pulsewidth(SERVO_PIN, 0)
        pi_instance.stop()
        print("pigpio connection closed.")
    except Exception as e:
        print(f"Error during cleanup: {e}")

def signal_handler(sig, frame):
    """Gracefully handles keyboard interrupt (Ctrl+C)."""
    print("\n\nStopping servo and exiting...")
    if 'pi' in globals():
        stop_servo(pi)
    sys.exit(0)

# Catch Ctrl+C to ensure clean shutdown
signal.signal(signal.SIGINT, signal_handler)

print(f"Attempting to connect to pigpio daemon...")
try:
    pi = pigpio.pi() 
    if not pi.connected:
        print("Error: Could not connect to pigpio daemon.")
        print("Please ensure the pigpio daemon is running (e.g., 'sudo pigpiod')")
        sys.exit(1)
    
    print("Connection successful.")

    # 1. Configure the servo pin
    pi.set_PWM_frequency(SERVO_PIN, REFRESH_RATE_HZ)
    print(f"PWM Frequency set to {pi.get_PWM_frequency(SERVO_PIN)} Hz.")
    
    # 2. Go to the symmetrical center position
    print(f"Setting initial pulse: {CURRENT_PULSE} us...")
    pi.set_servo_pulsewidth(SERVO_PIN, CURRENT_PULSE)
    time.sleep(1) 

    # --- Manual Control Loop ---
    print("\n--- Manual Servo Control ---")
    print(f"Current Pulse Width: {CURRENT_PULSE} us (Center)")
    print("Use 'w' to Increase Pulse Width (+10us) | (Max: 2320 us)")
    print("Use 's' to Decrease Pulse Width (-10us) | (Min: 680 us)")
    print("Press 'q' to Quit.")

    while True:
        # Get single key press
        key = getch()
        
        # Determine the target pulse width based on input
        if key == 'w':
            new_pulse = CURRENT_PULSE + ADJUSTMENT_STEP
        elif key == 's':
            new_pulse = CURRENT_PULSE - ADJUSTMENT_STEP
        elif key == 'a':
            new_pulse = MIN_PULSE  # Jump to minimum
        elif key == 'd':
            new_pulse = MAX_PULSE  # Jump to maximum
        elif key == 'c':
            new_pulse = CENTER_PULSE  # Jump to center
        elif key == 'q':
            # Quit the loop
            print("\n\nExiting Manual Mode.")
            break
        else:
            # Ignore other keys
            continue

        # Check against the absolute physical limits
        if MIN_PULSE <= new_pulse <= MAX_PULSE:
            CURRENT_PULSE = new_pulse
            pi.set_servo_pulsewidth(SERVO_PIN, CURRENT_PULSE)
            
            # Print status update
            print(f"\rCurrent Pulse Width: {CURRENT_PULSE} us  ", end="", flush=True)

        elif new_pulse < MIN_PULSE:
            CURRENT_PULSE = MIN_PULSE
            print(f"\r[LIMIT] Cannot go below {MIN_PULSE} us. Current Pulse: {CURRENT_PULSE} us", end="", flush=True)
            pi.set_servo_pulsewidth(SERVO_PIN, CURRENT_PULSE) # Ensure it's set to the exact min

        elif new_pulse > MAX_PULSE:
            CURRENT_PULSE = MAX_PULSE
            print(f"\r[LIMIT] Cannot go above {MAX_PULSE} us. Current Pulse: {CURRENT_PULSE} us", end="", flush=True)
            pi.set_servo_pulsewidth(SERVO_PIN, CURRENT_PULSE) # Ensure it's set to the exact max
        
        time.sleep(0.01) # Small delay to prevent keys from registering too fast

    stop_servo(pi)
        
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    if 'pi' in locals():
        stop_servo(pi)
    sys.exit(1)
