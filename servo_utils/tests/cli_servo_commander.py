# cli_servo_commander.py
# A runnable script that takes 12 joint angles (in degrees) via CLI input
# and commands the hardware interface to set the servo positions.

# Example commands:
# 0 0 0 0 0 0 0 0 0 0 0 0
# 0 0 -10 0 0 0 0 0 0 0 0 0
# 0 0 -45 0 0 0 0 0 0 0 0 0
# 0 61 0 0 0 0 0 0 0 0 0 0 

import time
import math
import numpy as np
from servo_utils.hardware_interface import HardwareInterface
from servo_utils.config import NUM_AXES, NUM_LEGS

COMMAND_FILE = "servo_utils/tests/cases/vertical.txt"
DT = 0.05 # seconds

def cli_servo_commander():
    """
    Initializes the hardware interface, enters a command loop to read
    12 joint angles from the CLI, and publishes them to the servos.
    """
    
    hw_interface = None # Initialize to None for cleanup safety
    expected_count = NUM_AXES * NUM_LEGS
    
    # 1. Open hardware interface cleanly
    try:
        hw_interface = HardwareInterface()
        print("Hardware Interface initialized and ready.")
        print("-" * 50)
        
        # 2. Main command loop
        while True:
            try:
                # Inputs
                angles_inputs = []
                if COMMAND_FILE:
                    with open(COMMAND_FILE, "r") as f:
                        for user_input in f:
                            angles_inputs.append(np.degrees([float(val) for val in user_input.split()]))
                            print(f"Read input: {user_input}")    
                else:
                    print(f"Enter {expected_count} joint angles (in degrees, space-separated):")
                    print(" i.e., Leg 0 (Axes 0-2), Leg 1 (Axes 0-2), Leg 2 (Axes 0-2), Leg 3 (Axes 0-2).")
                    
                    # Get user input
                    user_input = input("Angles: ")
                    
                    # Check for exit command
                    if user_input.lower() in ['exit', 'quit', 'q']:
                        print("Exiting command loop.")
                        break
                        
                    # Parse input string into a list of degrees
                    angles_inputs.append([float(val) for val in user_input.split()])
                
                
                for angles_deg_flat in angles_inputs:
                    # Validate input count
                    if len(angles_deg_flat) != expected_count:
                        print(f"Error: Expected {expected_count} angles, but received {len(angles_deg_flat)}.")
                        continue
                    
                    # Convert to radians and reshape into the (NUM_LEGS, NUM_AXES) structure (4x3)
                    # NumPy is used for efficient conversion and reshaping.
                    angles_rad_flat = np.radians(angles_deg_flat)
                    joint_angles_rad = angles_rad_flat.reshape((NUM_LEGS, NUM_AXES))
                    
                    # 3. Publish to the servo
                    hw_interface.set_actuator_positions(joint_angles_rad)
                    
                    print(f"Command published successfully at {time.strftime('%H:%M:%S')}")
                    print("-" * 50)

                    # Enforce dt                    
                    time.sleep(DT)

            except ValueError as e:
                # Handle non-numeric input or parsing errors
                print(f"Input Error: Please enter 12 space-separated numbers. Details: {e}")
            except Exception as e:
                # Handle any error during the set_actuator_positions call
                print(f"An error occurred while commanding servos: {e}")
            finally:
                # Only run once if command file specified
                if COMMAND_FILE:
                    break
                
    except Exception as e:
        print(f"\n\nFATAL: Failed to initialize Hardware Interface: {e}")
        
    # 4. Handle shutdown cleanly
    finally:
        if hw_interface:
            hw_interface.close()
            print("Interface closed and all actuator signals disabled.")
    
if __name__ == "__main__":
    cli_servo_commander()