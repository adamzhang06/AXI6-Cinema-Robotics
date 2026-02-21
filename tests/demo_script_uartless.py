import sys
from tmc_driver import (
    Tmc2209,
    Loglevel,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)

def rotate_degrees(tmc, degrees):
    # Calculate steps based on 1600 steps per 360-degree revolution
    steps = int((degrees / 360.0) * 1600)
    
    print(f"Executing: {degrees} degrees -> {steps} steps")
    # Use RELATIVE movement so it rotates from its current position
    tmc.run_to_position_steps(steps, MovementAbsRel.RELATIVE)

def main():
    print("---")
    print("AXI6 Interactive Degree Controller")
    print("---")

    # -----------------------------------------------------------------------
    # Initiate the Tmc2209 class
    # (Using the pins from your successful test: EN=21, STEP=16, DIR=20)
    # -----------------------------------------------------------------------
    tmc = Tmc2209(
        TmcEnableControlPin(21),
        TmcMotionControlStepDir(16, 20),
        loglevel=Loglevel.INFO, # Set to INFO so it doesn't spam the terminal with debug text
    )

    # Set the Acceleration and maximal Speed in fullsteps
    tmc.acceleration_fullstep = 1000
    tmc.max_speed_fullstep = 250

    # Activate the motor current output
    tmc.set_motor_enabled(True)
    print("Motor Enabled. Holding torque active.\n")

    try:
        # -----------------------------------------------------------------------
        # Terminal Input Loop
        # -----------------------------------------------------------------------
        while True:
            user_input = input("Enter degrees to rotate (0-360) or 'q' to quit: ")
            
            if user_input.lower() == 'q':
                break
                
            try:
                degrees = float(user_input)
                # Allow negative numbers if you want to rotate backward!
                if -360 <= degrees <= 360:
                    rotate_degrees(tmc, degrees)
                else:
                    print("Please enter a value between -360 and 360.")
            except ValueError:
                print("Invalid input. Please enter a valid number.")

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        
    finally:
        # -----------------------------------------------------------------------
        # Safe Shutdown Sequence
        # -----------------------------------------------------------------------
        print("\nDeactivating motor...")
        tmc.set_motor_enabled(False)
        del tmc
        print("---")
        print("SCRIPT FINISHED")
        print("---")

if __name__ == "__main__":
    main()