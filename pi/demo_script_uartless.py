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
    
    # Get acceleration and speed from user (Enter for defaults)
    accel_input = input("Acceleration [2000]: ").strip()
    accel = int(accel_input) if accel_input else 2000

    speed_input = input("Max Speed [1000]: ").strip()
    max_speed = int(speed_input) if speed_input else 1000

    tmc.acceleration_fullstep = accel
    tmc.max_speed_fullstep = max_speed

    # Activate the motor current output
    tmc.set_motor_enabled(True)
    print(f"\nMotor Enabled. Speed: {max_speed} | Accel: {accel}")
    print("Commands: degrees (-360 to 360), 'a' = set accel, 's' = set speed, 'q' = quit\n")

    try:
        # -----------------------------------------------------------------------
        # Terminal Input Loop
        # -----------------------------------------------------------------------
        while True:
            user_input = input(f"[spd={max_speed} acc={accel}] Enter degrees or command: ").strip()

            if user_input.lower() == 'q':
                break
            elif user_input.lower() == 's':
                val = input("  New max speed: ").strip()
                try:
                    max_speed = int(val)
                    tmc.max_speed_fullstep = max_speed
                    print(f"  Speed set to {max_speed}")
                except ValueError:
                    print("  Invalid number.")
                continue
            elif user_input.lower() == 'a':
                val = input("  New acceleration: ").strip()
                try:
                    accel = int(val)
                    tmc.acceleration_fullstep = accel
                    print(f"  Accel set to {accel}")
                except ValueError:
                    print("  Invalid number.")
                continue

            try:
                degrees = float(user_input)
                if -360 <= degrees <= 360:
                    rotate_degrees(tmc, degrees)
                else:
                    print("Please enter a value between -360 and 360.")
            except ValueError:
                print("Invalid input. Enter degrees, 'a', 's', or 'q'.")

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