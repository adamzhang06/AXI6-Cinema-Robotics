import sys
from tmc_driver import (
    Tmc2209,
    Loglevel,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)

STEPS_PER_REV = 1600  # Steps for 360 degrees

def execute_move(tmc, theta, duration, easing):
    """
    Calculate and execute a move with easing.
    
    theta:    angle in degrees (negative = reverse)
    duration: time in seconds
    easing:   1-100 (1 = sharp, 100 = smooth)
    """
    # Convert degrees to steps
    steps = int(theta * STEPS_PER_REV / 360)

    # Calculate speed and acceleration
    max_speed = abs(steps) / duration
    accel = (max_speed) / (2 * duration * (easing / 100))

    # Clamp to integers (driver needs ints)
    max_speed = max(int(max_speed), 1)
    accel = max(int(accel), 1)

    print(f"  θ={theta}° → {steps} steps | speed={max_speed} | accel={accel} | time={duration}s | ease={easing}%")

    tmc.acceleration_fullstep = accel
    tmc.max_speed_fullstep = max_speed
    tmc.run_to_position_steps(steps, MovementAbsRel.RELATIVE)

def main():
    print("---")
    print("AXI6 Motion Controller")
    print("Input: time (s), angle (°), easing (1-100)")
    print("---\n")

    tmc = Tmc2209(
        TmcEnableControlPin(21),
        TmcMotionControlStepDir(16, 20),
        loglevel=Loglevel.INFO,
    )

    tmc.set_motor_enabled(True)
    print("Motor Enabled.\n")

    try:
        while True:
            user_input = input("Time(s), Angle(°), Ease(1-100)  [or 'q' to quit]: ").strip()

            if user_input.lower() == 'q':
                break

            try:
                parts = user_input.replace(',', ' ').split()
                if len(parts) != 3:
                    print("  Enter 3 values: time angle easing  (e.g. '2 90 50')")
                    continue

                duration = float(parts[0])
                theta = float(parts[1])
                easing = float(parts[2])

                if duration <= 0:
                    print("  Time must be > 0")
                    continue
                if easing < 1 or easing > 100:
                    print("  Easing must be 1-100")
                    continue

                execute_move(tmc, theta, duration, easing)

            except ValueError:
                print("  Invalid input. Example: 2 90 50  (2s, 90°, 50% easing)")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        print("\nDeactivating motor...")
        tmc.set_motor_enabled(False)
        del tmc
        print("---\nSCRIPT FINISHED\n---")

if __name__ == "__main__":
    main()