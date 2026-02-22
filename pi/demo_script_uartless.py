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
    easing:   1 - 100 (1 = sharp, 100 = smooth triangle)
    """
    # Assuming standard NEMA 17 (200 full steps/rev). Adjust if using 0.9° (400)
    FULL_STEPS_PER_REV = 200 
    
    # Calculate microstepping multiplier (e.g., 1600 / 200 = 8 microsteps)
    MICROSTEPS = STEPS_PER_REV / FULL_STEPS_PER_REV 

    # Convert degrees to target microstep pulses
    target_steps = int(theta * STEPS_PER_REV / 360)
    abs_steps = abs(target_steps)
    
    if abs_steps == 0:
        return

    # 1. Calculate Acceleration Time (t_a)
    t_a = (easing * duration) / 200.0

    # 2. Max Speed and Accel in MICROSTEPS per sec
    max_speed_micro = abs_steps / (duration - t_a)
    accel_micro = max_speed_micro / t_a

    # 3. FIX: Convert to FULL STEPS per sec for the TMC properties
    max_speed_full = max_speed_micro / MICROSTEPS
    accel_full = accel_micro / MICROSTEPS

    print(f"  θ={theta}° → {target_steps} steps | speed={max_speed_full:.2f} | accel={accel_full:.2f} | time={duration}s | ease={easing}%")

    # The library accepts floats for these properties
    tmc.acceleration_fullstep = accel_full
    tmc.max_speed_fullstep = max_speed_full
    
    # run_to_position_steps expects raw microstep pulses
    tmc.run_to_position_steps(target_steps, MovementAbsRel.RELATIVE)

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