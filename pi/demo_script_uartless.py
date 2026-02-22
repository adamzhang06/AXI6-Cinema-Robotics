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
    # Convert degrees to steps (keep signed for direction)
    target_steps = int(theta * STEPS_PER_REV / 360)
    abs_steps = abs(target_steps)
    
    # Catch zero-movement to prevent division by zero errors
    if abs_steps == 0:
        return

    # 1. Calculate Acceleration Time (t_a)
    # Easing (1-100) mapped to a max of 50% of the total duration
    t_a = (easing * duration) / 200.0

    # 2. Calculate Maximum Speed (steps / second)
    # v_max = total_distance / (total_time - acceleration_time)
    max_speed = abs_steps / (duration - t_a)

    # 3. Calculate Acceleration (steps / second^2)
    # a_max = v_max / acceleration_time
    accel = max_speed / t_a

    # Clamp to integers (TMC driver requires ints)
    max_speed_int = max(int(max_speed), 1)
    accel_int = max(int(accel), 1)

    print(f"  θ={theta}° → {target_steps} steps | speed={max_speed_int} | accel={accel_int} | time={duration}s | ease={easing}%")

    tmc.acceleration_fullstep = accel_int
    tmc.max_speed_fullstep = max_speed_int
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