import sys
from tmc_driver import (
    Tmc2209,
    Loglevel,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)

STEPS_PER_REV = 1600  # Steps for 360 degrees

def execute_move(tmc, theta, duration, n):
    """
    Calculate and execute a move.
    
    theta:    angle in degrees (negative = reverse)
    duration: time in seconds
    n:        2+ (2 = sharp, 1000 = smooth triangle)
    """
    import time

    # Convert to integer steps FIRST, then derive speed/accel from actual steps
    steps = round(theta * STEPS_PER_REV / 360)

    v_avg = abs(steps) / duration
    v_max = v_avg * (n / (n - 1))
    a_max = (v_max / duration) * n

    # Round to nearest int (not truncate) to minimize timing error
    v_max_int = max(round(v_max), 1)
    a_max_int = max(round(a_max), 1)

    # Expected timing breakdown
    t_accel = v_max / a_max  # time to ramp up
    t_cruise = duration - 2 * t_accel  # time at constant speed
    print(f"  θ={theta}° → {steps} steps | speed={v_max_int} | accel={a_max_int} | n={n}")
    print(f"  profile: {t_accel:.2f}s ramp + {max(t_cruise, 0):.2f}s cruise + {t_accel:.2f}s ramp = {duration}s")

    tmc.acceleration_fullstep = a_max_int
    tmc.max_speed_fullstep = v_max_int

    t_start = time.time()
    tmc.run_to_position_steps(steps, MovementAbsRel.RELATIVE)
    actual = time.time() - t_start
    print(f"  actual time: {actual:.2f}s (expected {duration}s, diff={actual-duration:+.2f}s)")

def main():
    print("---")
    print("AXI6 Motion Controller")
    print("Input: time (s), angle (°), n(2+)")
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
            user_input = input("Time(s), Angle(°), n(2+)  [or 'q' to quit]: ").strip()

            if user_input.lower() == 'q':
                break

            try:
                parts = user_input.replace(',', ' ').split()
                if len(parts) != 3:
                    print("  Enter 3 values: time angle n  (e.g. '2 90 50')")
                    continue

                duration = float(parts[0])
                theta = float(parts[1])
                n = float(parts[2])

                if duration <= 0:
                    print("  Time must be > 0")
                    continue
                # if n < 2:
                #     print("  n must be 2+")
                #     continue

                execute_move(tmc, theta, duration, n)

            except ValueError:
                print("  Invalid input. Example: 2 90 50  (2s, 90°, n=50)")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        print("\nDeactivating motor...")
        tmc.set_motor_enabled(False)
        del tmc
        print("---\nSCRIPT FINISHED\n---")

if __name__ == "__main__":
    main()