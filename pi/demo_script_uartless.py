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

    steps = theta * STEPS_PER_REV / 360 

    v_avg = steps / duration
    v_max = v_avg * (n / (n - 1))
    a_max = (v_max / duration) * n

    print(f"  θ={theta}° → {steps} steps | speed={v_max:.2f} | accel={a_max:.2f} | time={duration}s | n={n}")

    # The library accepts floats for these properties
    tmc.acceleration_fullstep = a_max
    tmc.max_speed_fullstep = v_max
    
    # run_to_position_steps expects raw microstep pulses
    tmc.run_to_position_steps(steps, MovementAbsRel.ABSOLUTE)

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
                if n < 2:
                    print("  n must be 2+")
                    continue

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