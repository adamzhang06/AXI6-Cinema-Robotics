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
    easing:   2 (2 = sharp, 1000 = smooth triangle)
    """

    steps = theta * STEPS_PER_REV / 360 
    v_max = (steps / duration) * (easing / (easing - 1))
    a_max = (v_max / duration) * easing

    print(f"  θ={theta}° → {target_steps} steps | speed={max_speed_full:.2f} | accel={accel_full:.2f} | time={duration}s | ease={easing}%")

    # The library accepts floats for these properties
    tmc.acceleration_fullstep = a_max
    tmc.max_speed_fullstep = v_max
    
    # run_to_position_steps expects raw microstep pulses
    tmc.run_to_position_steps(target_steps, MovementAbsRel.RELATIVE)

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
                    print("  Enter 3 values: time angle easing  (e.g. '2 90 50')")
                    continue

                duration = float(parts[0])
                theta = float(parts[1])
                easing = float(parts[2])

                if duration <= 0:
                    print("  Time must be > 0")
                    continue
                if easing < 2:
                    print("  Easing must be 2+")
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