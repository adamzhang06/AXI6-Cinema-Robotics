import sys
from tmc_driver import (
    Tmc2209,
    Loglevel,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)

STEPS_PER_REV = 1600  # Steps for 360 degrees

# Auto-calibrated at startup
SPEED_CORRECTION = 1.0

def calibrate_driver(tmc):
    """Run a short test move to measure the driver's actual speed factor."""
    import time
    global SPEED_CORRECTION

    test_steps = 200
    test_speed = 200
    test_accel = 10000  # High accel so move is nearly all cruise

    tmc.acceleration_fullstep = test_accel
    tmc.max_speed_fullstep = test_speed

    print("[CALIBRATING] Running 200 steps at speed 200...")
    t0 = time.time()
    tmc.run_to_position_steps(test_steps, MovementAbsRel.RELATIVE)
    actual_time = time.time() - t0

    expected_time = test_steps / test_speed  # Should be 1.0s
    SPEED_CORRECTION = actual_time / expected_time

    print(f"[CALIBRATED] Expected {expected_time:.2f}s, actual {actual_time:.2f}s")
    print(f"[CALIBRATED] Speed correction factor: {SPEED_CORRECTION:.3f}")
    print(f"[CALIBRATING] Returning to start position...")

    # Return to original position
    tmc.run_to_position_steps(-test_steps, MovementAbsRel.RELATIVE)
    print()

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

    # Apply calibrated correction factor
    v_max_hw = max(round(v_max * SPEED_CORRECTION), 1)
    a_max_hw = max(round(a_max * SPEED_CORRECTION * SPEED_CORRECTION), 1)
    # Note: accel is distance/time² so it gets correction² 

    # Expected timing breakdown
    t_accel = v_max / a_max  # time to ramp up
    t_cruise = duration - 2 * t_accel  # time at constant speed
    print(f"  θ={theta}° → {steps} steps | speed={v_max_hw} | accel={a_max_hw} | n={n}")
    print(f"  profile: {t_accel:.2f}s ramp + {max(t_cruise, 0):.2f}s cruise + {t_accel:.2f}s ramp = {duration}s")

    tmc.acceleration_fullstep = a_max_hw
    tmc.max_speed_fullstep = v_max_hw

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

    # Auto-calibrate: measure actual driver speed
    calibrate_driver(tmc)

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