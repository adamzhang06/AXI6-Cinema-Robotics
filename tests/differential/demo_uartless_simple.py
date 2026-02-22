"""
Demo file for basic movement without UART (only STEP/DIR/EN control)
"""

from tmc_driver import (
    Tmc2209,
    Loglevel,
    Board,
    tmc_gpio,
    MovementAbsRel,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)
from tmc_driver.com import TmcComUart


print("---")
print("SCRIPT START")
print("---")


# -----------------------------------------------------------------------
# initiate the Tmc2209 class
# use your pins for pin_en, pin_step, pin_dir here
# -----------------------------------------------------------------------
UART_PORT = {
    Board.RASPBERRY_PI: "/dev/serial0",
    Board.RASPBERRY_PI5: "/dev/ttyAMA0",
    Board.NVIDIA_JETSON: "/dev/ttyTHS1",
}

tmc1 = Tmc2209(
    TmcEnableControlPin(21),
    TmcMotionControlStepDir(16, 20),
    TmcComUart(UART_PORT.get(tmc_gpio.BOARD, "/dev/serial0")),
    loglevel=Loglevel.DEBUG,
)
tmc2 = Tmc2209(
    TmcEnableControlPin(26),
    TmcMotionControlStepDir(13, 19),
    TmcComUart(UART_PORT.get(tmc_gpio.BOARD, "/dev/serial0")),
    loglevel=Loglevel.DEBUG,
)


# -----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
# -----------------------------------------------------------------------
tmc1.tmc_logger.loglevel = Loglevel.DEBUG
tmc1.movement_abs_rel = MovementAbsRel.ABSOLUTE

tmc2.tmc_logger.loglevel = Loglevel.DEBUG
tmc2.movement_abs_rel = MovementAbsRel.ABSOLUTE


# -----------------------------------------------------------------------
# set the Acceleration and maximal Speed
# -----------------------------------------------------------------------
# tmc.set_acceleration(2000)
# tmc.set_max_speed(500)

# -----------------------------------------------------------------------
# set the Acceleration and maximal Speed in fullsteps
# -----------------------------------------------------------------------
tmc1.acceleration_fullstep = 256
tmc1.max_speed_fullstep = 640

tmc2.acceleration_fullstep = 256
tmc2.max_speed_fullstep = 640


# -----------------------------------------------------------------------
# activate the motor current output
# -----------------------------------------------------------------------
tmc1.set_motor_enabled(True)
tmc2.set_motor_enabled(True)


# -----------------------------------------------------------------------
# move the motor 1 revolution
# -----------------------------------------------------------------------
tmc1.run_to_position_steps(400)  # move to position 400
tmc1.run_to_position_steps(0)  # move to position 0

tmc2.run_to_position_steps(400)  # move to position 400
tmc2.run_to_position_steps(0)  # move to position 0


# -----------------------------------------------------------------------
# deactivate the motor current output
# -----------------------------------------------------------------------
tmc1.set_motor_enabled(False)
tmc2.set_motor_enabled(False)

print("---\n---")


# -----------------------------------------------------------------------
# deinitiate the Tmc2209 class
# -----------------------------------------------------------------------
del tmc1
del tmc2

print("---")
print("SCRIPT FINISHED")
print("---")