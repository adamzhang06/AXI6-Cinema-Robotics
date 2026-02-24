import RPi.GPIO as GPIO
import time

def setup_gpio(pan_pins, tilt_pins):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pan_pins + tilt_pins, GPIO.OUT)
    # Enable motors (LOW = Enabled for TMC2209)
    GPIO.output(pan_pins[2], GPIO.LOW)
    GPIO.output(tilt_pins[2], GPIO.LOW)

def cleanup_gpio(pan_pins, tilt_pins):
    # Disable motors (HIGH = Disabled)
    GPIO.output(pan_pins[2], GPIO.HIGH)
    GPIO.output(tilt_pins[2], GPIO.HIGH)
    GPIO.cleanup()

def motor_pulse_loop(step_pin, times_array, start_time_real):
    for target_t in times_array:
        while time.time() - start_time_real < target_t:
            pass
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.0000001)
        GPIO.output(step_pin, GPIO.LOW)
