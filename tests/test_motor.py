import time
import RPi.GPIO as GPIO

# Define your BCM GPIO pins
DIR_PIN = 20
STEP_PIN = 21
EN_PIN = 16

def setup_motor():
    # Use BCM numbering (the numbers on the Pi's chip, not the physical pin order)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(EN_PIN, GPIO.OUT)
    
    # TMC2209 is active-LOW. We pull the Enable pin LOW to turn on the motor current.
    GPIO.output(EN_PIN, GPIO.LOW)
    print("Motor Enabled. Holding torque active.")

def move_motor(steps, delay=0.001, direction="RIGHT"):
    print(f"Moving {direction} for {steps} steps...")
    
    # Set the direction pin
    GPIO.output(DIR_PIN, GPIO.HIGH if direction == "RIGHT" else GPIO.LOW)
    
    # Pulse the step pin (This is exactly what your motor_worker thread will do later!)
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay / 2)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay / 2)

def main():
    try:
        setup_motor()
        time.sleep(1) # Wait a second so you can feel the holding torque engage
        
        # Move forward 800 steps (Usually 4 full revolutions on a NEMA 17)
        move_motor(800, delay=0.002, direction="RIGHT")
        time.sleep(0.5)
        
        # Move backward 800 steps
        move_motor(800, delay=0.002, direction="LEFT")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        # ALWAYS pull Enable HIGH to shut off the current and prevent overheating
        GPIO.output(EN_PIN, GPIO.HIGH) 
        GPIO.cleanup()
        print("Motor disabled. GPIO pins cleaned up safely.")

if __name__ == "__main__":
    main()