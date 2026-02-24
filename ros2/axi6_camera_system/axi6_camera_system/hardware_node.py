import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time

from .gpio_motor_utils import motor_pulse_loop

# Motor configuration for scalability
MOTORS = {
    "pan":   {"STEP": 16, "DIR": 20, "EN": 21},
    "tilt":  {"STEP": 13, "DIR": 19, "EN": 26},
    # "roll":  {"STEP": xx, "DIR": xx, "EN": xx},
    # "slide": {"STEP": xx, "DIR": xx, "EN": xx},
}

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        self.subscription = self.create_subscription(
            String,
            'trajectory',
            self.trajectory_callback,
            10)
        # Setup all motor GPIOs
        import RPi.GPIO as GPIO
        all_pins = []
        for pins in MOTORS.values():
            all_pins.extend([pins["STEP"], pins["DIR"], pins["EN"]])
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(all_pins, GPIO.OUT)
        for pins in MOTORS.values():
            GPIO.output(pins["EN"], GPIO.LOW)  # Enable all motors
        self.get_logger().info('HardwareNode ready for trajectories.')

    def trajectory_callback(self, msg):
        data = json.loads(msg.data)
        # data should be a dict: {"pan": {"fwd": bool, "times": [...]}, "tilt": {...}, ...}
        import RPi.GPIO as GPIO
        for axis, traj in data.items():
            if axis not in MOTORS:
                self.get_logger().warning(f"Unknown axis '{axis}' in trajectory, skipping.")
                continue
            pins = MOTORS[axis]
            GPIO.output(pins["DIR"], GPIO.HIGH if traj.get("fwd", True) else GPIO.LOW)
        start_time_real = time.time() + 0.1
        threads = []
        for axis, traj in data.items():
            if axis not in MOTORS:
                continue
            pins = MOTORS[axis]
            t = threading.Thread(target=motor_pulse_loop, args=(pins["STEP"], traj["times"], start_time_real))
            threads.append(t)
            t.start()
        for t in threads:
            t.join()
        self.get_logger().info("Trajectory execution complete.")
        # Optionally, disable motors after move:
        for pins in MOTORS.values():
            GPIO.output(pins["EN"], GPIO.HIGH)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()