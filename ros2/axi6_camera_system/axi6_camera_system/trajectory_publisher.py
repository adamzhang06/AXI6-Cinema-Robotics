import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(String, 'trajectory', 10)

    def publish_differential_trajectory(self, pan_fwd, pan_times, tilt_fwd, tilt_times):
        # Compose the message in the format expected by hardware_node
        msg = {
            'pan': {'fwd': pan_fwd, 'times': pan_times},
            'tilt': {'fwd': tilt_fwd, 'times': tilt_times}
        }
        self.publisher_.publish(String(data=json.dumps(msg)))
        self.get_logger().info('Published differential trajectory to /trajectory topic.')


def calculate_step_times(total_steps, total_time, phases=3.0):
    if total_steps == 0:
        return []
    t_a = total_time / phases
    t_c = total_time - t_a
    v_max = total_steps / (total_time - t_a)
    a = v_max / t_a
    P_a = 0.5 * a * (t_a**2)
    P_c = P_a + v_max * (t_c - t_a)
    step_times = []
    for p in range(1, int(total_steps) + 1):
        if p <= P_a:
            t = math.sqrt((2 * p) / a)
        elif p <= P_c:
            t = t_a + ((p - P_a) / v_max)
        else:
            dp = p - P_c
            discriminant = max(0.0, (v_max**2) - (2 * a * dp))
            t_prime = (v_max - math.sqrt(discriminant)) / a
            t = t_c + t_prime
        step_times.append(t)
    return step_times


# Example usage: python3 -m axi6_camera_system.trajectory_publisher
if __name__ == '__main__':
    rclpy.init()
    node = TrajectoryPublisher()

    # Get user input for pan/tilt and time
    pan_angle = float(input("Enter pan angle (deg): "))
    tilt_angle = float(input("Enter tilt angle (deg): "))
    total_time = float(input("Enter total move time (s): "))
    phases = float(input("Enter total phases (e.g., 3.0): "))

    steps_per_rev = 400
    # Differential linkage for 2-motor test: motor1 = pan + tilt, motor2 = pan - tilt
    motor1_angle = pan_angle + tilt_angle
    motor2_angle = pan_angle - tilt_angle

    m1_steps = int(abs((motor1_angle / 360.0) * steps_per_rev))
    m2_steps = int(abs((motor2_angle / 360.0) * steps_per_rev))
    m1_fwd = (motor1_angle >= 0)
    m2_fwd = (motor2_angle >= 0)

    m1_times = calculate_step_times(m1_steps, total_time, phases=phases)
    m2_times = calculate_step_times(m2_steps, total_time, phases=phases)

    # For now, publish as 'pan' and 'tilt' for hardware_node compatibility
    axes_dict = {
        'pan': {'fwd': m1_fwd, 'times': m1_times},
        'tilt': {'fwd': m2_fwd, 'times': m2_times}
    }
    node.publisher_.publish(String(data=json.dumps(axes_dict)))
    node.get_logger().info('Published trajectory to /trajectory topic.')
    rclpy.shutdown()
