import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


# List of axes to control
AXES = ['pan', 'tilt']  # Add 'roll', 'slide', etc. as needed

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # Declare separate PID parameters for each axis
        param_list = []
        for axis in AXES:
            param_list.extend([
                (f'{axis}_kp', 1.0),
                (f'{axis}_ki', 0.0),
                (f'{axis}_kd', 0.0),
                (f'{axis}_max_output', 1.0),
                (f'{axis}_min_output', -1.0),
            ])
        self.declare_parameters(namespace='', parameters=param_list)
        self.error_sub = self.create_subscription(Float32, 'vision_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Store PID state for each axis
        self.integral = {axis: 0.0 for axis in AXES}
        self.prev_error = {axis: 0.0 for axis in AXES}

    def error_callback(self, msg):
        # For demonstration, assume msg.data is a single error for 'pan'.
        # For multi-axis, use a custom message or topic per axis.
        axis = 'pan'  # Change as needed or parse from msg
        error = msg.data
        self.integral[axis] += error
        derivative = error - self.prev_error[axis]
        kp = self.get_parameter(f'{axis}_kp').value
        ki = self.get_parameter(f'{axis}_ki').value
        kd = self.get_parameter(f'{axis}_kd').value
        output = kp * error + ki * self.integral[axis] + kd * derivative
        output = max(self.get_parameter(f'{axis}_min_output').value, min(self.get_parameter(f'{axis}_max_output').value, output))
        twist = Twist()
        twist.linear.x = output  # For multi-axis, set the correct field
        self.cmd_pub.publish(twist)
        self.prev_error[axis] = error
        self.get_logger().info(f'Published cmd_vel for {axis}: {output}')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
