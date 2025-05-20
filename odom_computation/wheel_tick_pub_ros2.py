# wheel_tick_pub_ros2.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class WheelTickPublisher(Node):
    def __init__(self):
        super().__init__('wheel_tick_publisher')
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.0)

        self.wheel_radius = 0.065
        self.wheel_base = 0.21
        self.ticks_per_rev = 512

        self.linear_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.angular_velocity = self.get_parameter('angular_velocity').get_parameter_value().double_value

        self.left_ticks = 0
        self.right_ticks = 0

        self.left_pub = self.create_publisher(Int32, '/left_wheel_ticks', 10)
        self.right_pub = self.create_publisher(Int32, '/right_wheel_ticks', 10)

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_ticks)

    def publish_ticks(self):
        dt = 0.1
        v_left = self.linear_velocity - (self.angular_velocity * self.wheel_base / 2.0)
        v_right = self.linear_velocity + (self.angular_velocity * self.wheel_base / 2.0)

        left_dist = v_left * dt
        right_dist = v_right * dt

        ticks_per_meter = self.ticks_per_rev / (2 * 3.141592 * self.wheel_radius)

        self.left_ticks += int(left_dist * ticks_per_meter)
        self.right_ticks += int(right_dist * ticks_per_meter)

        self.left_pub.publish(Int32(data=self.left_ticks))
        self.right_pub.publish(Int32(data=self.right_ticks))

def main(args=None):
    rclpy.init(args=args)
    node = WheelTickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
