import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Vector3
from tf_transformations import quaternion_from_euler
import math
import time

class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')

        self.wheel_radius = 0.065  
        self.wheel_base = 0.21     
        self.ticks_per_rev = 512
        self.ticks_per_meter = self.ticks_per_rev / (2 * math.pi * self.wheel_radius)


        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # subscribing to the wheel tick Topics
        self.create_subscription(Int32, '/left_wheel_ticks', self.left_callback, 10)
        self.create_subscription(Int32, '/right_wheel_ticks', self.right_callback, 10)

        self.left_ticks = 0
        self.right_ticks = 0
        self.new_data = False

        # Publishing to the odom topic 
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer is set to 10 Hz
        self.timer = self.create_timer(0.1, self.update_odom)

    def left_callback(self, msg):
        self.left_ticks = msg.data
        self.new_data = True

    def right_callback(self, msg):
        self.right_ticks = msg.data
        self.new_data = True

    def update_odom(self):
        if not self.new_data or self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        delta_left = self.left_ticks - self.prev_left_ticks
        delta_right = self.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        dist_left = delta_left / self.ticks_per_meter
        dist_right = delta_right / self.ticks_per_meter
        dist_avg = (dist_left + dist_right) / 2.0

        delta_theta = (dist_right - dist_left) / self.wheel_base

        self.x += dist_avg * math.cos(self.theta + delta_theta / 2.0)
        self.y += dist_avg * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta


        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi


        quat = quaternion_from_euler(0, 0, self.theta)

        # Publishing the  odometry to the /odom topic
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        odom_msg.twist.twist.linear = Vector3(x=dist_avg / dt, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=delta_theta / dt)

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


