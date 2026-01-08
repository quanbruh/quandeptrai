#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDrive(Node):
    def __init__(self):
        super().__init__("xoaytron")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 10 Hz

        # Tốc độ tuyến tính và góc quay
        self.linear_speed = 1.0  # m/s
        self.angular_speed = 0.3  # rad/s

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Driving in circle: linear={msg.linear.x}, angular={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = CircleDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
