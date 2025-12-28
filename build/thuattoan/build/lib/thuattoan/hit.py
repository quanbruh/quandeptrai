import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class HitObjectNode(Node):
    def __init__(self):
        super().__init__("hit_object_node")

        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, "/diff_cont/cmd_vel_unstamped", 10)

        self.closest_dist = None
        self.closest_angle = None

    def scan_callback(self, msg):
        angle = msg.angle_min
        min_dist = 999
        min_angle = 0

        # tìm vật gần nhất
        for r in msg.ranges:
            if msg.range_min < r < min_dist:
                min_dist = r
                min_angle = angle

            angle += msg.angle_increment

        self.closest_dist = min_dist
        self.closest_angle = min_angle

        self.control_robot()

    def control_robot(self):
        if self.closest_dist is None:
            return

        cmd = Twist()

        # 1. Xoay về phía vật
        if abs(self.closest_angle) > 0.1:
            cmd.angular.z = 0.4 * (1 if self.closest_angle > 0 else -1)
            cmd.linear.x = 0.0
        else:
            # 2. Khi đã hướng đúng → tiến thẳng
            if self.closest_dist > 0.2:
                cmd.linear.x = 0.3  # tốc độ tiến
                cmd.angular.z = 0.0
            else:
                # 3. Đã tông sát → dừng
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("Đã tông vào vật!")

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HitObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
