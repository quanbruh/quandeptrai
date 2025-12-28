import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarPointPrinter(Node):
    def __init__(self):
        super().__init__("lidar_point_printer")

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.callback,
            10
        )

    def callback(self, msg: LaserScan):
        angle = msg.angle_min
        points = []  # lưu danh sách điểm XY

        for r in msg.ranges:
            if 0.05 < r < msg.range_max:   # bỏ giá trị lỗi
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

            angle += msg.angle_increment

        # In ra vài điểm để không spam quá
        self.get_logger().info(f"Số điểm quét được: {len(points)}")
        
        for i in range(0, len(points), 30):  # in 1/30 số điểm
            x, y = points[i]
            print(f"Vật tại: x={x:.2f} m , y={y:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarPointPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
