import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class CloudViewer(Node):
    def __init__(self):
        super().__init__('cloud_viewer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.cloud_callback,
            10)

    def cloud_callback(self, msg):
        count = 0
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = point
            print(f"Point[{count}]: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            count += 1
            if count >= 10:  # 只打印前10个
                break

rclpy.init()
node = CloudViewer()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

