#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/lidar_points', 10)
        self.timer = self.create_timer(1.0, self.publish_test_data)
        self.get_logger().info('Test publisher started')

    def publish_test_data(self):
        # 创建测试点云数据
        num_points = 1000
        points = np.random.rand(num_points, 3) * 10.0  # 随机3D点
        intensities = np.random.rand(num_points) * 255.0  # 随机强度
        rings = np.random.randint(0, 64, num_points)  # 随机激光线束编号
        timestamps = np.arange(num_points) * 1000  # 递增时间戳（纳秒）

        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.frame_id = "hesai_lidar"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = num_points
        msg.is_dense = False
        msg.is_bigendian = False

        # 定义字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='timestamp', offset=18, datatype=PointField.UINT32, count=1)
        ]

        msg.point_step = 22  # 4+4+4+4+2+4 = 22 bytes
        msg.row_step = msg.point_step * msg.width

        # 填充数据
        data = []
        for i in range(num_points):
            # x, y, z (float32)
            data.extend(struct.pack('fff', points[i, 0], points[i, 1], points[i, 2]))
            # intensity (float32)
            data.extend(struct.pack('f', intensities[i]))
            # ring (uint16)
            data.extend(struct.pack('H', rings[i]))
            # timestamp (uint32)
            data.extend(struct.pack('I', timestamps[i]))

        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f'Published test point cloud with {num_points} points')

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2, '/lidar_points_ready', self.callback, 10)
        self.get_logger().info('Test subscriber started')

    def callback(self, msg):
        self.get_logger().info(f'Received processed point cloud: {msg.width} points, {len(msg.fields)} fields')
        # 打印字段信息
        for field in msg.fields:
            self.get_logger().info(f'Field: {field.name}, type: {field.datatype}, offset: {field.offset}')

def main():
    rclpy.init()
    
    publisher = TestPublisher()
    subscriber = TestSubscriber()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
