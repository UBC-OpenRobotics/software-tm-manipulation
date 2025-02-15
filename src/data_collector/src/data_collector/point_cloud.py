#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import numpy as np
import pandas as pd
import struct

class PointCloudCollector(Node):
    def __init__(self):
        super().__init__('point_cloud')

        self.points = self.create_subscription( PointCloud2, '/camera_sensor/points', self.point_subscriber, 10)
        
        self.points_publisher = self.create_publisher( PointCloud2, 'filtered_point_cloud', 10)

        self.get_logger().info("PointCloudCollector has started.")

    def point_subscriber(self, data):   
        self.get_logger().info(f"Received: {data.width * data.height}")

        point_cloud = self.read_points(data)

        self.get_logger().info(f'points: {point_cloud}')

    def read_points(self, points):
        fmt = 'fff'
        point_step = points.point_step
        data = points.data

        res = np.array([
            struct.unpack_from(fmt, data, offset=i)
            for i in range(0, len(data), point_step)
        ])

        res = res[res[:, 0] > 0.5]
        # res = res[res[:, 1] > 0.5]
        # res = res[res[:, 2] > 0.5]

        return res

    def plot_point_cloud(points):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap='jet', marker='o', s=1)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Point Cloud Visualization')

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()