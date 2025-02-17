#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from sensor_msgs_py import point_cloud2

class PointCloudCollector(Node):
    def __init__(self):
        super().__init__('point_cloud')

        # Subscribe to original point cloud
        self.points_subscriber = self.create_subscription(
            PointCloud2, '/camera_sensor/points', self.point_subscriber, 10)

        # Publisher for filtered point cloud
        self.points_publisher = self.create_publisher(PointCloud2, 'filtered_point_cloud', 10)

        self.get_logger().info("PointCloudCollector has started.")

    def point_subscriber(self, data):   
        self.get_logger().info(f"Received PointCloud2: {data.width * data.height} points")

        np_msg = self.pointcloud2_to_numpy(data)

        # filtered_points = np_msg[np_msg[:, 2] > 0.5]
        # filtered_points = np_msg[((np_msg[:, 3].astype(np.uint32) >> 16) & 0xFF) > 2]

        ######################################

        # Define a 45-degree rotation around Z-axis and a translation (1, 2, 3)
        theta = np.radians(45)
        cos_t, sin_t = np.cos(theta), np.sin(theta)

        transformation = np.array([
            [cos_t, -sin_t, 0, 0.0],  # Rotate & move x
            [sin_t, cos_t,  0, 0.0],  # Rotate & move y
            [0,     0,      1, -1.0],  # Move z
            [0,     0,      0, 1]     # Homogeneous row
        ])

        # Transform the points
        transformed_points = self.transform_point_cloud(np_msg, transformation)

        ######################################

        output = self.numpy_to_pointcloud2(transformed_points)

        self.points_publisher.publish(output)

# for future reference gpt wont help here, I found the below article helpful.
# https://docs.ros.org/en/iron/p/sensor_msgs_py/sensor_msgs_py.point_cloud2.html

    def numpy_to_pointcloud2(self, np_array):
        """Convert a NumPy array to a PointCloud2 message."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)
        ]

        # Convert NumPy array to a list of tuples
        points_list = np_array.tolist()

        return point_cloud2.create_cloud(header, fields, points_list)

    def pointcloud2_to_numpy(self, points_msg):
        """Converts PointCloud2 message to NumPy array."""
        return point_cloud2.read_points_numpy(points_msg, field_names=['x', 'y', 'z', "rgb"], skip_nans=True)

    def transform_point_cloud(self, points, transformation):
        """
        Applies a 4x4 transformation matrix to a point cloud.

        Args:
            points (np.ndarray): Nx4 array where each row is [x, y, z, rgb].
            transformation (np.ndarray): 4x4 transformation matrix.

        Returns:
            np.ndarray: Nx4 transformed point cloud.
        """
        if points.shape[1] != 4:
            raise ValueError("Input points must have shape (N, 4) with [x, y, z, rgb]")

        # Convert points to homogeneous coordinates (Nx4 -> Nx3 + Nx1)
        xyz = points[:, :3]  # Extract x, y, z
        ones = np.ones((xyz.shape[0], 1))  # Homogeneous coordinate

        # Convert to (N, 4) for matrix multiplication
        xyz_homogeneous = np.hstack((xyz, ones))

        # Apply transformation (Nx4) @ (4x4) -> (Nx4)
        transformed_xyz_homogeneous = xyz_homogeneous @ transformation.T

        # Extract transformed x, y, z (ignore homogeneous coordinate)
        transformed_xyz = transformed_xyz_homogeneous[:, :3]

        # Keep original RGB values
        transformed_points = np.hstack((transformed_xyz, points[:, 3:4]))

        return transformed_points

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
