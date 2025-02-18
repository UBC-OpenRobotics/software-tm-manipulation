#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs

class PointCloudCollector(Node):
    def __init__(self):
        super().__init__('point_cloud')

        # Subscribe to original point cloud
        self.points_subscriber = self.create_subscription(
            PointCloud2, '/camera_sensor/points', self.point_subscriber, 10)

        # Publisher for filtered point cloud
        self.points_publisher = self.create_publisher(PointCloud2, 'filtered_point_cloud', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.broadcast_tf)

        self.get_logger().info("PointCloudCollector has started")

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  # Parent frame
        t.child_frame_id = "camera"  # Child frame

        # Set translation
        t.transform.translation.x = 0.28
        t.transform.translation.y = 0.34
        t.transform.translation.z = 0.28

        # Convert Euler (roll=0, pitch=0.9, yaw=3.1416) to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(3.8, 0, 3.14)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def point_subscriber(self, data):   
        self.get_logger().info(f"Received PointCloud2: {data.width * data.height} points")

        np_msg = self.pointcloud2_to_numpy(data)

        # filtered_points = np_msg[np_msg[:, 2] > 0.5]
        # filtered_points = np_msg[((np_msg[:, 3].astype(np.uint32) >> 16) & 0xFF) > 2]

        ######################################

        # Get transform from 'stand' to 'world'
        transform = self.tf_buffer.lookup_transform("world", "camera", rclpy.time.Time())

        # Extract translation
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z])

        # Extract rotation quaternion
        q = transform.transform.rotation
        quaternion = np.array([q.x, q.y, q.z, q.w])

        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_matrix(quaternion)

        # Create 4x4 transformation matrix
        tf_matrix = np.eye(4)  # Identity matrix
        tf_matrix[:3, :3] = rotation_matrix  # Top-left 3x3 is rotation
        tf_matrix[:3, 3] = translation  # Top-right 3x1 is translation

        # Transform the points
        transformed_points = self.transform_point_cloud(np_msg, tf_matrix)

        ######################################

        output = self.numpy_to_pointcloud2(transformed_points)

        self.points_publisher.publish(output)

# for future reference gpt wont help here, I found the below article helpful.
# https://docs.ros.org/en/iron/p/sensor_msgs_py/sensor_msgs_py.point_cloud2.html

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to a quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def quaternion_to_matrix(self, q):
        """Convert quaternion [x, y, z, w] to a rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

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
