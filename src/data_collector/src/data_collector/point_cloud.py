#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import tf2_ros
import tf_transformations as tf
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("PointCloudCollector has started.")

    def point_subscriber(self, data):   
        self.get_logger().info(f"Received PointCloud2: {data.width * data.height} points")

        np_msg = self.pointcloud2_to_numpy(data)

        # filtered_points = np_msg[np_msg[:, 2] > 0.5]
        # filtered_points = np_msg[((np_msg[:, 3].astype(np.uint32) >> 16) & 0xFF) > 2]

        ######################################

        # Lookup transform from "base_link" to "map"
        transform_stamped = self.tf_buffer.lookup_transform('rx150/base_link', 'world', rclpy.time.Time())

        # Convert TransformStamped to a 4x4 transformation matrix
        transformation_matrix = self.transform_to_matrix(transform_stamped)

        # Convert to homogeneous coordinates (N,4)
        ones = np.ones((np_msg.shape[0], 1))
        np_points_homogeneous = np.hstack((np_msg, ones))

        # Apply transformation
        transformed_points = (transformation_matrix @ np_points_homogeneous.T).T[:, :3]  # (N,3)

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

    def transform_to_matrix(self, transform_stamped):
        """Convert a ROS TransformStamped to a 4x4 homogeneous transformation matrix."""
        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation

        # Convert quaternion to rotation matrix
        rotation_matrix = tf.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

        # Construct 4x4 transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = [t.x, t.y, t.z]

        return transformation_matrix
    

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
