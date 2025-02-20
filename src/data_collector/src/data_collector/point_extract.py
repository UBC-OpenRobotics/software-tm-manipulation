#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import cv2
import numpy as np

class PointExtractCollector(Node):
    """
    This `ROS2` node subscribes to the `/camera_sensor/points` topic and publishes a filtered
    `PointCloud2` message to the camera_mask topic. 
    The filtering is based on a mask generated
    from the RGB data (extracted from the point cloud) using OpenCV processing.
    """
    def __init__(self):
        super().__init__('camera_stream_collector')
        self.display_visualization = True

        # SUBSCRIBER: Subscribe to the point cloud topic.
        self.points_subscriber = self.create_subscription(
            PointCloud2, '/camera_sensor/points', self.point_callback, 10)
        self.get_logger().info("PointCloud subscription started.")

        # PUBLISHER: Publish the filtered point cloud.
        self.publisher = self.create_publisher(PointCloud2, 'camera_mask', 10)
        publisher_timer_period = 0.5  # seconds
        self.timer = self.create_timer(publisher_timer_period, self.process_and_publish_data)

        self.point_data_raw = None

    def point_callback(self, msg) -> None:
        """
            Callback for inflowing PointCloud2 messages.
        """
        self.point_data_raw = msg
        num_points = msg.width * msg.height
        self.get_logger().info(f"Received PointCloud2 with {num_points} points")

    def process_and_publish_data(self):
        """
            Main processing function
        """
        if self.point_data_raw is None:
            return

        # PointCloud2 message -> numpy array.
        pc_array = point_cloud2.read_points_numpy(
            self.point_data_raw, field_names=['x', 'y', 'z', 'rgb'], skip_nans=True)
        height = self.point_data_raw.height
        width = self.point_data_raw.width
        if height * width != pc_array.shape[0]:
            self.get_logger().error("[FATAL] PointCloud2 appeared in an unexpected format, cannot apply 2D mask.")
            return

        # Reshape to 2D: (height, width, num_fields)
        pc_array = pc_array.reshape((height, width, -1))

        # Extract the RGB image from the point cloud
        # The 'rgb' field is stored as a float, the BITS tell us how to extract the data
        rgb_float = pc_array[..., 3]
        # 1) Convert the float32 data as uint32 to prep for extraction
        rgb_uint = rgb_float.view(np.uint32)
        # 2) Extract individual channels using bit-masking and shifting.

        # Verilog Equivalent:

        # wire [31:0] rgb_uint;
        # wire [7:0] r, g, b;
        # assign {r,g,b} = {rgb_uint[23:16], rgb_uint[15:8], rgb_uint[7:0]};

        r = ((rgb_uint >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_uint >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_uint & 0xFF).astype(np.uint8)
        
        cv_image = cv2.merge([b, g, r]) # Merge channels to get a BGR image (OpenCV uses BGR).

        mask = self.find_colored_cubes(cv_image) 
        filtered_point_cloud = self.filter_point_cloud(self.point_data_raw, mask) 
        self.get_logger().info("Publishing filtered PointCloud2")
        self.publisher.publish(filtered_point_cloud)

    def filter_point_cloud(self, point_cloud: PointCloud2, mask: np.ndarray) -> PointCloud2:
        """
            Filters the given PointCloud2 using a 2D mask (np.ndarray) that highlights regions of interest.
            
            Returns: A new PointCloud2 containing only the points corresponding to nonzero mask pixels.
        
        """
        pc_array = point_cloud2.read_points_numpy(
            point_cloud, field_names=['x', 'y', 'z', 'rgb'], skip_nans=True)
        height = point_cloud.height
        width = point_cloud.width
        if height * width != pc_array.shape[0]:
            self.get_logger().error("[FATAL] PointCloud2 appears in an unexpected format, cannot apply 2D mask.")
            return point_cloud
        
        pc_array = pc_array.reshape((height, width, -1))

        # Resize mask if its dimensions don't match the point cloud.
        if mask.shape[0] != height or mask.shape[1] != width:
            mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        else:
            mask_resized = mask

        bool_mask = (mask_resized != 0)
        pc_filtered = pc_array[bool_mask].reshape(-1, pc_array.shape[-1])
        return self.numpy_to_pointcloud2(pc_filtered)

    def numpy_to_pointcloud2(self, np_array):
        """
        Converts a numpy array (shape: [N, num_fields]) to a PointCloud2 message.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)
        ]
        points_list = np_array.tolist()
        return point_cloud2.create_cloud(header, fields, points_list)

    def __get_color_name(self, hsv_colour):
        """
        Helper function to determine a color name from HSV values.

        """
        h, s, v = hsv_colour
        if v < 50:
            return "Black"
        if s < 50:
            return "White" if v > 200 else "Gray"
        if (h < 10) or (h >= 160):
            return "Red"
        elif 10 <= h < 25:
            return "Orange"
        elif 25 <= h < 35:
            return "Yellow"
        elif 35 <= h < 85:
            return "Green"
        elif 85 <= h < 135:
            return "Blue"
        elif 135 <= h < 160:
            return "Purple"
        else:
            return "Unknown"

    def find_colored_cubes(self, cv_image):
        """
            Processes the provided BGR image to detect regions of interest based on color.
            - Draws contours and labels on the image for visualiation.
            - Returns a binary mask (`np.ndarray`) where non-zero values indicate detected regions.
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_color = cv2.inRange(hsv, (0, 50, 0), (179, 255, 255))
        blurred = cv2.GaussianBlur(mask_color, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # combined_mask superimposes all detected contours.
        combined_mask = np.zeros(cv_image.shape[:2], dtype="uint8")
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            # Superimpose the contour onto combined_mask.
            cv2.drawContours(combined_mask, [approx], -1, 255, -1)
            
            # OPENCV VISUALIZTION
            x, y, w, h = cv2.boundingRect(approx)
            mask_temp = np.zeros(cv_image.shape[:2], dtype="uint8")
            cv2.drawContours(mask_temp, [approx], -1, 255, -1)
            mean_val = cv2.mean(cv_image, mask=mask_temp)[:3]
            avg_colour_bgr = np.uint8([[list(mean_val)]])
            avg_colour_hsv = cv2.cvtColor(avg_colour_bgr, cv2.COLOR_BGR2HSV)[0][0]
            color_label = self.__get_color_name(avg_colour_hsv)
            label_text_color_bgr = (int(mean_val[0]), int(mean_val[1]), int(mean_val[2]))
            cv2.drawContours(cv_image, [approx], -1, (0, 255, 0), 2)
            cv2.putText(cv_image, color_label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, label_text_color_bgr, 2)
        
        if self.display_visualization:
            cv2.imshow("Result: Detected Cubes", cv_image)
            cv2.waitKey(1)
        
        return combined_mask

def main(args=None):
    rclpy.init(args=args)
    node = PointExtractCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
