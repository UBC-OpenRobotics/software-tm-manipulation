#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import sys

IMAGE_TYPES = [ 'rgb_frame', ]
FRAME_RATES = [ 6, 15, 30, 60 ]


class Pub(Node):
    def __init__(self):
        super().__init__('intel_d435_camera_pub')

        self.image_type = 'rgb_frame'
        self.frame_rate = 30
        self.get_params()

        self.pub = self.create_publisher(Image, self.image_type, 10)
        timer_period = 1 / self.frame_rate
        self.br_rgb = CvBridge()

        try:
            self.pipe = rs.pipeline()
            self.cfg  = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, self.frame_rate)
            self.pipe.start(self.cfg)
            self.timer = self.create_timer(timer_period, self.timer_callback)
        except Exception as e:
            print(e)
            self.get_logger().error('INTEL REALSENSE IS NOT CONNECTED')

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        self.pub.publish(self.br_rgb.cv2_to_imgmsg(color_image))

    def get_params(self):
        self.declare_parameter('image_type', 'rgb_frame')
        self.declare_parameter('frame_rate', 30)
        # image_type
        image_type = self.get_parameter('image_type').get_parameter_value().string_value
        if image_type not in IMAGE_TYPES :
            self.get_logger().error(f'Invalid image type: {image_type}. Valid options are: {", ".join(IMAGE_TYPES)}.')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        # frame_rate
        try:
            frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        except ValueError as e:
            self.get_logger().error(f'Invalid value for frame_rate: {e}. Expected an integer.')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        if frame_rate not in FRAME_RATES :
            self.get_logger().error(f'Invalid frame rate: {frame_rate}. Valid options are: {", ".join(map(str, FRAME_RATES))}.')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        self.image_type = image_type
        self.frame_rate = frame_rate
        self.get_logger().info('init... ')
        self.get_logger().info(f'image_type : {self.image_type}')
        self.get_logger().info(f'frame_rate : {self.frame_rate}')



def main(args = None):
    rclpy.init(args = args)
    pub = Pub()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()