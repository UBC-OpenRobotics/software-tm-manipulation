#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
import sys

IMAGE_TYPES = [ 'rgb_frame', ]



class Sub(Node):
    def __init__(self):
        super().__init__('intel_d435_camera_sub')

        self.image_type = 'rgb_frame'
        self.mirror = False 
        self.get_params()

        self.sub = self.create_subscription(Image, self.image_type, self.rgb_frame_callback, 10)
        
        self.br_rgb = CvBridge()


    def rgb_frame_callback(self, data):
        self.get_logger().warning('Receiving RGB frame')
        current_frame = self.br_rgb.imgmsg_to_cv2(data)
        if self.mirror :
            current_frame = cv2.flip(current_frame, 1)
        cv2.imshow('RGB', current_frame)
        cv2.waitKey(1)

    def get_params(self):
        self.declare_parameter('image_type', 'rgb_frame')
        self.declare_parameter('mirror', False)
        # image_type
        image_type = self.get_parameter('image_type').get_parameter_value().string_value
        if image_type not in IMAGE_TYPES :
            self.get_logger().error(f'Invalid image type: {image_type}. Valid options are: {", ".join(IMAGE_TYPES)}.')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        # mirror
        mirror = self.get_parameter('mirror').get_parameter_value().bool_value
        if not isinstance(mirror, bool):
            self.get_logger().error(f'Invalid value for mirror: {mirror}. Expected a boolean value.')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        self.image_type = image_type
        self.mirror = mirror
        self.get_logger().info('init... ')
        self.get_logger().info(f'image_type : {self.image_type}')
        self.get_logger().info(f'image_type : {self.mirror}')



def main(args = None):
    rclpy.init(args = args)
    sub = Sub()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main':
    main()