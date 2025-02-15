#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import pickle
import os
import time
import signal
import pandas as pd

class Collector(Node):
    def __init__(self):
        super().__init__('collector')
        self.joints = self.create_subscription(
            JointState,
            '/rx150/joint_states',
            self.joint_callback,
            10
        )
        self.joints

        self.positions = []
        self.date = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())

        self.file_path = os.path.expanduser(f'~/interbotix_ws/src/data_collector/positions/{self.date}.pkl')
        self.file_path_csv = os.path.expanduser(f'~/interbotix_ws/src/data_collector/positions/{self.date}.csv')

        # self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Collector has started.')

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello, ROS 2!'
    #     self.publisher_.publish(msg) 
    #     self.get_logger().info(f'Publishing: {msg.data}')

    def joint_callback(self, data):
        self.get_logger().info(f'Received: {data.position}')
        self.positions.append(data.position)

    def save_and_exit(self, signum, frame):
        # Prevent calling shutdown again if already done
        df = pd.DataFrame(self.positions, columns=['pos1', 'pos2', 'pos3', 'pos4', 'pos5', 'pos6', 'pos7', 'pos8'])

        self.get_logger().info(f"Saving {len(self.positions)} joint positions entries to {self.file_path}")
        try:
            with open(self.file_path, 'wb') as f:
                pickle.dump(self.positions, f)
            df.to_csv(self.file_path_csv, index=False)
            self.get_logger().info(f"Saved")
        except Exception as e:
            self.get_logger().info({e})
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Collector()

    signal.signal(signal.SIGINT, node.save_and_exit)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()