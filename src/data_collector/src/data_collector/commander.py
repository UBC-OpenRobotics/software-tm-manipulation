#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
# notice one must use the moveit module provided by interbotix in order to work
from interbotix_moveit_interface_msgs.srv import MoveItPlan 
from tf_transformations import quaternion_from_euler

class MoveItPlannerNode(Node):
    def __init__(self):
        super().__init__('moveit_planner_node')

        self.get_logger().info('MoveIt Planner Node started')
        # Create a client for the MoveItPlan service
        self.client = self.create_client(MoveItPlan, 'moveit_plan')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for MoveItPlan service...')

        self.request = MoveItPlan.Request()

        # Send the plan request

        # for i in range(-5,5):
            # delta = i*0.05

        self.get_logger().info('Sending plan request...')
        self.moveTo(0.2, 0.1, 0.3, 1, 1, 1)
        # self.moveTo(0.2, 0.1, 0.1, 0.2)
        # self.moveTo(0.2, -0.1, 0.3, 0.2)
        # self.moveTo(0.2, 0.1, 0.3, 0.2)

    def moveTo(self, x=0.0, y=0.0, z=0.0, raw=0.0, pitch=0.0, yaw=0.0):
        self.send_plan_request(x, y, z, raw, pitch, yaw)
        self.get_logger().info('Waiting for plan response...')
        self.send_execute_request()

    def send_plan_request(self, x, y, z, raw_i, pitch_i, yaw_i):
        """Send a plan request with a predefined target pose."""
        # Set command type to PLANNING
        self.request.cmd = MoveItPlan.Request.CMD_PLAN_POSITION  

        # Define a target pose for the end-effector
        target_pose = Pose()
        target_pose.position.x = x 
        target_pose.position.y = y
        target_pose.position.z = z

        # Convert roll, pitch, yaw (RPY) to quaternion
        roll, pitch, yaw = raw_i, pitch_i, yaw_i
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        target_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Assign pose to the request
        self.request.ee_pose = target_pose
        
        # Call the service asynchronously
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.plan_response_callback)

    def send_execute_request(self):
        """Send a execute request to execute the planned motion"""

        #Set command type to execute
        self.request.cmd = MoveItPlan.Request.CMD_EXECUTE

        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.execute_done_callback)

    def plan_response_callback(self, future):
        """Handle MoveIt planning response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Planning successful!')
            else:
                self.get_logger().warn('Planning failed!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def execute_done_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Executed successful')
            else:
                self.get_logger().warn('Execution failed!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main():
    rclpy.init()
    node = MoveItPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
