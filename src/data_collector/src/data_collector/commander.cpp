#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("commander");
    
    moveit::planning_interface::MoveGroupInterface move_group(node, "interbotix_arm");
    
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.0;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.40;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;  // Identity quaternion
    
    RCLCPP_INFO(node->get_logger(), "Setting target pose: x=%f, y=%f, z=%f", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Motion plan was computed successfully. Executing...");
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "Motion execution complete.");
    } else {
        RCLCPP_WARN(node->get_logger(), "Motion planning failed!");
    }
    
    rclcpp::shutdown();
    return 0;
}