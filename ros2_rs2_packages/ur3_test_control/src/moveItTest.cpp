#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create a node with automatic parameter declaration
    auto node = rclcpp::Node::make_shared("moveIt_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    RCLCPP_INFO(node->get_logger(), "Starting MoveIt test node...");
    
    // Define your planning group
    static const std::string PLANNING_GROUP = "ur_manipulator";
    
    RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface for group: %s", PLANNING_GROUP.c_str());
    
    // Create the MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    
    // Print info about the robot
    RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(node->get_logger(), "Available planning groups:");
    
    const std::vector<std::string>& group_names = move_group.getJointModelGroupNames();
    for (const auto& group_name : group_names) {
        RCLCPP_INFO(node->get_logger(), "  %s", group_name.c_str());
    }
    
    // Get current pose and joint values
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    std::vector<double> current_joints = move_group.getCurrentJointValues();
    
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f", 
                current_pose.pose.position.x, 
                current_pose.pose.position.y, 
                current_pose.pose.position.z);
    
    RCLCPP_INFO(node->get_logger(), "Current joint values:");
    for (size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.3f", i, current_joints[i]);
    }
    
    // Wait for everything to be ready
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Define a target pose for your end effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.4;
    
    RCLCPP_INFO(node->get_logger(), "Setting target pose: x=%.3f, y=%.3f, z=%.3f", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    move_group.setPoseTarget(target_pose);
    
    // Set planning parameters
    move_group.setMaxVelocityScalingFactor(0.2);  // 20% of maximum velocity
    move_group.setMaxAccelerationScalingFactor(0.2);  // 20% of maximum acceleration
    move_group.setPlanningTime(10.0);  // Give the planner 10 seconds
    
    // Plan the trajectory
    RCLCPP_INFO(node->get_logger(), "Planning trajectory...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto planning_result = move_group.plan(plan);
    
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Plan found! Now executing...");
        auto execution_result = move_group.execute(plan);
        
        if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Motion executed successfully!");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Motion execution failed with error code: %d", execution_result.val);
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed with error code: %d", planning_result.val);
    }
    
    RCLCPP_INFO(node->get_logger(), "Done. Shutting down...");
    rclcpp::shutdown();
    return 0;
}