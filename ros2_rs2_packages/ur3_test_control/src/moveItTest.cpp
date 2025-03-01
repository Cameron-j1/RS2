#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <utility>
#include <string>
#include <stdexcept>
#include <cmath>

#define SQUARE_SIZE 32.0
// Initial joint pose: 0, -90, 90, -90, -90, 0

class RobotKinematics : public rclcpp::Node {
    public:
        RobotKinematics() : Node("robot_kinematics") {
            chess_sub = this->create_subscription<std_msgs::msg::String>(
                "/chess_moves", 10,
                std::bind(&RobotKinematics::chess_topic_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Kinematic node started");
        }

        void setMoveGroup(moveit::planning_interface::MoveGroupInterface* mg) {
            move_group_ptr = mg;
        }
    
    private:
        void chess_topic_callback(const std_msgs::msg::String::SharedPtr msg) {
            std::string msgData = msg->data;
            std::pair<double, double> currentPiece = chessToGridCenter(msgData[0], msgData[1]), goal = chessToGridCenter(msgData[2], msgData[3]);
            RCLCPP_INFO(this->get_logger(), "Stockfish move: '%s'", msgData.c_str());
            maneuver(currentPiece, goal, msgData[4]);
        }

        void maneuver(std::pair<double, double> cur, std::pair<double, double> goal, char moveType) {
            std::vector<geometry_msgs::msg::Pose> points = {};
            geometry_msgs::msg::Pose tempPosition;
            tempPosition.position.z = operation_height;
            tempPosition.orientation.x = 1.0;
            tempPosition.orientation.y = 0.0;
            tempPosition.orientation.z = 0.0;
            tempPosition.orientation.w = 0.0;
            if (moveType == 'n') {
                tempPosition.position.x = cur.first;
                tempPosition.position.y = cur.second;
                RCLCPP_INFO(this->get_logger(), "xStart: %.3f%% and yStart: %.3f%%", cur.first, cur.second);
                points.push_back(tempPosition);
                tempPosition.position.z = pickupHeight;
                points.push_back(tempPosition);
                tempPosition.position.z = operation_height;
                points.push_back(tempPosition);
                tempPosition.position.x = goal.first;
                tempPosition.position.y = goal.second;
                RCLCPP_INFO(this->get_logger(), "xEnd: %.3f%% and yEnd: %.3f%%", goal.first, goal.second);
                points.push_back(tempPosition);
                tempPosition.position.z = pickupHeight;
                points.push_back(tempPosition);
                tempPosition.position.z = operation_height;
                points.push_back(tempPosition);
            }
            
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = move_group_ptr->computeCartesianPath(points, 0.01, 0.0, trajectory);
            if (fraction >= 0.95) {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group_ptr->execute(plan);
                RCLCPP_INFO(this->get_logger(), "chess move successful");
            } else {
                RCLCPP_WARN(this->get_logger(), "Path only %.2f%% complete", fraction * 100.0);
            }
        }

        std::pair<double, double> chessToGridCenter(char file, char rank) {
            // Convert file to column index (0-based: 'a' = 0, 'h' = 7)
            double col = file - 'a';
            if (col < 0 || col > 7) {
                throw std::invalid_argument("File must be between 'a' and 'h'");
            }
        
            // Convert rank to row index (0-based: '1' = 0, '8' = 7)
            double row = rank - '1';
            if (row < 0 || row > 7) {
                throw std::invalid_argument("Rank must be between '1' and '8'");
            }
            
            // Calculate bottom-left corner of the square
            double x = col * SQUARE_SIZE; // e.g., col 0 -> 0, col 1 -> 32
            double y = row * SQUARE_SIZE; // e.g., row 0 -> 0, row 1 -> 32
        
            // Return center of the square
            return {(x + SQUARE_SIZE / 2)/1000, (y + SQUARE_SIZE / 2)/1000}; // e.g., [16, 16] for "a1"
        }

        // Private variables and objects
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chess_sub;
        moveit::planning_interface::MoveGroupInterface* move_group_ptr;
        double operation_height = 0.15, pickupHeight = 0.05;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotKinematics>();

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    node->setMoveGroup(&move_group);
    RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(node->get_logger(), "Available planning groups:");
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");

    rclcpp::sleep_for(std::chrono::seconds(2));
    move_group.setMaxVelocityScalingFactor(0.2);  // 20% of maximum velocity
    move_group.setMaxAccelerationScalingFactor(0.2);  // 20% of maximum acceleration
    move_group.setPlanningTime(10.0);  // Give the planner 10 seconds
    // Initial pose of the robot
    move_group.setJointValueTarget({0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto planning_result = move_group.plan(plan);
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Moved to joint angles successfully");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute joint space plan");
        }
    }
            
    rclcpp::Rate rate(100); // 100 Hz loop

    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // Process any pending callbacks
        rate.sleep();            // Control the loop rate
    }
    // // Create a node with automatic parameter declaration
    // auto node = rclcpp::Node::make_shared("moveIt_test",
    //     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    // RCLCPP_INFO(node->get_logger(), "Starting MoveIt test node...");
    
    // // Define your planning group
    // static const std::string PLANNING_GROUP = "ur_manipulator";
    
    // RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface for group: %s", PLANNING_GROUP.c_str());
    
    // // Create the MoveGroupInterface
    // moveit::planning_interface::MoveGroupInterface (node, PLANNING_GROUP);
    
    // // Print info about the robot
    // RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
    // RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    // RCLCPP_INFO(node->get_logger(), "Available planning groups:");
    
    // const std::vector<std::string>& group_names = move_group.getJointModelGroupNames();
    // for (const auto& group_name : group_names) {
    //     RCLCPP_INFO(node->get_logger(), "  %s", group_name.c_str());
    // }
    
    // // Get current pose and joint values
    // geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    // std::vector<double> current_joints = move_group.getCurrentJointValues();
    
    // RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f, y=%.3f, z=%.3f", 
    //             current_pose.pose.position.x, 
    //             current_pose.pose.position.y, 
    //             current_pose.pose.position.z);
    
    // RCLCPP_INFO(node->get_logger(), "Current joint values:");
    // for (size_t i = 0; i < current_joints.size(); ++i) {
    //     RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.3f", i, current_joints[i]);
    // }
    
    // // Wait for everything to be ready
    // RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    // rclcpp::sleep_for(std::chrono::seconds(2));
    
    // // Define a target pose for your end effector
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.orientation.w = 1.0;
    // target_pose.position.x = 0.4;
    // target_pose.position.y = 0.2;
    // target_pose.position.z = 0.4;
    
    // RCLCPP_INFO(node->get_logger(), "Setting target pose: x=%.3f, y=%.3f, z=%.3f", 
    //             target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    // move_group.setPoseTarget(target_pose);
    
    // // Set planning parameters
    // move_group.setMaxVelocityScalingFactor(0.2);  // 20% of maximum velocity
    // move_group.setMaxAccelerationScalingFactor(0.2);  // 20% of maximum acceleration
    // move_group.setPlanningTime(10.0);  // Give the planner 10 seconds
    
    // // Plan the trajectory
    // RCLCPP_INFO(node->get_logger(), "Planning trajectory...");
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // auto planning_result = move_group.plan(plan);
    
    // if (planning_result == moveit::core::MoveItErrorCode::SUCCESS)
    // {
    //     RCLCPP_INFO(node->get_logger(), "Plan found! Now executing...");
    //     auto execution_result = move_group.execute(plan);
        
    //     if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
    //         RCLCPP_INFO(node->get_logger(), "Motion executed successfully!");
    //     } else {
    //         RCLCPP_ERROR(node->get_logger(), "Motion execution failed with error code: %d", execution_result.val);
    //     }
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed with error code: %d", planning_result.val);
    // }
    
    RCLCPP_INFO(node->get_logger(), "Done. Shutting down...");
    rclcpp::shutdown();
    return 0;
}