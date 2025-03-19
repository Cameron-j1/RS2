#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"
#include <utility>
#include <string>
#include <stdexcept>
#include <cmath>
#include <unordered_map>

#define SQUARE_SIZE 35.0
// Initial joint pose: 0, -90, 90, -90, -90, 0

class RobotKinematics : public rclcpp::Node {
    public:
        RobotKinematics() : Node("robot_kinematics") {
            chess_sub = this->create_subscription<std_msgs::msg::String>(
                "/chess_moves", 10,
                std::bind(&RobotKinematics::chess_topic_callback, this, std::placeholders::_1));
            
            marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
            RCLCPP_INFO(this->get_logger(), "Kinematic node started");

            servo_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/servo_control", 10);

            // moveToCameraViewJ();
        }

        void publishServoState(bool state) {
            auto message = std_msgs::msg::Bool();
            message.data = state;
            servo_publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: %s", state ? "true" : "false");
        }
    

        void setMoveGroup(moveit::planning_interface::MoveGroupInterface* mg) {
            move_group_ptr = mg;
        }

        void moveToCameraViewJ(){
            RCLCPP_INFO(this->get_logger(), "Attempting to move to camera view position...");
            moveToJointAngles(camera_view_jangle.at(0), camera_view_jangle.at(1), camera_view_jangle.at(2), camera_view_jangle.at(3), camera_view_jangle.at(4), camera_view_jangle.at(5));
        }

        void moveToJointAngles(double j1, double j2, double j3, double j4, double j5, double j6) {
            if (move_group_ptr == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "MoveGroup pointer is not initialized");
                return;
            }
            
            // Set the joint target values
            std::vector<double> joint_positions = {j1, j2, j3, j4, j5, j6};
            move_group_ptr->setJointValueTarget(joint_positions);
            
            // Plan the motion
            RCLCPP_INFO(this->get_logger(), "Planning motion to joint angles [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        j1, j2, j3, j4, j5, j6);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto planning_result = move_group_ptr->plan(plan);
            
            // Execute if planning was successful
            if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Planning successful, executing motion...");
                
                auto execution_result = move_group_ptr->execute(plan);
                if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Joint motion executed successfully");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute joint motion plan");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan joint motion");
            }
        }

    private:
        void chess_topic_callback(const std_msgs::msg::String::SharedPtr msg) {
            std::string msgData = msg->data;
            if (msgData.length() < 9) { // if greater than 9 means it's the fen string for the camera node
                std::pair<double, double> currentPiece = chessToGridCenter(msgData[0], msgData[1]), goal = chessToGridCenter(msgData[2], msgData[3]);
                RCLCPP_INFO(this->get_logger(), "Stockfish move: '%s'", msgData.c_str());
                if (msgData.length() == 6) // normal move
                    maneuver(currentPiece, goal, msgData[4], pieceHeight[msgData[5]], 0);
                else if (msgData.length() == 7)
                    maneuver(currentPiece, goal, msgData[4], pieceHeight[msgData[5]], pieceHeight[msgData[6]]);
            }
        }

        void maneuver(std::pair<double, double> cur, std::pair<double, double> goal, char moveType, double pickupHeight, double pickupHeightCap) {
            std::vector<geometry_msgs::msg::Pose> points = {};
            geometry_msgs::msg::Pose tempPosition;
            tempPosition.position.z = operation_height;
            tempPosition.orientation.x = 1.0;
            tempPosition.orientation.y = 0.0;
            tempPosition.orientation.z = 0.0;
            tempPosition.orientation.w = 0.0;
            // remove the captured piece from the board
            if (moveType == 'x') {
                tempPosition.position.x = goal.first;
                tempPosition.position.y = goal.second;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                RCLCPP_INFO(this->get_logger(), "Remove captured: xStart: %.3f%% and yStart: %.3f%% and zPickUp: %.3f%%", cur.first, cur.second, pickupHeightCap);
                tempPosition.position.z = pickupHeightCap;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(false);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // Raise the shit up
                tempPosition.position.z = operation_height;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                // WE HAVE TO FIND THE X AND Y OF THE DROPPING POSITION
                // tempPosition.position.x = goal.first;
                // tempPosition.position.y = goal.second;
                // moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(true);
            }   
            // Here, we play the damn piece
            if (moveType == 'n' || moveType == 'x') {
                tempPosition.position.x = cur.first;
                tempPosition.position.y = cur.second;
                RCLCPP_INFO(this->get_logger(), "xStart: %.3f%% and yStart: %.3f%% and zPickUp: %.3f%%", cur.first, cur.second, pickupHeight);
                publish_point(cur.first, cur.second, pickupHeight, 1.0, 0.0, 0.0);
                publish_point(goal.first, goal.second, pickupHeight, 0.0, 1.0, 0.0);
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.z = pickupHeight;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(false);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                tempPosition.position.z = operation_height;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.x = goal.first;
                tempPosition.position.y = goal.second;
                RCLCPP_INFO(this->get_logger(), "xEnd: %.3f%% and yEnd: %.3f%%", goal.first, goal.second);
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.z = pickupHeight;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(true);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                tempPosition.position.z = operation_height;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
            }
        }

        bool moveStraightToPoint(std::vector<geometry_msgs::msg::Pose> tempPosition, double vel, double acc) {
            move_group_ptr->setMaxVelocityScalingFactor(vel);  // 20% of maximum velocity
            move_group_ptr->setMaxAccelerationScalingFactor(acc);  // 20% of maximum acceleration
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = move_group_ptr->computeCartesianPath(tempPosition, 0.01, 0.0, trajectory);
            if (fraction >= 0.95) {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group_ptr->execute(plan);
                RCLCPP_INFO(this->get_logger(), "chess move successful");
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Path only %.2f%% complete", fraction * 100.0);
                return false;
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
            double x = col * SQUARE_SIZE;
            double y = row * SQUARE_SIZE;
        
            // Calculate center of the square (before rotation)
            double x_center = x + SQUARE_SIZE / 2;
            double y_center = y + SQUARE_SIZE / 2;
        
            // Step 1: Translate so the board's center is at (0, 0)
            double board_center_x = 4 * SQUARE_SIZE; // Center of 8x8 board
            double board_center_y = 4 * SQUARE_SIZE;
            double x_translated = x_center - board_center_x;
            double y_translated = y_center - board_center_y;
        
            // Step 2: Apply 90-degree counterclockwise rotation (x', y') = (-y, x)
            double x_rotated = -y_translated;
            double y_rotated = x_translated;
        
            // Step 3: Translate back to original position
            double x_final = x_rotated + board_center_x;
            double y_final = y_rotated + board_center_y;
        
            // Scale to meters (as in your original function)
            return {-((y_final / 1000) - 0.256/2), (x_final / 1000) + 0.2};
        }

        void publish_point(double x, double y, double z, double r, double g, double b) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world"; // Reference frame (change as needed)
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "point";
            marker.id = markerNum++;
            marker.type = visualization_msgs::msg::Marker::SPHERE; // Display as a sphere
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = x; // Example x-coordinate
            marker.pose.position.y = y; // Example y-coordinate
            marker.pose.position.z = z; // Example z-coordinate
            marker.pose.orientation.w = 1.0; // No rotation (identity quaternion)

            marker.scale.x = 0.02; // Size in meters
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.r = r; // Red
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 1.0; // Alpha (opacity)
            marker_pub->publish(marker);
        }

        // Private variables and objects
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chess_sub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        moveit::planning_interface::MoveGroupInterface* move_group_ptr;

        // calculate the height of other pieces based on the height of the pawn
        double operation_height = 0.15 + 0.1;//, pickupHeight = 0.05 + 0.1+0.015;
        int markerNum = 0;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_publisher_;
        std::vector<double> camera_view_jangle = {
            -74.63*(M_PI/180),
            -108.28*(M_PI/180),
            -4.65*(M_PI/180),
            -156.51*(M_PI/180),
            89.47*(M_PI/180),
            15.37*(M_PI/180),
        };

        std::unordered_map<char, double> pieceHeight = {
            {'p', 0.165}, {'r', 0.1702}, {'n', 0.1793},
            {'b', 0.1783}, {'q', 0.1848}, {'k', 0.1877},
            {'P', 0.165}, {'R', 0.1702}, {'N', 0.1793},
            {'B', 0.1783}, {'Q', 0.1848}, {'K', 0.1877}
        };
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
    
    // Wait a bit for ROS to be fully initialized
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    move_group.setMaxVelocityScalingFactor(0.05);  // 20% of maximum velocity
    move_group.setMaxAccelerationScalingFactor(0.02);  // 20% of maximum acceleration
    move_group.setPlanningTime(10.0);  // Give the planner 10 seconds
    // Initial pose of the robot
    move_group.setJointValueTarget({M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto planning_result = move_group.plan(plan);
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Moved to joint angles successfully");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute joint space plan");
        }
    }
            
    while (rclcpp::ok()) {
        rclcpp::spin(node); // Process any pending callbacks
    }
    
    RCLCPP_INFO(node->get_logger(), "Done. Shutting down...");
    rclcpp::shutdown();
    return 0;
}