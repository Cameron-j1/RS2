#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"

//for markers
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

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

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
                "camera_markers_auto", 10,
                std::bind(&RobotKinematics::marker_callback, this, std::placeholders::_1));

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
                std::this_thread::sleep_for(std::chrono::seconds(3));
                // Raise the shit up
                tempPosition.position.z = operation_height;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                // WE HAVE TO FIND THE X AND Y OF THE DROPPING POSITION
                tempPosition.position.x = -0.292;
                tempPosition.position.y = 0.290;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(true);
                std::this_thread::sleep_for(std::chrono::seconds(3));
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
                std::this_thread::sleep_for(std::chrono::seconds(3));
                tempPosition.position.z = operation_height;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.x = goal.first;
                tempPosition.position.y = goal.second;
                RCLCPP_INFO(this->get_logger(), "xEnd: %.3f%% and yEnd: %.3f%%", goal.first, goal.second);
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.z = pickupHeight;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(true);
                std::this_thread::sleep_for(std::chrono::seconds(3));
                tempPosition.position.z = operation_height;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
            }

            // moveToJointAngles(camera_view_jangle.at(0), camera_view_jangle.at(1),camera_view_jangle.at(2),camera_view_jangle.at(3),camera_view_jangle.at(4),camera_view_jangle.at(5));

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
            col = abs(col-7);
        
            // Convert rank to row index (0-based: '1' = 0, '8' = 7)
            double row = rank - '1';
            if (row < 0 || row > 7) {
                throw std::invalid_argument("Rank must be between '1' and '8'");
            }

            //measured from board
            double H8_X = 0.11436;
            double A1_Y = -0.469443135;

            double x_final = H8_X - col*(SQUARE_SIZE/1000);

            double y_final = A1_Y + row*(SQUARE_SIZE/1000);

            return robotReadToControlFrame({x_final, y_final}); //reordered here for transform
        }

        std::pair<double, double> robotReadToControlFrame(std::pair<double, double> in){
            in.first = -in.first;
            in.second = -in.second;
            // in.first = in.first + 0.15;
            return in;
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
        int markerNum = 50;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_publisher_;
        std::vector<double> camera_view_jangle = {
            58.92*(M_PI/180),
            -89.71*(M_PI/180),
            54.74*(M_PI/180),
            -55.06*(M_PI/180),
            -89.81*(M_PI/180),
            330.39*(M_PI/180),
        };

        std::unordered_map<char, double> pieceHeight = {
            {'p', 0.165}, {'r', 0.1793}, {'n', 0.1793},
            {'b', 0.1783-0.005}, {'q', 0.1848}, {'k', 0.1877},
            {'P', 0.165}, {'R', 0.1793}, {'N', 0.1793},
            {'B', 0.1783-0.005}, {'Q', 0.1848}, {'K', 0.1877}
        };

        //bring in markers for aruco positions
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::map<int, geometry_msgs::msg::TransformStamped> marker_transforms_;

        

        void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received marker array with %zu markers", msg->markers.size());
            
            for (const auto& marker : msg->markers) {
            int marker_id = marker.id;
            std::string frame_id = marker.header.frame_id;
            
            RCLCPP_INFO(this->get_logger(), "Marker ID: %d, Frame ID: %s", marker_id, frame_id.c_str());
            
            // Try to get the transform for this marker
            try {
                // You may need to adjust these frame names according to your setup
                // Here we assume we want to transform from marker frame to "base_link"
                std::string target_frame = "base_link";
                std::string source_frame = frame_id;
                
                geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                
                // Store the transform in the map, associated with the marker ID
                marker_transforms_[marker_id] = transform_stamped;
                
                RCLCPP_INFO(this->get_logger(), 
                        "Transform for marker %d: [%f, %f, %f] [%f, %f, %f, %f]",
                        marker_id,
                        transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y,
                        transform_stamped.transform.translation.z,
                        transform_stamped.transform.rotation.x,
                        transform_stamped.transform.rotation.y,
                        transform_stamped.transform.rotation.z,
                        transform_stamped.transform.rotation.w);
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform from %s to base_link: %s", 
                        frame_id.c_str(), ex.what());
            }
            }
        }
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
