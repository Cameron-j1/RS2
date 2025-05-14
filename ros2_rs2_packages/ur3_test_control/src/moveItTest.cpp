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

//for collision
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <Eigen/Geometry>

//for joint state
#include <sensor_msgs/msg/joint_state.hpp>

//for transformation matrices
#include <Eigen/Dense>
#include <Eigen/Geometry>

//for button press control with blocking button
#include <thread>
#include <atomic>
#include <mutex>

#include <utility>
#include <string>
#include <stdexcept>
#include <cmath>
#include <unordered_map>

#define SQUARE_SIZE 35.0

//speed variables
#define MAX_VEL_CARTESIAN 0.05
#define MAX_ACCEL_CARTESIAN 0.02

#define MAX_VEL_JOINT_TARGET 0.15
#define MAX_ACCEL_JOINT_TARGET 0.06


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

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&RobotKinematics::jointStateCallback, this, std::placeholders::_1));    

            button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "button_state", 10,
                std::bind(&RobotKinematics::button_callback, this, std::placeholders::_1));

            takeImage = this->create_publisher<std_msgs::msg::Bool>("/take_image", 10);

            player_turn_pub = this->create_publisher<std_msgs::msg::Bool>("/player_turn_bool", 10);

            estop_engaged_pub = this->create_publisher<std_msgs::msg::Bool>("/estop_flag", 10);

            board_oor_pub = this->create_publisher<std_msgs::msg::Bool>("/board_oor", 10);

            robot_stationary_ = false;
            pickup_dropoff_wait_ = 1.5; //seconds
            H1_transform_ = Eigen::Matrix4d::Identity();

            //intialise to default values for operation without arucos
            H1_X = 0.11436;
            H1_Y = -0.469443135;

            moveToJointAngles(camera_view_jangle.at(0), camera_view_jangle.at(1),camera_view_jangle.at(2),camera_view_jangle.at(3),camera_view_jangle.at(4),camera_view_jangle.at(5));
        }

        void publishServoState(bool state) {
            auto message = std_msgs::msg::Bool();
            message.data = state;
            servo_publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: %s", state ? "true" : "false");
        }

        double getYawFromQuaternion(double x, double y, double z, double w) {
            Eigen::Quaterniond q(w, x, y, z); // Note: Eigen uses w, x, y, z order
            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order
            return euler[0]; // yaw
        }        

        void setMoveGroup(moveit::planning_interface::MoveGroupInterface* mg) {
            std::lock_guard<std::mutex> lock(moveit_mutex);
            move_group_ptr = mg;
        }

    private:
        void moveToCameraViewJ(){
            RCLCPP_INFO(this->get_logger(), "Attempting to move to camera view position...");
            moveToJointAngles(camera_view_jangle.at(0), camera_view_jangle.at(1), camera_view_jangle.at(2), camera_view_jangle.at(3), camera_view_jangle.at(4), camera_view_jangle.at(5));
        }

        bool moveToJointAngles(double j1, double j2, double j3, double j4, double j5, double j6) {
            if (move_group_ptr == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "[moveToJointAnglesInThread] MoveGroup pointer is not initialized");
                return false;
            }

            std::vector<double> joint_positions = {j1, j2, j3, j4, j5, j6};
            int count = 0;
            int print_freq = 50;

            while (rclcpp::ok()) {
                // ─── Wait for Deadman ─────────────────────────────────────────────
                while (!button_state && rclcpp::ok()) {
                    count++;
                    if (count % print_freq == 0) {
                        RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Waiting for deadman switch...");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }

                // ─── Plan ─────────────────────────────────────────────────────────
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                {
                    std::unique_lock<std::mutex> lock(moveit_mutex);
                    RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Mutex acquired, planning...");

                    move_group_ptr->setStartStateToCurrentState();
                    move_group_ptr->setMaxVelocityScalingFactor(MAX_VEL_JOINT_TARGET);
                    move_group_ptr->setMaxAccelerationScalingFactor(MAX_ACCEL_JOINT_TARGET);

                    move_group_ptr->setJointValueTarget(joint_positions);
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Planning motion to joint angles [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                                j1, j2, j3, j4, j5, j6);

                    auto planning_result = move_group_ptr->plan(plan);
                    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_WARN(this->get_logger(), "[moveToJointAnglesInThread] Planning failed.");
                        return false;
                    }
                }

                // ─── Execute ──────────────────────────────────────────────────────
                move_group_ptr->asyncExecute(plan);
                auto start = std::chrono::steady_clock::now();
                while (rclcpp::ok()) {
                    if (!button_state) {
                        RCLCPP_WARN(this->get_logger(), "[moveToJointAnglesInThread] Deadman switch released, stopping...");
                        move_group_ptr->stop();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        break;  // Retry planning and execution
                    }

                    //check if the robot has finished it's move
                    if(robot_stationary_ && std::chrono::steady_clock::now() - start > std::chrono::seconds(2)){
                        //condition met for move complete
                        RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] SUCCESS move complete");
                        //delay for safety to ensure that the robot has actually stopped and planning for the next move won't fail
                        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        return true;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Retrying motion after interruption...");
            }
            return false;  // Should not be reached unless ROS is shutting down
        }


        void chess_topic_callback(const std_msgs::msg::String::SharedPtr msg) {
            std::string msgData = msg->data;
            if (msgData.length() < 12) { // if greater than 9 means it's the fen string for the camera node
                std::pair<double, double> currentPiece = chessToGridCenter(msgData[0], msgData[1]), goal = chessToGridCenter(msgData[2], msgData[3]);
                RCLCPP_INFO(this->get_logger(), "Stockfish move: '%s'", msgData.c_str());
                if(msgData.length() == 6){ // normal move
                    maneuver(currentPiece, goal, msgData[4], pieceHeight[msgData[5]], 0);
                }
                if(msgData.length() == 7 || msgData.length() == 8){ // capture move
                    maneuver(currentPiece, goal, msgData[4], pieceHeight[msgData[5]], pieceHeight[msgData[6]]);
                }
                if(msgData.length() == 11){ //castling
                    //example castling string e8g8h8f8ckr. first move 'e8g8' second move 'h8f8' move type 'c' first move peice 'k' second move peice 'r'
                    maneuver(currentPiece, goal, msgData[8], pieceHeight[msgData[9]], 0);
                    currentPiece = chessToGridCenter(msgData[4], msgData[5]);
                    goal = chessToGridCenter(msgData[6], msgData[7]);
                    maneuver(currentPiece, goal, msgData[8], pieceHeight[msgData[10]], 0);
                }
            }
        }

        void maneuver(std::pair<double, double> cur, std::pair<double, double> goal, char moveType, double pickupHeight, double pickupHeightCap){
            std::thread([this, cur, goal, moveType, pickupHeight, pickupHeightCap]() {
                this->maneuverInThread(cur, goal, moveType, pickupHeight, pickupHeightCap);
            }).detach();
        }

        void maneuverInThread(std::pair<double, double> cur, std::pair<double, double> goal, char moveType, double pickupHeight, double pickupHeightCap) {
            std::vector<geometry_msgs::msg::Pose> points = {};
            geometry_msgs::msg::Pose tempPosition;
            tempPosition.position.z = operation_height;
            tempPosition.orientation.x = 1.0;
            tempPosition.orientation.y = 0.0;
            tempPosition.orientation.w = 0.0;

            isTakeImage = false;
            auto msg = std_msgs::msg::Bool();
            msg.data = false;    // or false, depending on your logic
            player_turn_pub->publish(msg);

            RCLCPP_INFO(this->get_logger(), "[maneuver] angle move to viewing position");
            moveToJointAngles(-1.525+M_PI, -1.647, 0.291, -0.390, -1.549, 6.215);

            // remove the captured piece from the board
            if (moveType == 'x') {
                RCLCPP_INFO(this->get_logger(), "[maneuver] maneuver for capture move");
                tempPosition.position.x = goal.first;
                tempPosition.position.y = goal.second;
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                RCLCPP_INFO(this->get_logger(), "[maneuver] Remove captured: xStart: %.3f%% and yStart: %.3f%% and zPickUp: %.3f%%", cur.first, cur.second, pickupHeightCap);
                tempPosition.position.z = pickupHeightCap;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight down to pickup peice to be captured");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(true);
                std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                // Raise the shit up
                tempPosition.position.z = operation_height;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight up to pickup peice being captured");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                // WE HAVE TO FIND THE X AND Y OF THE DROPPING POSITION
                tempPosition.position.x = -0.292;
                tempPosition.position.y = 0.290;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to drop off position");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(false);
                std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
            }   
            // Here, we play the damn piece
            if (moveType == 'n' || moveType == 'x' || moveType == 'c'){
                tempPosition.position.x = cur.first;
                tempPosition.position.y = cur.second;
                RCLCPP_INFO(this->get_logger(), "xStart: %.3f%% and yStart: %.3f%% and zPickUp: %.3f%%", cur.first, cur.second, pickupHeight);
                publish_point(cur.first, cur.second, pickupHeight, 1.0, 0.0, 0.0);
                publish_point(goal.first, goal.second, pickupHeight, 0.0, 1.0, 0.0);
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice position");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.z = pickupHeight;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice height");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(true);
                std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                tempPosition.position.z = operation_height;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice position");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.x = goal.first;
                tempPosition.position.y = goal.second;
                RCLCPP_INFO(this->get_logger(), "xEnd: %.3f%% and yEnd: %.3f%%", goal.first, goal.second);
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice destination");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                tempPosition.position.z = pickupHeight;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to target peice destination height");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
                publishServoState(false);
                std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                tempPosition.position.z = operation_height;
                RCLCPP_INFO(this->get_logger(), "[maneuver] move straight back to operation height peice move done");
                moveStraightToPoint({tempPosition}, 0.05, 0.05);
            }

            RCLCPP_INFO(this->get_logger(), "[maneuver] angle move to viewing position");
            moveToJointAngles(1.544, -2.060, 0.372, -0.108, -1.651, -6.233); 
            isTakeImage = true;
            auto msg = std_msgs::msg::Bool();
            msg.data = true;    // or false, depending on your logic
            player_turn_pub->publish(msg);

        }

        bool moveStraightToPoint(std::vector<geometry_msgs::msg::Pose> tempPosition, double vel, double acc) {
            int count = 0;
            int print_freq = 50;

            while (rclcpp::ok()) {
                // ─── Wait for Deadman ─────────────────────────────────────────────
                while (!button_state && rclcpp::ok()) {
                    count++;
                    auto msg = std_msgs::msg::Bool();
                    msg.data = true;    // or false, depending on your logic
                    estop_engaged_pub->publish(msg);
                    if (count % print_freq == 0) {
                        RCLCPP_INFO(this->get_logger(), "[moveStraightToPoint] Waiting for deadman switch...");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }

                // ─── Plan ─────────────────────────────────────────────────────────
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction;

                {
                    std::unique_lock<std::mutex> lock(moveit_mutex);
                    RCLCPP_INFO(this->get_logger(), "[moveStraightToPoint] Mutex acquired, planning...");

                    move_group_ptr->setStartStateToCurrentState();
                    move_group_ptr->setMaxVelocityScalingFactor(vel);
                    move_group_ptr->setMaxAccelerationScalingFactor(acc);

                    fraction = move_group_ptr->computeCartesianPath(tempPosition, 0.01, 0.0, trajectory);
                    if (fraction < 0.95) {
                        RCLCPP_WARN(this->get_logger(), "Path only %.2f%% complete", fraction * 100.0);
                        return false;
                    }

                    plan.trajectory_ = trajectory;
                }
                auto msg = std_msgs::msg::Bool();
                msg.data = false;    // or false, depending on your logic
                estop_engaged_pub->publish(msg);

                // ─── Execute ──────────────────────────────────────────────────────
                move_group_ptr->asyncExecute(plan);
                auto start = std::chrono::steady_clock::now();

                while (rclcpp::ok()) {
                    if (!button_state) {
                        RCLCPP_WARN(this->get_logger(), "[moveStraightToPoint] Deadman switch released, stopping...");
                        
                        move_group_ptr->stop();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        break;  // Retry from outer loop
                    }

                    //check if the robot has finished it's move
                    if(robot_stationary_ && std::chrono::steady_clock::now() - start > std::chrono::seconds(2)){
                        //condition met for move complete
                        RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] SUCCESS move complete");
                        //delay for safety to ensure that the robot has actually stopped and planning for the next move won't fail
                        // std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        return true;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }

                RCLCPP_INFO(this->get_logger(), "[moveStraightToPoint] Retrying motion after interruption...");
            }

            return false;  // Shouldn't hit this unless ROS is shutting down
        }


        std::pair<double, double> chessToGridCenter(char file, char rank) {
            // Convert file to column index (0-based: 'a' = 0, 'h' = 7)
            double col = file - 'a';
            if (col < 0 || col > 7) {
                throw std::invalid_argument("File must be between 'a' and 'h'");
            }
            //'h' = 0 'a' = 7
            col = abs(col-7);

            // Convert rank to row index (0-based: '1' = 0, '8' = 7)
            double row = rank - '1';
            if (row < 0 || row > 7) {
                throw std::invalid_argument("Rank must be between '1' and '8'");
            }

            //i had to flip this for some reason on friday - watch might be temperamental
            double dx = col * (SQUARE_SIZE / 1000.0);
            double dy =  -row * (SQUARE_SIZE / 1000.0);

            if(board_yaw < 1.57){
                dx = -col * (SQUARE_SIZE / 1000.0);
                dy =  row * (SQUARE_SIZE / 1000.0);
            }

            // Apply 2D rotation to account for yaw
            double x_rotated = dx * cos(board_yaw) - dy * sin(board_yaw);
            double y_rotated = dx * sin(board_yaw) + dy * cos(board_yaw);

            // Final global position
            double x_final = H1_X + x_rotated;
            double y_final = H1_Y + y_rotated;



            //yaw debug messages
            // RCLCPP_INFO(this->get_logger(), "YAW!!!: %.5f", board_yaw);
            // RCLCPP_INFO(this->get_logger(), "x_rotated: %.5f ", x_rotated);
            // RCLCPP_INFO(this->get_logger(), "y_rotated: %.5f ", y_rotated);
            // RCLCPP_INFO(this->get_logger(), "x_final: %.5f ", x_final);
            // RCLCPP_INFO(this->get_logger(), "y_final: %.5f ", y_final);
            // RCLCPP_INFO(this->get_logger(), "dx: %.5f ", dx);
            // RCLCPP_INFO(this->get_logger(), "dy: %.5f ", dy);

            // std::this_thread::sleep_for(std::chrono::seconds(2));

            return robotReadToControlFrame({x_final, y_final}); //reordered here for transform
        }


        Eigen::Matrix4d multiplyMatrix4d(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B) {
            Eigen::Matrix4d result = Eigen::Matrix4d::Zero();

            for (int i = 0; i < 4; ++i) {           // row of A
                for (int j = 0; j < 4; ++j) {       // column of B
                    for (int k = 0; k < 4; ++k) {   // element index
                        result(i, j) += A(i, k) * B(k, j);
                    }
                }
            }

            return result;
        }


        void logMatrix(const std::string& name, const Eigen::Matrix4d& mat) {
            std::ostringstream oss;
            oss << "\n" << mat;
            RCLCPP_INFO(this->get_logger(), "%s: %s", name.c_str(), oss.str().c_str());
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
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeImage;
        moveit::planning_interface::MoveGroupInterface* move_group_ptr;

        // calculate the height of other pieces based on the height of the pawn
        double operation_height = 0.15 + 0.1;//, pickupHeight = 0.05 + 0.1+0.015;
        int markerNum = 50;
        bool isTakeImage = true;
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
            {'p', 0.165-20/1000}, {'r', ((0.1783-0.005) + (0.165-20/1000))/2 }, {'n', 0.1793},
            {'b', 0.181}, {'q', 0.1848-50/1000 }, {'k', 0.1877},
            {'P', 0.165-20/1000}, {'R', ((0.1783-0.005) + (0.165-20/1000))/2}, {'N', 0.1793},
            {'B', 0.181}, {'Q', 0.1848-50/1000}, {'K', 0.1877}
        };

        //bring in markers for aruco positions
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::map<int, geometry_msgs::msg::TransformStamped> marker_transforms_;
        double H1_X;
        double H1_Y;
        geometry_msgs::msg::Pose camPosition;
        Eigen::Matrix4d H1_transform_;
        double board_yaw = 0;

        Eigen::Matrix4d quaternionToTransformMatrix(const Eigen::Quaterniond& q, const Eigen::Vector3d& translation){
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        
            // Convert quaternion to rotation matrix
            Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
        
            // Insert rotation and translation into 4x4 matrix
            transform.block<3,3>(0,0) = rotation;
            transform.block<3,1>(0,3) = translation;
        
            return transform;
        }
        

        void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg){
            // RCLCPP_INFO(this->get_logger(), "Received marker array with %zu markers", msg->markers.size());
            
            for (const auto& marker : msg->markers) {
                int marker_id = marker.id;

                // RCLCPP_INFO(this->get_logger(), "MARKER!! ID: %d", marker_id);
                if(marker_id == 51 && robot_stationary_){
                    board_yaw = getYawFromQuaternion(
                        marker.pose.orientation.x,
                        marker.pose.orientation.y,
                        marker.pose.orientation.z,
                        marker.pose.orientation.w);
                }

                if (marker_id == 53 && robot_stationary_) { //only read the markers if the robot is currently stationary
                    std::string frame_id = marker.header.frame_id;             

                    auto q_msg = marker.pose.orientation;
                    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
                    Eigen::Vector3d t(marker.pose.position.x,
                        marker.pose.position.y,
                        marker.pose.position.z);

                    H1_X = -marker.pose.position.x;
                    H1_Y = -marker.pose.position.y;
                    auto msg = std_msgs::msg::Bool();
                    if(H1_X > std::abs(0.15) || H1_Y > 0.37 || std::abs(board_yaw) > 0.13){
                        msg.data = true;
                        board_oor_pub->publish(msg);
                    }
                    else{
                        msg.data = false;
                        board_oor_pub->publish(msg);
                    }

                    double dx = 3.5 * (SQUARE_SIZE / 1000.0);
                    double dy = -3.5 * (SQUARE_SIZE / 1000.0);

                    if(board_yaw < 1.57){
                        dx = -3.5 * (SQUARE_SIZE / 1000.0);
                        dy =  3.5 * (SQUARE_SIZE / 1000.0);
                    }

                    // Apply 2D rotation to account for yaw
                    double x_final = -(H1_X + (dx * cos(board_yaw) - dy * sin(board_yaw)));
                    double y_final = -(H1_Y + (dx * sin(board_yaw) + dy * cos(board_yaw)));

                    // camPosition.orientation.x = 1.0;
                    // camPosition.orientation.y = 0.0;
                    // camPosition.orientation.w = 0.0;
                    // camPosition.orientation.z = 0.0;
                    camPosition.orientation.x = marker.pose.orientation.x;
                    camPosition.orientation.y = marker.pose.orientation.y;
                    camPosition.orientation.z = marker.pose.orientation.z;
                    camPosition.orientation.w = marker.pose.orientation.w;
                    
                    camPosition.position.x = x_final + 0.037143913668;//matCamPos(0, 3);
                    camPosition.position.y = y_final - 0.068065853878;//matCamPos(1, 3);
                    camPosition.position.z = 0.40406;

                }
            }
        }        

        //to detect the robot moving
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            static std::vector<double> last_positions;

            const double POSITION_EPSILON = 1e-3;  // 0.001 rad = ~0.057 degrees

            bool moving = false;

            // First time: initialize and return
            if (last_positions.empty()) {
                last_positions = msg->position;
                robot_stationary_ = true;
                RCLCPP_INFO(this->get_logger(), "Initialized joint position tracking");
                return;
            }

            // Compare current with previous positions
            size_t n = std::min(last_positions.size(), msg->position.size());
            for (size_t i = 0; i < n; ++i) {
                if (std::abs(msg->position[i] - last_positions[i]) > POSITION_EPSILON) {
                    moving = true;
                    break;
                }
            }

            robot_stationary_ = !moving;

            if (robot_stationary_) {
                // RCLCPP_INFO(this->get_logger(), "Robot is stationary.");
            } else {
                // RCLCPP_INFO(this->get_logger(), "Robot is moving.");
            }

            // Update last_positions
            last_positions = msg->position;
        }

        //joint state sub obj
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        int pickup_dropoff_wait_;

        // player turn pub for pi variables
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr player_turn_pub;

        //e-stop state pub for pi variables
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_engaged_pub;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr board_oor_pub;



        //button state variables
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
        // bool button_state;
        std::atomic<bool> button_state;
        std::atomic<bool> robot_stationary_;
        std::mutex moveit_mutex;

        void button_callback(const std_msgs::msg::Bool::SharedPtr msg){
            if(msg->data) {
                RCLCPP_WARN(this->get_logger(), "E-stop engaged (button_state == true)!");
                auto msg = std_msgs::msg::Bool();
                rclcpp::sleep_for(std::chrono::milliseconds(50));
                if (robot_stationary_ && isTakeImage) {
                    geometry_msgs::msg::Pose camPositionReal = camPosition;
                    RCLCPP_INFO(this->get_logger(), "Button pressed, movinnnnn");
                    moveToJointAngles(M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0);

                    RCLCPP_INFO(this->get_logger(), 
                    "Camera Goal Position and Orientation:\n"
                    "Orientation: x=%.4f, y=%.4f, w=%.4f\n"
                    "Position: x=%.4f, y=%.4f, z=%.4f",
                    camPositionReal.orientation.x,
                    camPositionReal.orientation.y,
                    camPositionReal.orientation.w,
                    camPositionReal.position.x,
                    camPositionReal.position.y,
                    camPositionReal.position.z);

                    moveStraightToPoint({camPositionReal}, 0.05, 0.05);
                    takeImage->publish(std_msgs::msg::Bool().set__data(true));
                    isTakeImage = false;
                }
            }
            else{
                button_state = false;
                RCLCPP_WARN(this->get_logger(), "E-stop dis-engaged (button_state == false)!");
            }
        }

};


int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotKinematics>();

    //Create the MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    // Create the PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    node->setMoveGroup(&move_group);
    RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(node->get_logger(), "Available planning groups:");
    RCLCPP_INFO(node->get_logger(), "Waiting for 2 seconds...");
    
    // Wait a bit for ROS to be fully initialized
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Add a large cube directly under the robot base
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame(); // Use the planning frame
    collision_object.id = "floor_cube";
   
    // Define the cube as 1.5m in all dimensions
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.5;  // 1.5m in x-direction
    primitive.dimensions[primitive.BOX_Y] = 1.5;  // 1.5m in y-direction
    primitive.dimensions[primitive.BOX_Z] = 1.5;  // 1.5m in z-direction
   
    // Define the pose of the cube
    // and center it at x=0, y=0
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;  // Centered at x=0
    box_pose.position.y = 0.0;  // Centered at y=0
    box_pose.position.z = -0.75;  // Position is at the center of the box, so -0.75 puts the top at z=0

    // Add the primitive and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    RCLCPP_INFO(node->get_logger(), "Adding large cube under robot base to planning scene");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Give the planning scene some time to update
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    move_group.setMaxVelocityScalingFactor(MAX_VEL_JOINT_TARGET);  // 20% of maximum velocity
    move_group.setMaxAccelerationScalingFactor(MAX_ACCEL_JOINT_TARGET);  // 20% of maximum acceleration
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
    // move_group.setJointValueTarget({-1.525+M_PI, -1.647, 0.291, -0.390, -1.549, 6.215});
    move_group.setJointValueTarget({1.544, -2.060, 0.372, -0.108, -1.651, -6.233}); 


    // move_group.setJointValueTarget({-2.0948268375792445, 0.21491748491396123, -0.019584493046142626, -1.6386211554156702, -0.06730491319765264, 1.5663700103759766});
    planning_result = move_group.plan(plan);
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Moved to joint angles successfully");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute joint space plan");
        }
    }

    move_group.setMaxVelocityScalingFactor(MAX_VEL_CARTESIAN);  // 20% of maximum velocity
    move_group.setMaxAccelerationScalingFactor(MAX_ACCEL_CARTESIAN); 
            
    const size_t num_threads = 3;  // Set the number of threads you want
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
    executor.add_node(node);
    executor.spin();

    // while (rclcpp::ok()) {
    //     rclcpp::spin(node); // Process any pending callbacks
    // }

    //remove the objects
    RCLCPP_INFO(node->get_logger(), "Removing objects from the scene");
    std::vector<std::string> object_ids;
    object_ids.push_back("floor_cube");
    planning_scene_interface.removeCollisionObjects(object_ids);
    
    RCLCPP_INFO(node->get_logger(), "Done. Shutting down...");
    rclcpp::shutdown();
    return 0;
}
