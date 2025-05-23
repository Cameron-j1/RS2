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
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
 
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
#define MAX_VEL_CARTESIAN 0.0025
#define MAX_ACCEL_CARTESIAN 0.01
 
#define MAX_VEL_JOINT_TARGET 0.15
#define MAX_ACCEL_JOINT_TARGET 0.06
 
// Initial joint pose: 0, -90, 90, -90, -90, 0
 
class RobotKinematics : public rclcpp::Node {
    public:
        RobotKinematics() : Node("robot_kinematics") {
            // Declare and get simulation mode parameter
            this->declare_parameter("simulation_mode", false);
            simulation_mode_ = this->get_parameter("simulation_mode").as_bool();
            RCLCPP_INFO(this->get_logger(), "Simulation mode: %s", simulation_mode_ ? "true" : "false");

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

            chess_game_status_sub_ = this->create_subscription<std_msgs::msg::String>(
                    "/chess_game_status",
                    10,  // QoS history depth
                    std::bind(&RobotKinematics::chess_game_status_callback, this, std::placeholders::_1));

            chess_game_status_pub_ = this->create_publisher<std_msgs::msg::String>("/chess_game_status", 10);
 
            takeImage = this->create_publisher<std_msgs::msg::Bool>("/take_image", 10);

            chess_move_pub_ = this->create_publisher<std_msgs::msg::String>("/chess_moves", 10);
            status_pub = this->create_publisher<std_msgs::msg::String>("/status", 10);

            robot_stationary_ = false;
            pickup_dropoff_wait_ = 1.5; //seconds
            H1_transform_ = Eigen::Matrix4d::Identity();
 
            //intialise to default values for operation without arucos
            H1_X = 0.11436;
            H1_Y = -0.469443135;

            //promotion variable initialise
            request_peice_attach_ = false;

            //bin height

            //intialise cam position to default value
            camPosition.orientation.x = 1.0;
            camPosition.orientation.y = 0.0;
            camPosition.orientation.w = 0.0;
            camPosition.orientation.z = 0.0;
            camPosition.position.x = -(H1_X + ((-3.5 * (SQUARE_SIZE / 1000.0)) * cos(board_yaw) - (3.5 * (SQUARE_SIZE / 1000.0)) * sin(board_yaw))) + 0.037143913668;//matCamPos(0, 3);
            camPosition.position.y = -(H1_Y + ((-3.5 * (SQUARE_SIZE / 1000.0)) * sin(board_yaw) + (3.5 * (SQUARE_SIZE / 1000.0)) * cos(board_yaw))) - 0.068065853878;//matCamPos(1, 3);
            camPosition.position.z = 0.40406;
 
            moveToJointAngles(camera_view_jangle.at(0), camera_view_jangle.at(1),camera_view_jangle.at(2),camera_view_jangle.at(3),camera_view_jangle.at(4),camera_view_jangle.at(5));

            active_threads_ = 0;
            thread_mutex_ = std::make_unique<std::mutex>();
            thread_cv_ = std::make_unique<std::condition_variable>();
        }

        void publish_status(){
            std::string status_str;
            status_str += posError ? '1' : '0';
            status_str += yawError ? '1' : '0';
            status_str += eStop ? '1' : '0';
            status_str += playerTurn ? '1' : '0';

            auto msg = std_msgs::msg::String();
            msg.data = status_str;
            status_pub->publish(msg);
            // RCLCPP_WARN(this->get_logger(), "publishing error string: %s", status_str.c_str());
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
            move_group_ptr->allowReplanning(true);
        }
 
    private:
        void moveToCameraViewJ(){
            RCLCPP_INFO(this->get_logger(), "Attempting to move to camera view position...");
            moveToJointAngles(camera_view_jangle.at(0), camera_view_jangle.at(1), camera_view_jangle.at(2), camera_view_jangle.at(3), camera_view_jangle.at(4), camera_view_jangle.at(5));
        }
 
        void chess_topic_callback(const std_msgs::msg::String::SharedPtr msg) {
            std::string msgData = msg->data;

            RCLCPP_INFO(this->get_logger(), "[chess_topic_callback] received message: %s", msgData.c_str());

            //check for promotion move first
            if(msgData.find('=') != std::string::npos){
                RCLCPP_INFO(this->get_logger(), "[chess_topic_callback] promotion move");
                //promotion move
                maneuver(msgData[0], msgData[1], msgData[2], msgData[3], 'p', pieceHeight[msgData[5]], pieceHeight['p'], msgData);

            }else{
                RCLCPP_INFO(this->get_logger(), "[chess_topic_callback] normal move");
                if (msgData.length() < 12) { // if greater than 9 means it's the fen string for the camera node
                    RCLCPP_INFO(this->get_logger(), "Stockfish move: '%s'", msgData.c_str());
                    if(msgData.length() == 6){ // normal move
                        maneuver(msgData[0], msgData[1], msgData[2], msgData[3], msgData[4], pieceHeight[msgData[5]], 0, msgData);
                    }
                    if(msgData.length() == 7 || msgData.length() == 8){ // capture move
                        RCLCPP_INFO(this->get_logger(), "[chess_topic_callback] capture move");
                        maneuver(msgData[0], msgData[1], msgData[2], msgData[3], msgData[4], pieceHeight[msgData[5]], pieceHeight[msgData[6]], msgData);
                    }
                    if(msgData.length() == 11){ //castling
                        RCLCPP_INFO(this->get_logger(), "[chess_topic_callback] castling move");
                        //example castling string e8g8h8f8ckr. first move 'e8g8' second move 'h8f8' move type 'c' first move peice 'k' second move peice 'r'
                        maneuver(msgData[0], msgData[1], msgData[2], msgData[3], msgData[8], pieceHeight[msgData[9]], 0, msgData);
                        maneuver(msgData[4], msgData[5], msgData[6], msgData[7], msgData[8], pieceHeight[msgData[10]], 0, msgData);
                    }
                }
            }
        }
 
        void maneuver(char curFile, char curRank, char goalFile, char goalRank, char moveType, double pickupHeight, double pickupHeightCap, const std::string& original_move) {
            // Wait for any existing threads to complete
            {
                std::unique_lock<std::mutex> lock(*thread_mutex_);
                thread_cv_->wait(lock, [this]() { return active_threads_ == 0; });
            }

            // Create a shared pointer to ensure the thread has access to the data
            auto thread_data = std::make_shared<ManeuverData>(curFile, curRank, goalFile, goalRank, moveType, pickupHeight, pickupHeightCap, original_move);
            RCLCPP_INFO(this->get_logger(), "[maneuver] creating maneuver thread");
            
            // Increment active threads counter
            {
                std::lock_guard<std::mutex> lock(*thread_mutex_);
                active_threads_++;
            }

            // Create the thread with shared ownership
            std::thread([this, thread_data]() {
                try {
                    this->maneuverInThread(thread_data->curFile, thread_data->curRank, 
                                        thread_data->goalFile, thread_data->goalRank,
                                        thread_data->moveType, 
                                        thread_data->pickupHeight, thread_data->pickupHeightCap,
                                        thread_data->original_move);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception in maneuver thread: %s", e.what());
                }
                
                // Decrement active threads counter and notify waiting threads
                {
                    std::lock_guard<std::mutex> lock(*thread_mutex_);
                    active_threads_--;
                    thread_cv_->notify_all();
                }
            }).detach();
        }
 
        void maneuverInThread(char curFile, char curRank, char goalFile, char goalRank, char moveType, double pickupHeight, double pickupHeightCap, const std::string& original_move) {
            moveToJointAngles(1.544, -2.060, 0.372, -0.108, -1.651, -6.233);
            while(!H1_up_to_date_ && rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "blocking till H1 correct");
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            std::pair<double, double> cur = chessToGridCenter(curFile, curRank);
            std::pair<double, double> goal = chessToGridCenter(goalFile, goalRank);
            
            //loop so that if a move fails, it will try again
            bool maneuver_complete = false;
            bool first_try = true;
            while (rclcpp::ok()){
                if(maneuver_complete){
                    return;
                }
                
                if(!first_try){
                    // Publish the original move to retry
                    auto msg = std_msgs::msg::String();
                    msg.data = original_move;
                    chess_move_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Retrying move: %s", original_move.c_str());
                    return;
                }
                first_try = false;

                std::vector<geometry_msgs::msg::Pose> points = {};
                geometry_msgs::msg::Pose tempPosition;
                tempPosition.position.z = operation_height;
                tempPosition.orientation.x = 1.0;
                tempPosition.orientation.y = 0.0;
                tempPosition.orientation.w = 0.0;

                // ─── Move to viewing position ─────────────────────────────────────────────
                isTakeImage = false;
                RCLCPP_INFO(this->get_logger(), "[maneuver] angle move to viewing position");

                if(!moveToJointAngles(1.544, -2.060, 0.372, -0.108, -1.651, -6.233)) continue;

                // Wait until H1 position is up to date
                while(!H1_up_to_date_ && rclcpp::ok()) {
                    RCLCPP_INFO(this->get_logger(), "blocking till H1 correct");
                    std::this_thread::sleep_for(std::chrono::milliseconds(25));
                }

                // Log the positions for debugging
                RCLCPP_INFO(this->get_logger(), "H1 position: (%.3f, %.3f)", H1_X, H1_Y);
                RCLCPP_INFO(this->get_logger(), "Current position: (%.3f, %.3f)", cur.first, cur.second);
                RCLCPP_INFO(this->get_logger(), "Goal position: (%.3f, %.3f)", goal.first, goal.second);

                if(!moveToJointAngles(M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0)) continue;
    
                // remove the captured piece from the board
                if (moveType == 'x' || moveType == 'p') {
                    RCLCPP_INFO(this->get_logger(), "[maneuver] maneuver for capture move");
                    if(moveType == 'x'){
                        tempPosition.position.x = goal.first;
                        tempPosition.position.y = goal.second;
                    }else if(moveType == 'p'){ //if we are promoting we are essentially "capturing" our own peice
                        tempPosition.position.x = cur.first;
                        tempPosition.position.y = cur.second;
                    }
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] Remove captured: xStart: %.3f%% and yStart: %.3f%% and zPickUp: %.3f%%", cur.first, cur.second, pickupHeightCap);
                    tempPosition.position.z = pickupHeightCap;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight down to pickup peice to be captured");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    publishServoState(true);
                    std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                    // Raise the shit up
                    tempPosition.position.z = operation_height;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight up to pickup peice being captured");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    //bin stuff boi
                    if(!moveToJointAngles(1.544, -2.060, 0.372, -0.108, -1.651, -6.233)) continue;
                    if(!moveToJointAngles(1.544-M_PI/2, -2.060, 0.372, -0.108, -1.651, -6.233)) continue;
                    while(!robot_stationary_){
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    if(!moveToJointAngles(0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0)) continue;

                    tempPosition.position.x = x_Bin_Aruco;
                    tempPosition.position.y = y_Bin_Aruco;
                    tempPosition.position.z = aboveBinHeight;
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    tempPosition.position.z = binDropHeight;
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    publishServoState(false);
                    std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                    tempPosition.position.z = aboveBinHeight;
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    if(!moveToJointAngles(M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0)) continue;

                    
                    // tempPosition.position.x = -0.292;
                    // tempPosition.position.y = 0.290;
                    // RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to drop off position");
                    // moveStraightToPoint({tempPosition}, 0.05, 0.05);
                    // publishServoState(false);
                    // std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                }   

                if(moveType == 'p'){ // promotion logic
                    if(!moveToJointAngles(M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0)) continue;
                    request_peice_attach_ = true;
                    publish_chess_status("back_promote"); //send to pi
                    //wait for response
                    while(rclcpp::ok()){
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        if(!request_peice_attach_){
                            RCLCPP_INFO(this->get_logger(), "received message from pi that chess peice is attached");
                            break;
                        }
                    }
                }

                // Here, we play the damn piece
                if (moveType == 'n' || moveType == 'x' || moveType == 'c'){
                    tempPosition.position.x = cur.first;
                    tempPosition.position.y = cur.second;
                    RCLCPP_INFO(this->get_logger(), "xStart: %.3f%% and yStart: %.3f%% and zPickUp: %.3f%%", cur.first, cur.second, pickupHeight);
                    publish_point(cur.first, cur.second, pickupHeight, 1.0, 0.0, 0.0);
                    publish_point(goal.first, goal.second, pickupHeight, 0.0, 1.0, 0.0);
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice position");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    tempPosition.position.z = pickupHeight;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to target peice height");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    publishServoState(true);
                    std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                    tempPosition.position.z = operation_height;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice position");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                }
                if(moveType == 'n' || moveType == 'x' || moveType == 'c' || moveType == 'p'){
                    tempPosition.position.z = operation_height;
                    tempPosition.position.x = goal.first;
                    tempPosition.position.y = goal.second;
                    RCLCPP_INFO(this->get_logger(), "xEnd: %.3f%% and yEnd: %.3f%%", goal.first, goal.second);
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to above target peice destination");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    tempPosition.position.z = pickupHeight;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight to target peice destination height");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                    publishServoState(false);
                    std::this_thread::sleep_for(std::chrono::seconds(pickup_dropoff_wait_));
                    tempPosition.position.z = operation_height;
                    RCLCPP_INFO(this->get_logger(), "[maneuver] move straight back to operation height peice move done");
                    if(!moveStraightToPoint({tempPosition}, 0.05, 0.05)) continue;
                }
                
    
                RCLCPP_INFO(this->get_logger(), "[maneuver] angle move to viewing position");
                if(!moveToJointAngles(1.544, -2.060, 0.372, -0.108, -1.651, -6.233)) continue;
                isTakeImage = true;
                playerTurn = true;
                publish_status();

                maneuver_complete = true; //next loop will return
            }
        }

        bool moveToJointAngles(double j1, double j2, double j3, double j4, double j5, double j6) {
            if (move_group_ptr == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "[moveToJointAnglesInThread] MoveGroup pointer is not initialized");
                return false;
            }
            
            std::lock_guard<std::mutex> lock(moveit_mutex);
            // std::unique_lock<std::mutex> lock(moveit_mutex);
            RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Mutex acquired, planning...");
 
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
                    move_group_ptr->setStartStateToCurrentState();
                    move_group_ptr->setMaxVelocityScalingFactor(MAX_VEL_JOINT_TARGET);
                    move_group_ptr->setMaxAccelerationScalingFactor(MAX_ACCEL_JOINT_TARGET);
 
                    move_group_ptr->setJointValueTarget(joint_positions);
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
 
                    RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Planning motion to joint angles [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                                j1, j2, j3, j4, j5, j6);
 
                    auto planning_result = move_group_ptr->plan(plan);
                    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_WARN(this->get_logger(), "[moveToJointAnglesInThread] Planning failed. Attempting to replan...");
                        
                        // Try replanning a few times with different parameters
                        for(int attempt = 0; attempt < 3; attempt++) {
                            // Increase planning time for subsequent attempts
                            move_group_ptr->setPlanningTime(5.0 + attempt * 2.0);
                            
                            planning_result = move_group_ptr->plan(plan);
                            if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
                                RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Replanning succeeded on attempt %d", attempt + 1);
                                break;
                            }
                            
                            RCLCPP_WARN(this->get_logger(), "[moveToJointAnglesInThread] Replanning attempt %d failed", attempt + 1);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        
                        if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
                            RCLCPP_ERROR(this->get_logger(), "[moveToJointAnglesInThread] All planning attempts failed");
                            for(int i = 0; i < 5; i++) {
                                RCLCPP_ERROR(this->get_logger(), "[moveToJointAnglesInThread] All planning attempts failed");
                                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                            }
                            return false;
                        }
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
                    if(robot_stationary_ && std::chrono::steady_clock::now() - start > std::chrono::milliseconds(500)){
                        //condition met for move complete
                        RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] SUCCESS move complete");
                        //delay for safety to ensure that the robot has actually stopped and planning for the next move won't fail

                        bool stationary_constant = true;
                        for(int i = 0; i < 10; i++){
                            if(!robot_stationary_){
                                stationary_constant = false;
                                break;
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        if(stationary_constant){
                            return true;
                        }
                    }
 
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Retrying motion after interruption...");
            }
            return false;  // Should not be reached unless ROS is shutting down
        }
 
        bool moveStraightToPoint(std::vector<geometry_msgs::msg::Pose> tempPosition, double vel, double acc) {
            
            if (move_group_ptr == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "[moveStraightToPoint] MoveGroup pointer is not initialized");
                return false;
            }
            std::lock_guard<std::mutex> lock(moveit_mutex);

            // std::unique_lock<std::mutex> lock(moveit_mutex);
            RCLCPP_INFO(this->get_logger(), "[moveStraightToPoint] Mutex acquired, planning...");

            int count = 0;
            int print_freq = 50;
            while (rclcpp::ok()) {
                // ─── Wait for Deadman ─────────────────────────────────────────────
                while (!button_state && rclcpp::ok()) {
                    count++;
                    if (count % print_freq == 0) {
                        RCLCPP_INFO(this->get_logger(), "[moveStraightToPoint] Waiting for deadman switch...");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    eStop = true;
                    publish_status();
                }
                eStop = false;
                publish_status();
 
                // ─── Plan ─────────────────────────────────────────────────────────
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction;
 
                {
                    move_group_ptr->setStartStateToCurrentState();
                    move_group_ptr->setMaxVelocityScalingFactor(MAX_VEL_CARTESIAN);
                    move_group_ptr->setMaxAccelerationScalingFactor(MAX_ACCEL_CARTESIAN);
 
                    fraction = move_group_ptr->computeCartesianPath(tempPosition, 0.01, 0.0, trajectory, /*avoid_collisions =*/ true);
                    if (fraction < 0.95) {
                        RCLCPP_WARN(this->get_logger(), "[moveStraightToPoint] Path only %.2f%% complete", fraction * 100.0);

                        RCLCPP_WARN(this->get_logger(), "[moveStraightToPoint] Planning failed. Attempting to replan...");
                        
                        // Try replanning a few times with different parameters
                        for(int attempt = 0; attempt < 3; attempt++) {
                            // Increase planning time for subsequent attempts
                            move_group_ptr->setPlanningTime(5.0 + attempt * 2.0);
                            
                            fraction = move_group_ptr->computeCartesianPath(tempPosition, 0.01, 0.0, trajectory);
                            if (fraction > 0.95) {
                                RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] Replanning succeeded on attempt %d", attempt + 1);
                                break;
                            }
                            
                            RCLCPP_WARN(this->get_logger(), "[moveToJointAnglesInThread] Replanning attempt %d failed", attempt + 1);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        for(int i = 0; i<25; i++){
                            RCLCPP_WARN(this->get_logger(), "[moveStraightToPoint] All planning attempts failed!!!.");
                            RCLCPP_INFO(this->get_logger(), "X: %.3f, Y: %.3f, Z: %.3f", tempPosition[0].position.x, tempPosition[0].position.y, tempPosition[0].position.z);
                            std::this_thread::sleep_for(std::chrono::milliseconds(250));
                        }
                        return false;
                    }

                    // Check for collisions in the planned trajectory
                    try{  
                        moveit::core::RobotState current_state = *move_group_ptr->getCurrentState();
                        collision_detection::CollisionRequest collision_request;
                        collision_detection::CollisionResult collision_result;
                        collision_request.group_name = move_group_ptr->getName();
                        collision_request.contacts = true;
                        collision_request.max_contacts = 100;
    
                        // Get the planning scene
                        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
                        auto planning_scene = std::make_shared<planning_scene::PlanningScene>(move_group_ptr->getRobotModel());
                        if (!planning_scene) {
                            RCLCPP_ERROR(this->get_logger(), "[moveStraightToPoint] Failed to create planning scene");
                            return false;
                        }
    
                        // Update planning scene with current state
                        planning_scene->setCurrentState(current_state);
    
                        // Check each waypoint in the trajectory for collisions
                        for (const auto& waypoint : trajectory.joint_trajectory.points) {
                            // Update robot state with waypoint joint positions
                            for (size_t i = 0; i < waypoint.positions.size(); ++i) {
                                current_state.setJointPositions(trajectory.joint_trajectory.joint_names[i], &waypoint.positions[i]);
                            }
                            
                            // Check for collisions at this waypoint
                            planning_scene->checkCollision(collision_request, collision_result, current_state);
                            if (collision_result.collision) {
                                RCLCPP_WARN(this->get_logger(), "[moveStraightToPoint] Collision detected in planned trajectory");
                                return false;
                            }
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "[moveStraightToPoint] Exception during collision checking: %s", e.what());
                        return false;
                    } catch (...) {
                        RCLCPP_ERROR(this->get_logger(), "[moveStraightToPoint] Unknown exception during collision checking");
                        return false;
                    }

                    plan.trajectory_ = trajectory;
                }

                // ─── Execute ──────────────────────────────────────────────────────
                move_group_ptr->asyncExecute(plan);
                auto start = std::chrono::steady_clock::now();
                bool witness_robot_moving = false;
                while (rclcpp::ok()) {
                    if (!button_state) {
                        RCLCPP_WARN(this->get_logger(), "[moveStraightToPoint] Deadman switch released, stopping...");
                        eStop = true;
                        publish_status();
                        move_group_ptr->stop();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        break;  // Retry from outer loop
                    }

                    if(!robot_stationary_){
                        witness_robot_moving = true;
                    }
 
                    //check if the robot has finished it's move
                    if(robot_stationary_ && std::chrono::steady_clock::now() - start > std::chrono::milliseconds(500) && witness_robot_moving){
                        //condition met for move complete
                        RCLCPP_INFO(this->get_logger(), "[moveToJointAnglesInThread] SUCCESS move complete");
                        //delay for safety to ensure that the robot has actually stopped and planning for the next move won't fail
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        return true;
                    }
 
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
 
                RCLCPP_INFO(this->get_logger(), "[moveStraightToPoint] Retrying motion after interruption...");
            }
 
            return false;  // Shouldn't hit this unless ROS is shutting down
        }
 
 
        std::pair<double, double> chessToGridCenter(char file, char rank) {
            std::lock_guard<std::mutex> lock(h1_mutex);
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

            RCLCPP_INFO(this->get_logger(), "chessToGridCenter - Calculated positions: dx=%.6f, dy=%.6f, x_rotated=%.6f, y_rotated=%.6f, x_final=%.6f, y_final=%.6f",
                dx, dy, x_rotated, y_rotated, x_final, y_final);


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

        // status variables
        bool posError = false;
        bool yawError = false;
        bool eStop = false;
        bool playerTurn = true;
 
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
        double x_Bin_Aruco;
        double y_Bin_Aruco;
        geometry_msgs::msg::Pose camPosition;
        Eigen::Matrix4d H1_transform_;

        double aboveBinHeight = 0.1848+100/1000+0.125;
        double binDropHeight = aboveBinHeight - 50/1000;
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

                if(marker_id == 693 && robot_stationary_){
                    std::lock_guard<std::mutex> lock(bin_mutex);
                    x_Bin_Aruco = marker.pose.position.x;
                    y_Bin_Aruco = marker.pose.position.y;
                    RCLCPP_INFO(this->get_logger(), "x_Bin_Aruco: %.3f, y_Bin_Aruco: %.3f", x_Bin_Aruco, y_Bin_Aruco);
                }
 
                if (marker_id == 53 && robot_stationary_) { //only read the markers if the robot is currently stationary
                    // Check if enough time has passed since robot became stationary
                    auto current_time = std::chrono::steady_clock::now();
                    auto time_since_stationary = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - robot_stationary_time_).count();
                    
                    if (time_since_stationary >= 5) {
                        H1_up_to_date_ = true;
                    }

                    // RCLCPP_INFO(this->get_logger(), "UPDATING board position");
                    std::lock_guard<std::mutex> lock(h1_mutex);
                    std::string frame_id = marker.header.frame_id;             
 
                    auto q_msg = marker.pose.orientation;
                    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
                    Eigen::Vector3d t(marker.pose.position.x,
                        marker.pose.position.y,
                        marker.pose.position.z);
 
                    H1_X = -marker.pose.position.x;
                    H1_Y = -marker.pose.position.y;
                    // RCLCPP_INFO(this->get_logger(), "H1_X: %.3f, H1_Y: %.3f", H1_X, H1_Y);
                    // RCLCPP_INFO(this->get_logger(), "H1_up_to_date_: %s", H1_up_to_date_ ? "TRUE" : "FALSE");
                    
                    if(std::abs(H1_X) < 0.15 || H1_Y > 0.45 || H1_Y < 0.35){
                        posError = true;
                        // RCLCPP_WARN(this->get_logger(), "pos out of tolerance");
                    }
                    else if(board_yaw < 0.13 || board_yaw > (M_PI-0.13)){
                        // RCLCPP_INFO(this->get_logger(), "publishing board yaw");
                        yawError = true;
                        RCLCPP_WARN(this->get_logger(), "yaw out of tolerance");
                    }
                    else{
                        yawError = false;
                        posError = false;
                    }
                    publish_status();
                    // RCLCPP_INFO(this->get_logger(), "board yaw: %.4f", board_yaw);

                    double dx = 3.5 * (SQUARE_SIZE / 1000.0);
                    double dy = -3.5 * (SQUARE_SIZE / 1000.0);
 
                    if(board_yaw < 1.57){
                        dx = -3.5 * (SQUARE_SIZE / 1000.0);
                        dy =  3.5 * (SQUARE_SIZE / 1000.0);
                    }
 
                    // Apply 2D rotation to account for yaw
                    double x_final = -(H1_X + (dx * cos(board_yaw) - dy * sin(board_yaw)));
                    double y_final = -(H1_Y + (dx * sin(board_yaw) + dy * cos(board_yaw)));

                    camPosition.orientation.x = 1.0;
                    camPosition.orientation.y = 0.0;
                    camPosition.orientation.w = 0.0;
                    camPosition.orientation.z = 0.0;
                    // camPosition.orientation.x = marker.pose.orientation.x;
                    // camPosition.orientation.y = marker.pose.orientation.y;
                    // camPosition.orientation.z = marker.pose.orientation.z;
                    // camPosition.orientation.w = marker.pose.orientation.w;
                    
                    camPosition.position.x = x_final + 0.037143913668;//matCamPos(0, 3);
                    camPosition.position.y = y_final - 0.068065853878;//matCamPos(1, 3);
                    camPosition.position.z = 0.40406;
                }
            }
        }        
 
        //to detect the robot moving
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            static std::vector<double> last_positions;
            static int callbackCount;
            int callback_stationary_review = 5;
 
            const double POSITION_EPSILON = 0.001;  // 0.001 rad = ~0.057 degrees for gazebo
 
            bool moving = false;
 
            // First time: initialize and return
            if (last_positions.empty()) {
                last_positions = msg->position;
                robot_stationary_ = true;
                robot_stationary_time_ = std::chrono::steady_clock::now();
                H1_up_to_date_ = false;
                callbackCount = 0;
                RCLCPP_INFO(this->get_logger(), "Initialized joint position tracking");
                return;
            }
 
            // Compare current with previous positions
            if(callbackCount % callback_stationary_review == 0){
                
                size_t n = std::min(last_positions.size(), msg->position.size());
                for (size_t i = 0; i < n; ++i) {
                    if (std::abs(msg->position[i] - last_positions[i]) > POSITION_EPSILON) {
                        moving = true;
                        break;
                    }
                }

                // If robot just became stationary, update the timestamp
                if (!robot_stationary_ && !moving) {
                    robot_stationary_time_ = std::chrono::steady_clock::now();
                    H1_up_to_date_ = false;
                }
                
                robot_stationary_ = !moving;  
                if (robot_stationary_) {
                    // RCLCPP_INFO(this->get_logger(), "Robot is stationary.");
                } else {
                    // RCLCPP_INFO(this->get_logger(), "Robot is moving.");
                }
                last_positions = msg->position;
            }
 
            callbackCount +=1;
        }
 
        //joint state sub obj
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        int pickup_dropoff_wait_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chess_move_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;

        //button state variables
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
        // bool button_state;
        std::atomic<bool> button_state;
        std::atomic<bool> robot_stationary_;
        std::mutex moveit_mutex;
        std::mutex h1_mutex;
        std::mutex bin_mutex;

        // Add H1 position tracking variables
        std::atomic<bool> H1_up_to_date_;
        std::chrono::steady_clock::time_point robot_stationary_time_;

        //promotion global variable
        std::atomic<bool> request_peice_attach_;

        void button_callback(const std_msgs::msg::Bool::SharedPtr msg){
            button_state = msg->data;
            if(msg->data) {
                RCLCPP_WARN(this->get_logger(), "E-stop engaged (button_state == true)!");
                button_state = true;
                if(isTakeImage && !simulation_mode_){
                    RCLCPP_WARN(this->get_logger(), "[button_callback] starting takePictureInThread");
                    takePictureInThread();
                    isTakeImage = false;
                }
                if(isTakeImage && simulation_mode_){
                    playerTurn = false;
                    publish_status();
                }
            }
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chess_game_status_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chess_game_status_pub_;

        void chess_game_status_callback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received game status: '%s'", msg->data.c_str());
            std::string msgData = msg->data;
            if(msgData == "piece_attached"){
                request_peice_attach_ = false;
            }
        }

        void publish_chess_status(std::string data){
            auto message = std_msgs::msg::String();
            message.data = data; 
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            chess_game_status_pub_->publish(message);
        }

        void takePictureInThread() {
            // Wait for any existing threads to complete
            {
                std::unique_lock<std::mutex> lock(*thread_mutex_);
                thread_cv_->wait(lock, [this]() { return active_threads_ == 0; });
            }

            // Increment active threads counter
            {
                std::lock_guard<std::mutex> lock(*thread_mutex_);
                active_threads_++;
            }

            std::thread([this]() {
                try {
                    this->camera_maneuver();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception in camera maneuver thread: %s", e.what());
                }
                
                // Decrement active threads counter and notify waiting threads
                {
                    std::lock_guard<std::mutex> lock(*thread_mutex_);
                    active_threads_--;
                    thread_cv_->notify_all();
                }
            }).detach();
        }
 
        void camera_maneuver(){
            while(!robot_stationary_){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            //might be something to try soon
            // while(!H1_up_to_date_ && rclcpp::ok()) {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(25));
            // }

            //move goes to robot here
            playerTurn = false;
            publish_status();
 
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
 
            while(!robot_stationary_){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
 
            moveStraightToPoint({camPositionReal}, 0.05, 0.05);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            RCLCPP_INFO(this->get_logger(), "[camera_maneuver] taking image");
            takeImage->publish(std_msgs::msg::Bool().set__data(true));
            RCLCPP_INFO(this->get_logger(), "[camera_maneuver] published take image");
        }

    struct ManeuverData {
        char curFile;
        char curRank;
        char goalFile;
        char goalRank;
        char moveType;
        double pickupHeight;
        double pickupHeightCap;
        std::string original_move;

        ManeuverData(char cf, char cr, char gf, char gr, 
                    char mt, double ph, double phc, const std::string& om)
            : curFile(cf), curRank(cr), goalFile(gf), goalRank(gr), 
              moveType(mt), pickupHeight(ph), pickupHeightCap(phc),
              original_move(om) {}
    };

    // Add simulation mode member variable
    bool simulation_mode_;

    // Add thread tracking variables
    std::atomic<int> active_threads_;
    std::unique_ptr<std::mutex> thread_mutex_;
    std::unique_ptr<std::condition_variable> thread_cv_;
};
 
 
int main(int argc, char * argv[])
{
    // Set up logging environment variables
    setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{time}] [{name}]: {message}", 1);
    setenv("RCUTILS_LOGGING_USE_STDOUT", "0", 1);
    setenv("RCUTILS_LOGGING_BUFFERED_STREAM", "1", 1);
    setenv("RCUTILS_LOGGING_USE_STDOUT", "0", 1);
    setenv("RCUTILS_LOGGING_USE_STDERR", "0", 1);
    setenv("RCUTILS_LOGGING_USE_FILE", "1", 1);
    setenv("RCUTILS_LOGGING_FILE_PATH", "robot_kinematics.log", 1);
    
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotKinematics>();
 
    // Log startup information
    RCLCPP_INFO(node->get_logger(), "Robot Kinematics node starting up");
    RCLCPP_INFO(node->get_logger(), "Log file: robot_kinematics.log");
 
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

    // Add a second collision object for the specified volume
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group.getPlanningFrame();
    collision_object2.id = "volume_cube";

    // Define the dimensions for the second cube
    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive2.BOX_X] = 0.4;  // X: -0.4 to 0.4 = 0.8m
    primitive2.dimensions[primitive2.BOX_Y] = 0.5;  // Y: 0.1 to 0.6 = 0.5m
    primitive2.dimensions[primitive2.BOX_Z] = 0.05; // Z: 0 to 0.05 = 0.05m
    // primitive2.dimensions[primitive2.BOX_Z] = 0.5; // Z: 0 to 0.05 = 0.05m

    // Define the pose of the second cube
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0;     // Centered between -0.4 and 0.4
    box_pose2.position.y = 0.35;    // Centered between 0.1 and 0.6
    box_pose2.position.z = 0.025;   // Centered between 0 and 0.05
    // box_pose2.position.z = 0.5/2;   // Centered between 0 and 0.05

    // Add the primitive and pose to the second collision object
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object2.ADD;

    // Add both collision objects to the planning scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_object2);
    RCLCPP_INFO(node->get_logger(), "Adding collision objects to planning scene");
    planning_scene_interface.addCollisionObjects(collision_objects);
 
    // Give the planning scene some time to update
    rclcpp::sleep_for(std::chrono::seconds(2));
    
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
 
 