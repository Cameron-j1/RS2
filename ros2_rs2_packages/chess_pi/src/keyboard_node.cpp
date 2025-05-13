#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <mutex>  // For std::mutex

class KeyboardJointController : public rclcpp::Node {
public:
  KeyboardJointController()
  : Node("keyboard_joint_controller"), step_size_(0.01), running_(true)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

            // Define joint names (adjust as per your robot's URDF)
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    // current_positions_.resize(joint_names_.size(), 0.0);

    current_positions_ = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};

    joint_min_limits_ = {-6.283, -M_PI, -2.5, -10, -10, -10 };
    joint_max_limits_ = {  6.283,  0,  2.5,  10,  10,  10 };

    key_map_ = {
      {'q', {0,  1.0}},
      {'a', {0, -1.0}},
      {'w', {1,  1.0}},
      {'s', {1, -1.0}},
      {'e', {2,  1.0}},
      {'d', {2, -1.0}},
      {'r', {3,  1.0}},
      {'f', {3, -1.0}},
      {'t', {4,  1.0}},
      {'g', {4, -1.0}},
      {'y', {5,  1.0}},
      {'h', {5, -1.0}}
    };

    keyboard_thread_ = std::thread(&KeyboardJointController::keyboardLoop, this);

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { this->publishJointState(); }
    );

    RCLCPP_INFO(this->get_logger(), "Keyboard Joint Controller started. Use keys (q/a, w/s, e/d, r/f, t/g, y/h) to move joints. Press 'x' to exit.");
  }

  ~KeyboardJointController() override {
    running_ = false;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
  }

private:
  void publishJointState() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    
    // Lock access to current_positions_ before reading
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      msg.name = joint_names_;
      msg.position = current_positions_;
    }
    
    joint_pub_->publish(msg);
  }

  void keyboardLoop() {
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    while (running_ && rclcpp::ok()) {
      char c = getchar();
      if (c == 'x') {
        RCLCPP_INFO(this->get_logger(), "Exit key pressed. Shutting down keyboard input.");
        running_ = false;
        break;
      }
      if (key_map_.find(c) != key_map_.end()) {
        int joint_index = key_map_[c].first;
        double direction = key_map_[c].second;
        {
          // Lock access to current_positions_ while updating
          std::lock_guard<std::mutex> lock(joint_mutex_);
          double new_value = current_positions_[joint_index] + direction * step_size_;
          new_value = std::clamp(new_value, joint_min_limits_[joint_index], joint_max_limits_[joint_index]);
          current_positions_[joint_index] = new_value;
          RCLCPP_INFO(this->get_logger(), "Joint %d updated to: %.3f", joint_index, new_value);
        }
      }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;
  double step_size_;

  std::unordered_map<char, std::pair<int, double>> key_map_;

  std::thread keyboard_thread_;
  std::atomic<bool> running_;

  std::mutex joint_mutex_;  // Mutex to protect joint state vector
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardJointController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
