#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <algorithm>  // For std::clamp

class JointStateAnimator : public rclcpp::Node {
public:
    JointStateAnimator() : Node("joint_state_animator"), step_index_(0) {
        // Set up publisher for joint states
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // Define joint names (adjust as per your robot's URDF)
        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        // Define start and end joint positions (modify these values as needed)
        start_positions_ = {0.0, -M_PI/2, 0.0, 0.0, 0.0, 0.0};  // Example start state
        end_positions_   = {1.0, M_PI, 0.0, 0.0, 0.0, 0.0};    // Example end state

        // Define joint limits for each joint (update these with your actual robot's limits)
        joint_min_limits_ = {2*M_PI, -M_PI, -2.9671, -2.0944, -2.9671, -2.0944};
        joint_max_limits_ = {2*M_PI, 0,  2.9671,  2.0944,  2.9671,  2.0944};

        // Animation parameters
        steps_ = 50;       // Total interpolation steps
        duration_ = 5.0;   // Total animation time in seconds

        // Calculate publish period based on steps and duration
        double publish_rate = duration_ / steps_;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_rate),
            std::bind(&JointStateAnimator::publish_joint_state, this)
        );

        RCLCPP_INFO(this->get_logger(), "Joint State Animator Node Started.");
    }

private:
    void publish_joint_state() {
        if (step_index_ > steps_) {
            RCLCPP_INFO(this->get_logger(), "Animation complete.");
            timer_->cancel();
            return;
        }

        

        // Compute the interpolated joint positions, while respecting joint limits
        std::vector<double> interpolated_positions(joint_names_.size());
        double alpha = static_cast<double>(step_index_) / steps_;

        for (size_t i = 0; i < joint_names_.size(); i++) {
            // Linear interpolation between start and end
            double interpolated = (1 - alpha) * start_positions_[i] + alpha * end_positions_[i];
            // Clamp the result within the joint limits
            interpolated_positions[i] = std::clamp(interpolated, joint_min_limits_[i], joint_max_limits_[i]);
        }

        // Create and populate the JointState message
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = joint_names_;
        joint_msg.position = interpolated_positions;

        // Publish the joint state message
        joint_pub_->publish(joint_msg);

        RCLCPP_INFO(this->get_logger(), "Published joint state step %d/%d", step_index_, steps_);
        step_index_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> start_positions_;
    std::vector<double> end_positions_;
    std::vector<double> joint_min_limits_;
    std::vector<double> joint_max_limits_;

    int step_index_;
    int steps_;
    double duration_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateAnimator>());
    rclcpp::shutdown();
    return 0;
}
