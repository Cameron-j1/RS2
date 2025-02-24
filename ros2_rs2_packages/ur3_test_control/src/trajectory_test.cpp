#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// KDL includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <memory>
#include <cmath>

class UR3eController : public rclcpp::Node
{
public:
  UR3eController() : Node("ur3e_controller")
  {
    // Build the kinematic chain using DH parameters.
    buildKDLChain();

    // Create forward kinematics and IK solvers.
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);

    // Set joint limits (radians)
    unsigned int nj = chain_.getNrOfJoints();
    lower_limits_ = KDL::JntArray(nj);
    upper_limits_ = KDL::JntArray(nj);
    lower_limits_(0) = 10* -2 * M_PI;
    upper_limits_(0) = 10*  2 * M_PI;
    lower_limits_(1) = 10* -210.0 * M_PI / 180.0;
    upper_limits_(1) = 10*  35.0 * M_PI / 180.0;
    lower_limits_(2) = 10* -150.0 * M_PI / 180.0;
    upper_limits_(2) = 10*  150.0 * M_PI / 180.0;
    lower_limits_(3) = 10* -2 * M_PI;
    upper_limits_(3) = 10*  2 * M_PI;
    lower_limits_(4) = 10* -2 * M_PI;
    upper_limits_(4) = 10*  2 * M_PI;
    lower_limits_(5) = 10* -2 * M_PI;
    upper_limits_(5) = 10*  2 * M_PI;

    // Create the IK solver with joint limits.
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      chain_, lower_limits_, upper_limits_, *fk_solver_, *ik_vel_solver_, 10000, 1e-1);

    // Create publisher for joint trajectory commands.
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/scaled_joint_trajectory_controller/joint_trajectory", 10);
  }

  // Converts a ROS Pose to a KDL::Frame.
  KDL::Frame poseToKDLFrame(const geometry_msgs::msg::Pose &pose)
  {
    return KDL::Frame(
      KDL::Rotation::Quaternion(pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w),
      KDL::Vector(pose.position.x,
                  pose.position.y,
                  pose.position.z)
    );
  }

  // Given a target pose, computes IK and publishes a trajectory command.
  bool moveToPose(const geometry_msgs::msg::PoseStamped &target_pose)
  {
    KDL::Frame target_frame = poseToKDLFrame(target_pose.pose);

    // Use an initial guess of zero for all joints.
    KDL::JntArray q_init(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < q_init.rows(); ++i) {
      q_init(i) = 0.0;
    }

    KDL::JntArray q_solution(chain_.getNrOfJoints());
    int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_solution);
    if (ik_result < 0) {
      RCLCPP_ERROR(this->get_logger(), "IK solver failed with code: %d", ik_result);
      return false;
    }

    // Build and publish a JointTrajectory message.
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (unsigned int i = 0; i < q_solution.rows(); ++i) {
      point.positions.push_back(q_solution(i));
    }
    point.time_from_start.sec = 3; // Execute over 3 seconds.
    traj_msg.points.push_back(point);

    traj_pub_->publish(traj_msg);
    RCLCPP_INFO(this->get_logger(), "Published trajectory to move to target pose.");
    return true;
  }

private:
  // Build the KDL chain using the provided DH parameters.
  // Transformation order used: RotZ(joint) -> TransZ(d) -> TransX(a) -> RotX(alpha)
  void buildKDLChain()
  {
    chain_ = KDL::Chain();

    // Link 1: d = 0.15185, a = 0, alpha = pi/2.
    chain_.addSegment(KDL::Segment("link1",
      KDL::Joint(KDL::Joint::RotZ),
      dhTransform(0.0, M_PI / 2, 0.15185)));

    // Link 2: d = 0, a = -0.24355, alpha = 0.
    chain_.addSegment(KDL::Segment("link2",
      KDL::Joint(KDL::Joint::RotZ),
      dhTransform(-0.24355, 0.0, 0.0)));

    // Link 3: d = 0, a = -0.2132, alpha = 0.
    chain_.addSegment(KDL::Segment("link3",
      KDL::Joint(KDL::Joint::RotZ),
      dhTransform(-0.2132, 0.0, 0.0)));

    // Link 4: d = 0.13105, a = 0, alpha = pi/2.
    chain_.addSegment(KDL::Segment("link4",
      KDL::Joint(KDL::Joint::RotZ),
      dhTransform(0.0, M_PI / 2, 0.13105)));

    // Link 5: d = 0.08535, a = 0, alpha = -pi/2.
    chain_.addSegment(KDL::Segment("link5",
      KDL::Joint(KDL::Joint::RotZ),
      dhTransform(0.0, -M_PI / 2, 0.08535)));

    // Link 6: d = 0.0921, a = 0, alpha = 0.
    chain_.addSegment(KDL::Segment("link6",
      KDL::Joint(KDL::Joint::RotZ),
      dhTransform(0.0, 0.0, 0.0921)));
  }

  // Helper to create a KDL::Frame from DH parameters (a, alpha, d).
  KDL::Frame dhTransform(double a, double alpha, double d)
  {
    return KDL::Frame(
      KDL::Rotation::RotX(alpha),
      KDL::Vector(a, 0.0, d));
  }

  // KDL objects and ROS publisher.
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
  KDL::JntArray lower_limits_;
  KDL::JntArray upper_limits_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto controller_node = std::make_shared<UR3eController>();

  // Define a target pose (example).
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = 0.3;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.0;
  // Identity quaternion (no rotation)
  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 1.0;

  if (!controller_node->moveToPose(target_pose)) {
    RCLCPP_ERROR(controller_node->get_logger(), "Failed to move to the desired pose");
  }

  rclcpp::spin(controller_node);
  rclcpp::shutdown();
  return 0;
}
