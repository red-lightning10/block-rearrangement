#ifndef GRASPING_MOVEIT_MOVEIT_UTILS_HPP_
#define GRASPING_MOVEIT_MOVEIT_UTILS_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/buffer.h>
#include <string>
#include <stdexcept>
#include <vector>
#include <memory>

namespace grasping_moveit {

class MoveGroupUtilities
{
public:

  explicit MoveGroupUtilities(
    moveit::planning_interface::MoveGroupInterface& move_group,
    rclcpp::Node::SharedPtr node = nullptr,
    rclcpp::Logger logger = rclcpp::get_logger("moveit_utils"))
    : move_group_(move_group), node_(node), logger_(std::move(logger))
  {
    // Create planning scene publisher if node is provided
    if (node_) {
      planning_scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", rclcpp::QoS(10));
    }
  }

  /**
   * Move robot in straight-line motion to target pose using MoveIt's Cartesian path planning
   * Do not use if we want to perform collision avoidance
   */
  bool moveL(const geometry_msgs::msg::Pose& target_pose,
    double velocity_scaling = 0.02,
    double acceleration_scaling = 0.02,
    double eef_step = 0.01);

  /**
   * Plan and execute motion to joint space goal using specified planner
   * Takes pre-computed IK solutions (joint values) that are verified collision-free
   * and plans a smooth path
   * #TODO: Not working as intended, returns wonky paths.
   */
  bool movePlan(const std::vector<double>& joint_values,
    const std::string& planner_id = "chomp",
    double velocity_scaling = 0.1,
    double acceleration_scaling = 0.1);

  geometry_msgs::msg::Quaternion rotation_vector_to_quaternion(double rx, double ry, double rz);
  moveit::planning_interface::MoveGroupInterface& getMoveGroup() { return move_group_; }

private:
  /**
   * Re-time-parameterize trajectory with scaled velocity/acceleration using MoveIt's Time Optimal Trajectory Generation
   * @param trajectory_msg Trajectory to re-time (modified in place)
   * @param velocity_scaling: Velocity scaling factor for trajectory
   * @param acceleration_scaling: Acceleration scaling factor for trajectory
   */
  void retimeTrajectory(
    moveit_msgs::msg::RobotTrajectory& trajectory_msg,
    double velocity_scaling,
    double acceleration_scaling);

  moveit::planning_interface::MoveGroupInterface& move_group_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Logger logger_;
};

}  // namespace grasping_moveit

#endif  // GRASPING_MOVEIT_MOVEIT_UTILS_HPP_



