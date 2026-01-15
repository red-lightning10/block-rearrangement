#include "grasping_moveit/moveit_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <map>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

namespace grasping_moveit {

geometry_msgs::msg::Quaternion MoveGroupUtilities::rotation_vector_to_quaternion(
    double rx, double ry, double rz)
{
  // Create Eigen rotation vector and convert directly to quaternion
  Eigen::Vector3d rotation_vec(rx, ry, rz);
  Eigen::AngleAxisd angle_axis(rotation_vec.norm(), rotation_vec.normalized());
  Eigen::Quaterniond eigen_quat(angle_axis);
  
  // Convert Eigen quaternion to ROS message
  geometry_msgs::msg::Quaternion q;
  q.w = eigen_quat.w();
  q.x = eigen_quat.x();
  q.y = eigen_quat.y();
  q.z = eigen_quat.z();
  
  return q;
}

void MoveGroupUtilities::retimeTrajectory(
  moveit_msgs::msg::RobotTrajectory& trajectory_msg,
  double velocity_scaling,
  double acceleration_scaling)
{
  // Validate trajectory before retiming
  if (trajectory_msg.joint_trajectory.points.empty()) {
    throw std::runtime_error("Cannot re-time-parameterize: trajectory is empty");
  }
  
  if (trajectory_msg.joint_trajectory.joint_names.empty()) {
    throw std::runtime_error("Cannot re-time-parameterize: trajectory has no joint names");
  }
  
  if (trajectory_msg.joint_trajectory.points[0].positions.empty()) {
    throw std::runtime_error("Cannot re-time-parameterize: trajectory start state has no positions");
  }
  
  // Get robot model and group name
  const moveit::core::RobotModelConstPtr& robot_model = move_group_.getRobotModel();
  const std::string group_name = move_group_.getName();
  
  // Create robot state from trajectory start state (first point)
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setJointGroupPositions(group_name, trajectory_msg.joint_trajectory.points[0].positions);
  robot_state.update();
  
  // Convert message to RobotTrajectory object
  robot_trajectory::RobotTrajectory robot_trajectory(robot_model, group_name);
  robot_trajectory.setRobotTrajectoryMsg(robot_state, trajectory_msg);
  
  // Re-time-parameterize with TOTG using velocity/acceleration scaling factors
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(robot_trajectory, velocity_scaling, acceleration_scaling);
  
  // Convert back to message
  robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);
}

bool MoveGroupUtilities::moveL(
  const geometry_msgs::msg::Pose& target_pose,
  double velocity_scaling,
  double acceleration_scaling,
  double eef_step)
{
  
  RCLCPP_DEBUG(logger_, "  Position: (%.4f, %.4f, %.4f)", 
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  RCLCPP_DEBUG(logger_, "  Orientation: (%.4f, %.4f, %.4f, %.4f)", 
              target_pose.orientation.x, target_pose.orientation.y,
              target_pose.orientation.z, target_pose.orientation.w);
  RCLCPP_DEBUG(logger_, "  MoveGroup reference frame: %s", move_group_.getPoseReferenceFrame().c_str());
  RCLCPP_DEBUG(logger_, "  End-effector link: %s", move_group_.getEndEffectorLink().c_str());
  
  // Get current pose for comparison
  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose();
  RCLCPP_DEBUG(logger_, "  Current end-effector pose: (%.4f, %.4f, %.4f)", 
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  
  // Set velocity/acceleration limits
  move_group_.setMaxVelocityScalingFactor(velocity_scaling);
  move_group_.setMaxAccelerationScalingFactor(acceleration_scaling);
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);   // computeCartesianPath starts from current state, so only target waypoint is needed
  
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;  // Disable jump threshold check for now
  
  RCLCPP_DEBUG(logger_, "  Calling computeCartesianPath() with eef_step=%.4f", eef_step);
  double fraction = move_group_.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);
    
  RCLCPP_DEBUG(logger_, 
              "Cartesian path computed: %.2f%% achieved (need 95%%)", fraction * 100.0);
  
  // Check if path was computed successfully
  if (fraction < 0.95) {
    RCLCPP_WARN(logger_, 
                "Cartesian path failed: only %.2f%% achieved (need 95%%)", fraction * 100.0);
    return false;
  }
  
  // Re-time-parameterize with speed limits after obtainting trajectory
  try {
    retimeTrajectory(trajectory, velocity_scaling, acceleration_scaling);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Trajectory retiming failed: %s", e.what());
    return false;
  }
  
  // Execute trajectory after retiming
  auto result = move_group_.execute(trajectory);
  return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool MoveGroupUtilities::movePlan(
  const std::vector<double>& joint_values,
  const std::string& planner_id,
  double velocity_scaling,
  double acceleration_scaling)
{
  try {
    // Set planner
    //TODO: Check if this is correct
    if (planner_id == "ompl") {
      move_group_.setPlannerId("RRTConnect");
    } else if (planner_id == "chomp" || planner_id.empty()) {
      move_group_.setPlannerId("chomp_interface/CHOMPPlanner");
    } else {
      move_group_.setPlannerId(planner_id);
    }
    RCLCPP_INFO(logger_, "Using planner: %s for joint space planning", move_group_.getPlannerId().c_str());
    
    move_group_.setPlanningTime(10.0);  // Give more time to optimize
    
    // Set number of planning attempts
    move_group_.setNumPlanningAttempts(5);
    
    // Set velocity/acceleration limits
    move_group_.setMaxVelocityScalingFactor(velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scaling);
    
    // Set joint space target
    move_group_.setJointValueTarget(joint_values);
    
    // Set goal tolerance in joint space
    move_group_.setGoalJointTolerance(0.01);  // 0.01 overall rad tolerance
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = static_cast<bool>(move_group_.plan(plan));
    
    if (!plan_success) {
      RCLCPP_WARN(logger_, "Planning failed with planner %s", move_group_.getPlannerId().c_str());
      return false;
    }
    
    RCLCPP_DEBUG(logger_, "Planning succeeded. Trajectory has %zu waypoints", 
                plan.trajectory_.joint_trajectory.points.size());
    
    // Re-time-parameterize with speed limits
    try {
      retimeTrajectory(plan.trajectory_, velocity_scaling, acceleration_scaling);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Trajectory retiming failed: %s", e.what());
      return false;
    }
    
    // Execute trajectory
    auto result = move_group_.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger_, "Successfully executed planned trajectory");
      return true;
    } else {
      RCLCPP_WARN(logger_, "Execution failed with error code: %d", result.val);
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in movePlan: %s", e.what());
    return false;
  }
}

} // namespace grasping_moveit
