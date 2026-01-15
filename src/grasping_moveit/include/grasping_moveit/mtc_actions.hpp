#ifndef GRASPING_MOVEIT_MTC_ACTIONS_HPP_
#define GRASPING_MOVEIT_MTC_ACTIONS_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include "pick_place_generator/object.hpp"
#include "grasping_moveit/task_parameters.hpp"

namespace grasping_moveit {

class MTCActions {
public:
  MTCActions(
    const TaskParameters& params,
    const rclcpp::Node::SharedPtr& node,
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client);
  
  moveit::task_constructor::Task createGraspTask(
    const std::string& object_id,
    const pick_place_generator::GraspCandidate& candidate);
  
  moveit::task_constructor::Task createRetractGraspTask(
    const std::string& object_id,
    const std::string& support_surface);
  
  moveit::task_constructor::Task createPlaceTask(
    const std::string& object_id,
    const pick_place_generator::GraspCandidate& place_candidate);
  
  moveit::task_constructor::Task createRetractPlaceTask(
    const std::string& object_id);
  
  bool commandGripper(double position, double max_effort, std::string& error);

private:
  void configureTask(moveit::task_constructor::Task& task);
  
  TaskParameters params_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner_;
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> sampling_planner_;
};

}  // namespace grasping_moveit

#endif  // GRASPING_MOVEIT_MTC_ACTIONS_HPP_

