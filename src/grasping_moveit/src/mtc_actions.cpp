#include "grasping_moveit/mtc_actions.hpp"
#include "pick_place_generator/object.hpp"

#include <memory>
#include <chrono>
#include <future>
#include <Eigen/Geometry>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/container.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace grasping_moveit {

namespace mtc = moveit::task_constructor;
using pick_place_generator::GraspCandidate;

MTCActions::MTCActions(
  const TaskParameters& params,
  const rclcpp::Node::SharedPtr& node,
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client)
  : params_(params), node_(node), gripper_client_(gripper_client)
{
  // Initialize cartesian planner
  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(params.cartesian_max_velocity_scaling);
  cartesian_planner_->setMaxAccelerationScalingFactor(params.cartesian_max_acceleration_scaling);
  cartesian_planner_->setStepSize(params.cartesian_step_size);
  cartesian_planner_->setMinFraction(params.cartesian_min_fraction);
  cartesian_planner_->setJumpThreshold(params.cartesian_jump_threshold);
  
  // Initialize sampling planner
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  sampling_planner_->setPlannerId(params.planner_id);
}

void MTCActions::configureTask(mtc::Task& task)
{
  task.setProperty("group", params_.planning_group);
  task.setProperty("eef", params_.eef_group);
  task.setProperty("hand", params_.eef_group);
  task.setProperty("hand_grasping_frame", params_.hand_frame);
  task.setProperty("ik_frame", params_.hand_frame);
}

mtc::Task MTCActions::createGraspTask(
  const std::string& object_id,
  const GraspCandidate& candidate)
{
  mtc::Task task;
  task.stages()->setName("grasp task");
  task.loadRobotModel(node_);
  configureTask(task);

  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current_state));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to pre-grasp", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(candidate.pre_grasp_pose);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("approach grasp", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(candidate.grasp_pose);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions(
      object_id,
      task.getRobotModel()->getJointModelGroup(params_.eef_group)->getLinkModelNamesWithCollisionGeometry(),
      true);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCActions::createRetractGraspTask(
  const std::string& object_id,
  const std::string& support_surface)
{
  mtc::Task task;
  task.stages()->setName("retract grasp task");
  task.loadRobotModel(node_);

  task.setProperty("group", params_.planning_group);
  task.setProperty("eef", params_.eef_group);
  task.setProperty("hand", params_.eef_group);

  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current_state));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
    stage->allowCollisions({object_id}, { support_surface }, true);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(object_id, params_.hand_frame);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(params_.hand_frame);
    stage->setMinMaxDistance(params_.lift_object_min_dist, params_.lift_object_max_dist);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = params_.base_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,support)");
    stage->allowCollisions({object_id}, { support_surface }, false);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home (retract)", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(params_.home_pose);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCActions::createPlaceTask(
  const std::string& object_id,
  const GraspCandidate& place_candidate)
{
  mtc::Task task;
  task.stages()->setName("place task");
  task.loadRobotModel(node_);
  configureTask(task);

  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    auto predicate = std::make_unique<mtc::stages::PredicateFilter>("verify attached object", std::move(current_state));
    predicate->setPredicate([object_id](const mtc::SolutionBase& solution, std::string& comment) {
      const auto& state = solution.start()->scene()->getCurrentState();
      if (!state.hasAttachedBody(object_id)) {
        comment = "requested object '" + object_id + "' is not attached";
        return false;
      }
      return true;
    });
    task.add(std::move(predicate));
  }

  {
    auto place_container = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place_container->properties(), { "eef", "hand", "group" });
    place_container->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group" });

    {
      auto move_to_pre_place = std::make_unique<mtc::stages::MoveTo>("move to pre-place", cartesian_planner_);
      move_to_pre_place->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      move_to_pre_place->setGoal(place_candidate.pre_grasp_pose);
      place_container->insert(std::move(move_to_pre_place));
    }

    {
      auto move_to_place = std::make_unique<mtc::stages::MoveTo>("move to place location", cartesian_planner_);
      move_to_place->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      move_to_place->setGoal(place_candidate.grasp_pose);
      place_container->insert(std::move(move_to_place));
    }

    task.add(std::move(place_container));
  }

  return task;
}

mtc::Task MTCActions::createRetractPlaceTask(
  const std::string& object_id)
{
  mtc::Task task;
  task.stages()->setName("retract place");
  task.loadRobotModel(node_);

  task.setProperty("group", params_.planning_group);
  task.setProperty("eef", params_.eef_group);
  task.setProperty("hand", params_.eef_group);
  task.setProperty("hand_grasping_frame", params_.hand_frame);
  task.setProperty("ik_frame", params_.hand_frame);

  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current_state));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(object_id, params_.hand_frame);
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(params_.hand_frame);
    stage->setMinMaxDistance(params_.lift_object_min_dist, params_.lift_object_max_dist);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = params_.hand_frame;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  {
    auto move_to_home = std::make_unique<mtc::stages::MoveTo>("move to home (retract)", cartesian_planner_);
    move_to_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    move_to_home->setGoal(params_.home_pose);
    task.add(std::move(move_to_home));
  }

  return task;
}

bool MTCActions::commandGripper(double position, double max_effort, std::string& error)
{
  using namespace std::chrono_literals;
  
  if (!gripper_client_) {
    error = "gripper action client not initialized";
    return false;
  }

  if (!gripper_client_->wait_for_action_server(10s)) {
    error = "gripper action server unavailable";
    return false;
  }

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  auto goal_handle_future = gripper_client_->async_send_goal(goal);
  if (goal_handle_future.wait_for(10s) != std::future_status::ready) {
    error = "gripper action goal send timeout";
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    error = "gripper action goal rejected";
    return false;
  }

  auto result_future = gripper_client_->async_get_result(goal_handle);
  if (result_future.wait_for(10s) != std::future_status::ready) {
    error = "gripper action result timeout";
    return false;
  }

  auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    error = "gripper action failed";
    return false;
  }

  return true;
}

}  // namespace grasping_moveit

