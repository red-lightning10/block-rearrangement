#ifndef GRASPING_MOVEIT_TASK_PARAMETERS_HPP_
#define GRASPING_MOVEIT_TASK_PARAMETERS_HPP_

#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace grasping_moveit {

struct TaskParameters {
  // Robot configuration
  std::string planning_group;
  std::string eef_group;
  std::string hand_frame;
  std::string attach_link;
  std::string base_frame;
  
  // Grasp approach/retreat distances
  double approach_object_min_dist;
  double approach_object_max_dist;
  double lift_object_min_dist;
  double lift_object_max_dist;
  
  // Cartesian planning parameters
  double cartesian_max_velocity_scaling;
  double cartesian_max_acceleration_scaling;
  double cartesian_step_size;
  double cartesian_min_fraction;
  double cartesian_jump_threshold;
  
  // Sampling planner
  std::string planner_id;
  
  // Grasp execution parameters
  double grasp_velocity;
  double grasp_acceleration;
  double grasp_eef_step;
  double retract_velocity;
  double retract_acceleration;
  
  // Home pose
  geometry_msgs::msg::PoseStamped home_pose;
};

}  // namespace grasping_moveit

#endif  // GRASPING_MOVEIT_TASK_PARAMETERS_HPP_

