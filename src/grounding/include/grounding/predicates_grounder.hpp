#ifndef GROUNDING_PREDICATES_GROUNDER_HPP_
#define GROUNDING_PREDICATES_GROUNDER_HPP_

#include "grounding/object_registry.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <vector>

namespace grounding
{

struct GroundedState
{
  std::vector<std::string> predicates;
};

class PredicatesGrounder
{
public:
  explicit PredicatesGrounder(rclcpp::Node * node);

  void refresh_from_moveit();
  GroundedState compute_state();

  ObjectRegistry & registry() { return registry_; }

  // Adjust object position based on a predicate
  bool adjustObject(const std::string& object_id, const std::string& predicate);

  // Adjust all objects based on their predicates in the current grounded state
  void adjustObjects();

private:
  bool is_on_table(const TrackedObject & obj);
  bool is_on(const TrackedObject & top, const TrackedObject & bottom) const;
  bool is_clear(const TrackedObject & obj) const;
  bool is_holding(const TrackedObject & obj) const;

  rclcpp::Node * node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ObjectRegistry registry_;

  // Parameters
  double table_height_;
  double cube_height_;
  double on_tolerance_;
  double lateral_threshold_;
  double placement_clearance_;  // Clearance gap to prevent collisions when adjusting positions
  double match_distance_;
  double tf_timeout_;
  std::string planning_frame_;
  std::string table_base_name_;
  std::vector<std::string> ignored_objects_;

  bool debug_logging_;
};

}  // namespace grounding

#endif  // GROUNDING_PREDICATES_GROUNDER_HPP_
