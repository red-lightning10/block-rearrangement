#ifndef SPAWN_OBJECTS_SPAWN_OBJECTS_HPP_
#define SPAWN_OBJECTS_SPAWN_OBJECTS_HPP_

#include <array>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <interfaces/msg/detection.hpp>
#include <interfaces/msg/detection_array.hpp>
#include <interfaces/srv/project_to3_d.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

namespace spawn_objects
{

class SpawnObjects
{
public:
  SpawnObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
               tf2_ros::Buffer &tf_buffer,
               const rclcpp::Client<interfaces::srv::ProjectTo3D>::SharedPtr &project_client,
               double cube_size,
               rclcpp::Logger logger);

  int SpawnFromDetections(const std::vector<interfaces::msg::Detection> &detections);
  int ClearAll();

private:
  std::string MakeObjectName(int class_id, size_t index) const;
  geometry_msgs::msg::Point ComputeCubeCenter(
    const geometry_msgs::msg::Point &world_pos_base, size_t detection_index) const;
  moveit_msgs::msg::CollisionObject MakeCollisionObject(
    const std::string &id,
    const geometry_msgs::msg::Point &center,
    double orientation_deg) const;

  moveit::planning_interface::PlanningSceneInterface &planning_scene_interface_;
  tf2_ros::Buffer &tf_buffer_;
  rclcpp::Client<interfaces::srv::ProjectTo3D>::SharedPtr project_client_;
  double cube_size_;
  rclcpp::Logger logger_;
  std::set<std::string> spawned_objects_;
};

}  // namespace spawn_objects

#endif  // SPAWN_OBJECTS_SPAWN_OBJECTS_HPP_


