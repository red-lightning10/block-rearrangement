#include "spawn_objects/spawn_objects.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace spawn_objects
{
SpawnObjects::SpawnObjects(
  moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
  tf2_ros::Buffer &tf_buffer,
  const rclcpp::Client<interfaces::srv::ProjectTo3D>::SharedPtr &project_client,
  double cube_size,
  rclcpp::Logger logger)
: planning_scene_interface_(planning_scene_interface),
  tf_buffer_(tf_buffer),
  project_client_(project_client),
  cube_size_(cube_size),
  logger_(std::move(logger))
{
}

int SpawnObjects::SpawnFromDetections(const std::vector<interfaces::msg::Detection> &detections)
{
  RCLCPP_INFO(logger_, "Spawning %zu objects in planning scene", detections.size());

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::unordered_map<int, size_t> color_index_by_class;
  int spawned_count = 0, detection_index = 0;

  for (const auto &detection : detections) {
    const size_t color_index = color_index_by_class[detection.class_id]++;
    std::string obj_name = MakeObjectName(detection.class_id, color_index);

    if (spawned_objects_.find(obj_name) != spawned_objects_.end()) {
      RCLCPP_WARN(logger_, "Object %s already spawned, skipping", obj_name.c_str());
      detection_index++;
      continue;
    }

    auto project_request = std::make_shared<interfaces::srv::ProjectTo3D::Request>();
    project_request->centroid = detection.centroid;
    project_request->depth = detection.depth;

    auto project_future = project_client_->async_send_request(project_request);
    if (project_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN(logger_, "Failed to project detection %d to 3D coords (timeout)", detection_index);
      detection_index++;
      continue;
    }

    auto project_response = project_future.get();
    if (!project_response->success) {
      RCLCPP_WARN(
        logger_, "3D projection failed for detection %d: %s", detection_index,
        project_response->message.c_str());
      detection_index++;
      continue;
    }

    geometry_msgs::msg::Point cube_center_base = 
      ComputeCubeCenter(project_response->world_coords, detection_index);

    moveit_msgs::msg::CollisionObject collision_object =
      MakeCollisionObject(obj_name, cube_center_base, detection.orientation);

    collision_objects.push_back(collision_object);
    spawned_objects_.insert(obj_name);
    spawned_count++;
    detection_index++;

    RCLCPP_INFO(
      logger_,
      "Spawned %s at (%.3f, %.3f, %.3f) in ur10_base frame",
      obj_name.c_str(),
      cube_center_base.x, cube_center_base.y, cube_center_base.z);
  }

  if (spawned_count > 0) {
    bool success = planning_scene_interface_.applyCollisionObjects(collision_objects);
    if (success) {
      RCLCPP_INFO(logger_, "Successfully spawned %d objects in planning scene", spawned_count);
    } else {
      RCLCPP_ERROR(logger_, "Failed to apply collision objects to planning scene");
    }
  }

  return spawned_count;
}

int SpawnObjects::ClearAll()
{
  RCLCPP_INFO(logger_, "Clearing all spawned objects from planning scene");

  if (spawned_objects_.empty()) {
    RCLCPP_INFO(logger_, "No objects to clear");
    return 0;
  }

  std::vector<std::string> object_ids(spawned_objects_.begin(), spawned_objects_.end());

  // Remove objects and apply the changes to the planning scene
  std::vector<moveit_msgs::msg::CollisionObject> remove_objects;
  for (const auto &id : object_ids) {
    moveit_msgs::msg::CollisionObject remove_obj;
    remove_obj.id = id;
    remove_obj.operation = remove_obj.REMOVE;
    remove_objects.push_back(remove_obj);
  }
  planning_scene_interface_.applyCollisionObjects(remove_objects);
  spawned_objects_.clear();

  RCLCPP_INFO(logger_, "Cleared %zu objects", object_ids.size());
  return object_ids.size();
}

std::string SpawnObjects::MakeObjectName(int class_id, size_t index) const
{
  std::string object_prefix = "cube";
  if (class_id == 0) {
    object_prefix = "blue_cube";
  } else if (class_id == 1) {
    object_prefix = "green_cube";
  } else if (class_id == 2) {
    object_prefix = "orange_cube";
  } else if (class_id == 3) {
    object_prefix = "red_cube";
  } else if (class_id == 4) {
    object_prefix = "yellow_cube";
  }

  std::ostringstream oss;
  oss << object_prefix << "_" << index;
  return oss.str();
}

geometry_msgs::msg::Point SpawnObjects::ComputeCubeCenter(
  const geometry_msgs::msg::Point &world_pos_base, size_t detection_index) const
{
  geometry_msgs::msg::Point cube_center_base;
  try {
    const auto cam_to_base = tf_buffer_.lookupTransform(
      "ur10_base",
      "camera_color_optical_frame",
      tf2::TimePointZero,
      std::chrono::seconds(1));
    RCLCPP_INFO(logger_, "Found camera frame: camera_color_optical_frame");
    
    Eigen::Quaterniond quat(
      cam_to_base.transform.rotation.w,
      cam_to_base.transform.rotation.x,
      cam_to_base.transform.rotation.y,
      cam_to_base.transform.rotation.z);
    Eigen::Matrix3d rot_matrix = quat.toRotationMatrix();
    Eigen::Vector3d camera_z_axis = rot_matrix.col(2);

    /* Logic being that going in the direction of camera's z-axis by an offset of half cube size will
    give us the center of the cube in the base frame. 
    This only works if the camera is looking at the cube from above or sidewise.
    Future work: Use features of object to spawn them accordingly (like corners of cube).*/
    Eigen::Vector3d offset = (cube_size_ / 2.0) * camera_z_axis;
    cube_center_base.x = world_pos_base.x + offset.x();
    cube_center_base.y = world_pos_base.y + offset.y();
    cube_center_base.z = world_pos_base.z + offset.z();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
      logger_, "Detection %zu: using simple z-offset; camera transform failed: %s",
      detection_index, e.what());
    cube_center_base.x = world_pos_base.x;
    cube_center_base.y = world_pos_base.y;
    cube_center_base.z = world_pos_base.z + cube_size_ / 2.0;
  }

  return cube_center_base;
}

moveit_msgs::msg::CollisionObject SpawnObjects::MakeCollisionObject(
  const std::string &id,
  const geometry_msgs::msg::Point &center,
  double orientation_deg) const
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "ur10_base";
  collision_object.header.stamp = rclcpp::Clock().now();
  collision_object.id = id;
  collision_object.operation = collision_object.ADD;

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = cube_size_;
  box.dimensions[1] = cube_size_;
  box.dimensions[2] = cube_size_;
  collision_object.primitives.push_back(box);

  geometry_msgs::msg::Pose pose;
  pose.position = center;

  double orientation_rad = orientation_deg * M_PI / 180.0;
  pose.orientation.z = std::sin(orientation_rad / 2.0);
  pose.orientation.w = std::cos(orientation_rad / 2.0);

  collision_object.primitive_poses.push_back(pose);
  return collision_object;
}

}  // namespace spawn_objects
