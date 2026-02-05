#include "pick_place_generator/cube.hpp"
#include <cmath>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

namespace pick_place_generator {

Cube::Cube(const std::string& object_id, double size) 
  : Object(object_id), Surface(object_id), size_(size)
{
}

std::vector<GraspCandidate> Cube::sampleGrasp(
  const std::string& base_frame,
  double pre_grasp_offset,
  tf2_ros::Buffer& tf_buffer,
  const rclcpp::Logger& logger) const
{
  return generateTopDownGrasps(base_frame, pre_grasp_offset, tf_buffer, logger);
}

std::vector<GraspCandidate> Cube::generateTopDownGrasps(
  const std::string& base_frame,
  double pre_grasp_offset,
  tf2_ros::Buffer& tf_buffer,
  const rclcpp::Logger& logger) const
{
  std::vector<GraspCandidate> candidates;
  
  moveit::planning_interface::PlanningSceneInterface psi;
  std::string obj_id = Object::object_id();
  auto object_poses = psi.getObjectPoses({obj_id});
  
  if (object_poses.find(obj_id) == object_poses.end()) {
    RCLCPP_ERROR(logger, "Object '%s' not found in planning scene", obj_id.c_str());
    return candidates;
  }
  
  geometry_msgs::msg::Pose object_pose = object_poses[obj_id];
  
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose = object_pose;
  
  std::string object_frame = base_frame;
  if (base_frame != "world") {
    try {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf_buffer.transform(pose_stamped, transformed_pose, base_frame, tf2::durationFromSec(1.0));
      object_pose = transformed_pose.pose;
      object_frame = base_frame;
      RCLCPP_INFO(logger, "Transformed object pose from 'world' to '%s'", base_frame.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(logger, "Failed to transform pose from 'world' to '%s': %s. Using world frame.", 
                  base_frame.c_str(), ex.what());
      object_frame = "world";
    }
  }
  
  RCLCPP_INFO(logger, "Object '%s' pose in frame '%s': (%.4f, %.4f, %.4f)", 
               obj_id.c_str(), object_frame.c_str(),
               object_pose.position.x, object_pose.position.y, object_pose.position.z);
  
  
  // Align grasp yaw with the detected object yaw in the table plane.
  const double object_yaw = tf2::getYaw(object_pose.orientation);

  // Generate two top-down grasp poses aligned to object yaw (0° and 90° around Z).
  // Top-down grasp: gripper approaches from above.
  // Base orientation: gripper pointing down (pitch = π, roll = 0, yaw = varies).
  for (int i = 0; i < 2; ++i) {
    GraspCandidate candidate;
    
    // Keep wrist rotation limited by using quarter-turn alternatives.
    const double yaw = std::atan2(std::sin(object_yaw + (i * (M_PI / 2.0))),
                                  std::cos(object_yaw + (i * (M_PI / 2.0))));
    
    // Create grasp pose (at top of object)
    geometry_msgs::msg::Pose grasp_pose = convertTo6DPose(
      object_pose.position.x,
      object_pose.position.y,
      object_pose.position.z + size_/4,
      yaw);

    // Create pre-grasp pose
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
    pre_grasp_pose.position.z += pre_grasp_offset;
    
    candidate.grasp_pose.header.frame_id = object_frame;
    candidate.grasp_pose.pose = grasp_pose;
    candidate.pre_grasp_pose.header.frame_id = object_frame;
    candidate.pre_grasp_pose.pose = pre_grasp_pose;
    
    RCLCPP_DEBUG(logger, "Generated grasp pose in frame '%s': (%.3f, %.3f, %.3f), yaw=%.3f rad", 
                 object_frame.c_str(),
                 candidate.grasp_pose.pose.position.x, candidate.grasp_pose.pose.position.y, candidate.grasp_pose.pose.position.z, yaw);
    
    candidates.push_back(candidate);
  }
  
  RCLCPP_INFO(logger, "Generated %zu top-down grasp candidates for object '%s'", 
              candidates.size(), obj_id.c_str());
  
  return candidates;
}

std::vector<GraspCandidate> Cube::samplePlace(
  const std::string& base_frame,
  double pre_grasp_offset,
  tf2_ros::Buffer& tf_buffer,
  const rclcpp::Logger& logger) const
{
  std::vector<GraspCandidate> candidates = generateTopDownGrasps(base_frame, pre_grasp_offset, tf_buffer, logger);
  // we don't need the other yaw candidate...
  candidates[0].grasp_pose.pose.position.z += size_/4;
  candidates[0].pre_grasp_pose.pose.position.z += size_/4;
  return {candidates[0]};
}

}  // namespace pick_place_generator
