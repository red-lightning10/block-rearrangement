#include "pick_place_generator/place_utils.hpp"
#include "pick_place_generator/surface.hpp"
#include "pick_place_generator/cube.hpp"
#include "pick_place_generator/table.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <stdexcept>

namespace pick_place_generator {

std::vector<GraspCandidate> getPlaceCandidates(
  const Object& obj_a,
  const Surface& obj_b,
  const std::string& base_frame,
  double pre_grasp_offset,
  double threshold,
  tf2_ros::Buffer& tf_buffer,
  const rclcpp::Logger& logger)
{
  if (dynamic_cast<const Table*>(&obj_a)) {
    RCLCPP_ERROR(logger, "Object to be placed should not be a surface-only object (like Table). Object ID: '%s'", obj_a.object_id().c_str());
    return std::vector<GraspCandidate>();
  }
  
  // Get size from obj_a (only Cube has size() method)
  const Cube* cube_a = dynamic_cast<const Cube*>(&obj_a);
  if (!cube_a) {
    throw std::runtime_error("Object to be placed must have size parameter. Object ID: " + obj_a.object_id());
  }
  
  // Get place candidates on top of obj_b using samplePlace
  std::vector<GraspCandidate> place_candidates = obj_b.samplePlace(
    base_frame, pre_grasp_offset, tf_buffer, logger);
  
    RCLCPP_INFO(logger, "place_candidates: %zu", place_candidates.size());
    for (auto& candidate : place_candidates) {
      RCLCPP_INFO(logger, "candidate.grasp_pose.pose.position.z: %.4f", candidate.grasp_pose.pose.position.z);
    }

  // Adjust z coordinate of place based on held object
  double z_adjustment = cube_a->size() * 0.75 + threshold;

  for (auto& candidate : place_candidates) {
    candidate.grasp_pose.pose.position.z += z_adjustment;
    candidate.pre_grasp_pose.pose.position.z += z_adjustment;
  }

  RCLCPP_INFO(logger, 
              "Generated %zu place candidates for placing object '%s' (size=%.4f) on '%s'",
              place_candidates.size(), obj_a.object_id().c_str(), cube_a->size(), 
              obj_b.object_id().c_str());

  return place_candidates;
}

}  // namespace pick_place_generator

