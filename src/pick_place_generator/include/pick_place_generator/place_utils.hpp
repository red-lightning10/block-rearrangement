#ifndef PICK_PLACE_GENERATOR_PLACE_UTILS_HPP_
#define PICK_PLACE_GENERATOR_PLACE_UTILS_HPP_

#include <string>
#include <vector>
#include <tf2_ros/buffer.h>
#include <rclcpp/logger.hpp>
#include "pick_place_generator/object.hpp"
#include "pick_place_generator/surface.hpp"

namespace pick_place_generator {

std::vector<GraspCandidate> getPlaceCandidates(
  const Object& obj_a,
  const Surface& obj_b,
  const std::string& base_frame,
  double pre_grasp_offset,
  double threshold,
  tf2_ros::Buffer& tf_buffer,
  const rclcpp::Logger& logger);

}  // namespace pick_place_generator

#endif  // PICK_PLACE_GENERATOR_PLACE_UTILS_HPP_

