#ifndef PICK_PLACE_GENERATOR_OBJECT_HPP_
#define PICK_PLACE_GENERATOR_OBJECT_HPP_

#include <string>
#include <vector>
#include <tf2_ros/buffer.h>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace pick_place_generator {

struct GraspCandidate {
  geometry_msgs::msg::PoseStamped grasp_pose;
  geometry_msgs::msg::PoseStamped pre_grasp_pose;
};

class Object {
public:
  Object(const std::string& object_id)
    : object_id_(object_id) {}
  virtual ~Object() = default;

  virtual std::vector<GraspCandidate> sampleGrasp(
    const std::string& base_frame,
    double pre_grasp_offset,
    tf2_ros::Buffer& tf_buffer,
    const rclcpp::Logger& logger) const = 0;

  const std::string& object_id() const { return object_id_; }

protected:
  std::string object_id_;
};

}  // namespace pick_place_generator

#endif  // PICK_PLACE_GENERATOR_OBJECT_HPP_

