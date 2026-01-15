#ifndef PICK_PLACE_GENERATOR_SURFACE_HPP_
#define PICK_PLACE_GENERATOR_SURFACE_HPP_

#include <string>
#include <vector>
#include <tf2_ros/buffer.h>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include "pick_place_generator/object.hpp"

namespace pick_place_generator {

class Surface {
public:
  Surface(const std::string& object_id)
    : object_id_(object_id) {}
  virtual ~Surface() = default;

  virtual std::vector<GraspCandidate> samplePlace(
    const std::string& base_frame,
    double pre_grasp_offset,
    tf2_ros::Buffer& tf_buffer,
    const rclcpp::Logger& logger) const = 0;

  const std::string& object_id() const { return object_id_; }

protected:
  geometry_msgs::msg::Pose convertTo6DPose(double x, double y, double z, double yaw) const;

  std::string object_id_;
};

}  // namespace pick_place_generator

#endif  // PICK_PLACE_GENERATOR_SURFACE_HPP_

