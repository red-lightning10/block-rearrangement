#ifndef PICK_PLACE_GENERATOR_CUBE_HPP_
#define PICK_PLACE_GENERATOR_CUBE_HPP_

#include "pick_place_generator/object.hpp"
#include "pick_place_generator/surface.hpp"

namespace pick_place_generator {

class Cube : public Object, public Surface {
public:
  Cube(const std::string& object_id, double size);

  std::vector<GraspCandidate> sampleGrasp(
    const std::string& base_frame,
    double pre_grasp_offset,
    tf2_ros::Buffer& tf_buffer,
    const rclcpp::Logger& logger) const override;
  
  std::vector<GraspCandidate> samplePlace(
    const std::string& base_frame,
    double pre_grasp_offset,
    tf2_ros::Buffer& tf_buffer,
    const rclcpp::Logger& logger) const override;

  double size() const { return size_; }

private:
  std::vector<GraspCandidate> generateTopDownGrasps(
    const std::string& base_frame,
    double pre_grasp_offset,
    tf2_ros::Buffer& tf_buffer,
    const rclcpp::Logger& logger) const;
  
  double size_;
};

}  // namespace pick_place_generator

#endif  // PICK_PLACE_GENERATOR_CUBE_HPP_



