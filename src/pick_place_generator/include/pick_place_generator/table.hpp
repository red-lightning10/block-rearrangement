#ifndef PICK_PLACE_GENERATOR_TABLE_HPP_
#define PICK_PLACE_GENERATOR_TABLE_HPP_

#include "pick_place_generator/surface.hpp"

namespace pick_place_generator {

struct RegionOfInterest {
  double min_x, min_y, max_x, max_y;
};

class Table : public Surface {
public:
  Table(const std::string& object_id);

  std::vector<GraspCandidate> samplePlace(
    const std::string& base_frame,
    double pre_grasp_offset,
    tf2_ros::Buffer& tf_buffer,
    const rclcpp::Logger& logger) const override;

private:
  RegionOfInterest region_of_interest_;
};

}  // namespace pick_place_generator

#endif  // PICK_PLACE_GENERATOR_TABLE_HPP_

