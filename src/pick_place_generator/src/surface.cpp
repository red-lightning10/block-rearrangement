#include "pick_place_generator/surface.hpp"
#include <Eigen/Geometry>

namespace pick_place_generator {

geometry_msgs::msg::Pose Surface::convertTo6DPose(double x, double y, double z, double yaw) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  
  // Construct quaternion from RPY: roll=0.0, pitch=M_PI, yaw=varies
  Eigen::AngleAxisd roll_angle(0.0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(M_PI, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat = yaw_angle * pitch_angle * roll_angle;
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  
  return pose;
}

}  // namespace pick_place_generator

