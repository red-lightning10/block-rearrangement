#include "pick_place_generator/table.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <cmath>

namespace pick_place_generator {

Table::Table(const std::string& object_id)
: Surface(object_id)
{
  // Hardcoded RegionOfInterest: x from 0 to 0.3, y from -0.65 to -0.9
  region_of_interest_.min_x = 0.0;
  region_of_interest_.max_x = 0.3;
  region_of_interest_.min_y = -0.9;
  region_of_interest_.max_y = -0.65;
}

std::vector<GraspCandidate> Table::samplePlace(
  const std::string& base_frame,
  double pre_grasp_offset,
  tf2_ros::Buffer& /*tf_buffer*/,
  const rclcpp::Logger& logger) const
{
  std::vector<GraspCandidate> candidates;
  candidates.reserve(1000);  // Reserve space for 1000 candidates
  
  // Random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> x_dist(region_of_interest_.min_x, region_of_interest_.max_x);
  std::uniform_real_distribution<double> y_dist(region_of_interest_.min_y, region_of_interest_.max_y);
  
  // Generate 1000 random (x, y) samples from region of interest
  const int num_samples = 1000;
  for (int i = 0; i < num_samples; ++i) {
    // Sample random point in region of interest
    double x = x_dist(gen);
    double y = y_dist(gen);
    
    // Generate two poses with different yaw orientations (0° and 180°)
      
    // Create grasp pose using helper function
    geometry_msgs::msg::Pose grasp_pose = convertTo6DPose(x, y, 0.0, 0.0);  // z = 0.0 (table surface)
    
    // Create pre-grasp pose
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
    pre_grasp_pose.position.z += pre_grasp_offset;
    
    GraspCandidate candidate;
    candidate.grasp_pose.header.frame_id = base_frame;
    candidate.grasp_pose.pose = grasp_pose;
    candidate.pre_grasp_pose.header.frame_id = base_frame;
    candidate.pre_grasp_pose.pose = pre_grasp_pose;
    
    candidates.push_back(candidate);
  }
  
  RCLCPP_INFO(logger, "Generated %zu placement candidates for table '%s' (1000 samples)", 
              candidates.size(), object_id().c_str());
  
  return candidates;
}

}  // namespace pick_place_generator

