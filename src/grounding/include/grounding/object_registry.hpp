#ifndef GROUNDING_OBJECT_REGISTRY_HPP_
#define GROUNDING_OBJECT_REGISTRY_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace grounding
{

struct TrackedObject
{
  std::string name;
  std::string class_name;
  geometry_msgs::msg::Pose pose;
  rclcpp::Time last_seen;
  bool visible = true;
  bool attached = false;
};

struct ObjectDetection
{
  std::string class_name;
  geometry_msgs::msg::Pose pose;
  std::string source_id;
};

class ObjectRegistry
{
public:
  explicit ObjectRegistry(double match_distance = 0.05);

  const std::unordered_map<std::string, TrackedObject> & objects() const;
  TrackedObject * get(const std::string & name);

  void update(const std::vector<ObjectDetection> & detections, const rclcpp::Time & stamp);
  void set_attached(const std::string & name, bool attached);
  void update_pose(
    const std::string & name,
    const geometry_msgs::msg::Pose & pose,
    const rclcpp::Time & stamp);
  void remove(const std::string & name);
  void clear_attached();

private:
  double match_distance_;
  std::unordered_map<std::string, TrackedObject> objects_;
};

}  // namespace grounding

#endif  // GROUNDING_OBJECT_REGISTRY_HPP_
