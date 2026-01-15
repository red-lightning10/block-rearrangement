#include "grounding/object_registry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace grounding
{

ObjectRegistry::ObjectRegistry(double match_distance)
: match_distance_(match_distance)
{
  objects_.reserve(64);
}

const std::unordered_map<std::string, TrackedObject> & ObjectRegistry::objects() const
{
  return objects_;
}

TrackedObject * ObjectRegistry::get(const std::string & name)
{
  auto it = objects_.find(name);
  if (it == objects_.end()) {
    return nullptr;
  }
  return &it->second;
}

void ObjectRegistry::update(
  const std::vector<ObjectDetection> & detections,
  const rclcpp::Time & stamp)
{
  std::unordered_map<std::string, bool> matched;

  for (const auto & detection : detections) {
    auto it = objects_.find(detection.class_name);
    if (it == objects_.end()) {
      TrackedObject tracked;
      tracked.name = detection.class_name;
      tracked.class_name = detection.class_name;
      tracked.pose = detection.pose;
      tracked.last_seen = stamp;
      tracked.visible = true;
      tracked.attached = false;
      objects_[tracked.name] = tracked;
      matched[tracked.name] = true;
      continue;
    }

    it->second.pose = detection.pose;
    it->second.last_seen = stamp;
    it->second.visible = true;
    matched[it->second.name] = true;
  }

  for (auto & [name, obj] : objects_) {
    if (!matched.count(name)) {
      obj.visible = false;
    }
  }
}

void ObjectRegistry::set_attached(const std::string & name, bool attached)
{
  if (auto * obj = get(name)) {
    obj->attached = attached;
  }
}

void ObjectRegistry::update_pose(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp)
{
  if (auto * obj = get(name)) {
    obj->pose = pose;
    obj->last_seen = stamp;
  }
}

void ObjectRegistry::remove(const std::string & name)
{
  objects_.erase(name);
}

void ObjectRegistry::clear_attached()
{
  for (auto & [name, obj] : objects_) {
    (void)name;
    obj.attached = false;
  }
}

}  // namespace grounding


