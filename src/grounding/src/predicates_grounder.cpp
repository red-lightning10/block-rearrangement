#include "grounding/predicates_grounder.hpp"

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sstream>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace grounding
{

PredicatesGrounder::PredicatesGrounder(rclcpp::Node * node)
: node_(node),
  planning_scene_interface_(""),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_)
{
  table_height_ = node_->declare_parameter<double>("table_height");
  cube_height_ = node_->declare_parameter<double>("cube_height");
  on_tolerance_ = node_->declare_parameter<double>("on_tolerance");
  lateral_threshold_ = node_->declare_parameter<double>("lateral_threshold");
  placement_clearance_ = node_->declare_parameter<double>("placement_clearance", 0.002);  // Default 2mm clearance
  match_distance_ = node_->declare_parameter<double>("match_distance");
  tf_timeout_ = node_->declare_parameter<double>("tf_timeout");
  planning_frame_ = node_->declare_parameter<std::string>("planning_frame");
  table_base_name_ = node_->declare_parameter<std::string>("table_base_name");
  ignored_objects_ = node_->declare_parameter<std::vector<std::string>>("ignored_objects");
  debug_logging_ = node_->declare_parameter<bool>("debug_logging");

  registry_ = ObjectRegistry(match_distance_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[grounder] debug logging %s",
    debug_logging_ ? "enabled" : "disabled");
}

void PredicatesGrounder::refresh_from_moveit()
{
  const auto object_ids = planning_scene_interface_.getKnownObjectNames();
  if (debug_logging_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[grounder] known objects: %zu", object_ids.size());
  }
  if (object_ids.empty()) {
    return;
  }
  const auto poses = planning_scene_interface_.getObjectPoses(object_ids);

  std::vector<ObjectDetection> detections;
  detections.reserve(poses.size());
  std::vector<std::string> attached_ids;
  attached_ids.reserve(4);

  for (const auto & id_pose : poses) {
    // Skip ignored objects
    if (std::find(ignored_objects_.begin(), ignored_objects_.end(), id_pose.first) != ignored_objects_.end()) {
      continue;
    }
    ObjectDetection det;
    det.class_name = id_pose.first;  // assume id already encodes class
    det.pose = geometry_msgs::msg::Pose();
    det.pose.position = id_pose.second.position;
    det.pose.orientation = id_pose.second.orientation;
    detections.push_back(det);
    if (debug_logging_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[grounder] detection raw pose %s -> (%.4f, %.4f, %.4f)",
        det.class_name.c_str(),
        det.pose.position.x,
        det.pose.position.y,
        det.pose.position.z);
    }
  }

  const auto attached_objects = planning_scene_interface_.getAttachedObjects();
  for (const auto & [id, attached] : attached_objects) {
    geometry_msgs::msg::Pose relative_pose;
    if (!attached.object.primitive_poses.empty()) {
      relative_pose = attached.object.primitive_poses.front();
    } else if (!attached.object.mesh_poses.empty()) {
      relative_pose = attached.object.mesh_poses.front();
    } else {
      relative_pose.orientation.w = 1.0;
    }

    geometry_msgs::msg::PoseStamped pose_link;
    pose_link.header.frame_id = attached.link_name;
    pose_link.pose = relative_pose;

    geometry_msgs::msg::PoseStamped pose_world;
    try {
      pose_world = tf_buffer_.transform(pose_link, planning_frame_, tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "[grounder] failed to transform attached object %s: %s",
        id.c_str(), ex.what());
      continue;
    }

    ObjectDetection det;
    det.class_name = id;
    det.pose = pose_world.pose;
    detections.push_back(det);
    attached_ids.push_back(id);

    if (debug_logging_) {
      const auto & p = det.pose.position;
      RCLCPP_INFO(
        node_->get_logger(),
        "[grounder] attached object pose %s -> (%.4f, %.4f, %.4f)",
        det.class_name.c_str(), p.x, p.y, p.z);
    }
  }

  registry_.update(detections, node_->get_clock()->now());
  registry_.clear_attached();
  for (const auto & id : attached_ids) {
    registry_.set_attached(id, true);
  }
}

GroundedState PredicatesGrounder::compute_state()
{
  GroundedState state;
  bool holding_any = false;
  const auto & objects = registry_.objects();

  if (debug_logging_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[grounder] computing state for %zu objects", objects.size());
  }

  for (const auto & [name, obj] : objects) {
    if (debug_logging_) {
      const auto & p = obj.pose.position;
      RCLCPP_INFO(
        node_->get_logger(),
        "[grounder] object=%s pos=(%.4f, %.4f, %.4f) attached=%s",
        name.c_str(), p.x, p.y, p.z, obj.attached ? "true" : "false");
    }
    if (is_on_table(obj)) {
      state.predicates.emplace_back("(ONTABLE " + name + ")");
    }
    if (is_clear(obj)) {
      state.predicates.emplace_back("(CLEAR " + name + ")");
    }
    if (is_holding(obj)) {
      state.predicates.emplace_back("(HOLDING " + name + ")");
      holding_any = true;
    }
  }

  for (const auto & [name_a, obj_a] : objects) {
    for (const auto & [name_b, obj_b] : objects) {
      if (name_a == name_b) {
        continue;
      }
      if (is_on(obj_a, obj_b)) {
        state.predicates.emplace_back("(ON " + name_a + " " + name_b + ")");
      }
    }
  }

  if (!holding_any) {
    state.predicates.emplace_back("(HANDEMPTY)");
  }

  return state;
}

bool PredicatesGrounder::is_on_table(const TrackedObject & obj)
{
  const double height = obj.pose.position.z;
  const auto table_coords = planning_scene_interface_.getObjectPoses({table_base_name_});
  if (table_coords.empty()) {
    throw std::runtime_error("table_base not found: " + table_base_name_);
  }
  const auto & table_pose = table_coords.at(table_base_name_);
  const double table_height = table_pose.position.z + table_height_;
  const double center_height = table_height + (cube_height_ / 2.0);
  const double diff = std::abs(height - center_height);
  if (debug_logging_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[grounder] is_on_table name=%s z=%.4f table=%.4f diff=%.4f threshold=%.4f",
      obj.name.c_str(), height, center_height, diff, on_tolerance_);
  }
  return diff <= on_tolerance_;
}

bool PredicatesGrounder::is_on(const TrackedObject & top, const TrackedObject & bottom) const
{
  if (&top == &bottom) {
    return false;
  }
  if (top.attached) {
    return false;
  }

  const auto & p = top.pose.position;
  const auto & q = bottom.pose.position;

  double dz = p.z - q.z;
  double dx = p.x - q.x;
  double dy = p.y - q.y;

  double lateral = std::hypot(dx, dy);
  const bool result =
    std::abs(dz - cube_height_) <= on_tolerance_ &&
    lateral < lateral_threshold_;
  if (debug_logging_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[grounder] is_on top=%s bottom=%s dz=%.4f lateral=%.4f -> %s",
      top.name.c_str(), bottom.name.c_str(), dz, lateral, result ? "true" : "false");
  }
  return result;
}

bool PredicatesGrounder::is_clear(const TrackedObject & obj) const
{
  const auto & objects = registry_.objects();
  for (const auto & [name, other] : objects) {
    if (name == obj.name) {
      continue;
    }
    if (is_on(other, obj)) {
      return false;
    }
  }

  return !obj.attached;
}

bool PredicatesGrounder::is_holding(const TrackedObject & obj) const
{
  return obj.attached;
}

bool PredicatesGrounder::adjustObject(const std::string& object_id, const std::string& predicate)
{
  // Parse predicate: "(ONTABLE cube_1)" or "(ON cube_1 cube_2)"
  // Remove parentheses and split
  std::string pred = predicate;
  // Remove leading/trailing whitespace and parentheses
  while (!pred.empty() && (pred.front() == '(' || pred.front() == ' ')) {
    pred.erase(0, 1);
  }
  while (!pred.empty() && (pred.back() == ')' || pred.back() == ' ')) {
    pred.pop_back();
  }

  // Split into tokens
  std::vector<std::string> tokens;
  std::istringstream iss(pred);
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }

  if (tokens.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[grounder] Empty predicate for object '%s'", object_id.c_str());
    return false;
  }

  std::string pred_type = tokens[0];
  geometry_msgs::msg::Pose ideal_pose;

  // Get current object pose
  const auto current_poses = planning_scene_interface_.getObjectPoses({object_id});
  if (current_poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[grounder] Object '%s' not found in planning scene", object_id.c_str());
    return false;
  }
  const auto& current_pose = current_poses.at(object_id);
  ideal_pose = current_pose;  // Start with current pose

  // Handle different predicate types
  if (pred_type == "ONTABLE") {
    // Only adjust z to table_height + cube_height/2 + clearance (keep x, y unchanged)
    const auto table_coords = planning_scene_interface_.getObjectPoses({table_base_name_});
    if (table_coords.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "[grounder] Table '%s' not found", table_base_name_.c_str());
      return false;
    }
    const auto& table_pose = table_coords.at(table_base_name_);
    const double table_height = table_pose.position.z + table_height_;
    ideal_pose.position.z = table_height + (cube_height_ / 2.0) + placement_clearance_;
    // x and y remain unchanged from current_pose
    
    if (debug_logging_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[grounder] adjustObject: ONTABLE %s -> z=%.4f (x=%.4f, y=%.4f unchanged, table_height=%.4f + cube_height/2=%.4f + clearance=%.4f)",
        object_id.c_str(), ideal_pose.position.z, ideal_pose.position.x, ideal_pose.position.y,
        table_height, cube_height_ / 2.0, placement_clearance_);
    }
  } else if (pred_type == "ON") {
    // Format: "ON cube_1 cube_2" - adjust cube_1 to be on top of cube_2
    if (tokens.size() < 3) {
      RCLCPP_ERROR(node_->get_logger(), "[grounder] Invalid ON predicate: '%s' (expected: ON top bottom)", predicate.c_str());
      return false;
    }
    
    std::string top_obj = tokens[1];
    std::string bottom_obj = tokens[2];
    
    if (top_obj != object_id) {
      // This predicate doesn't involve the requested object
      return false;
    }

    // Get bottom object's pose
    const auto bottom_poses = planning_scene_interface_.getObjectPoses({bottom_obj});
    if (bottom_poses.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "[grounder] Bottom object '%s' not found for ON predicate", bottom_obj.c_str());
      return false;
    }
    const auto& bottom_pose = bottom_poses.at(bottom_obj);

    // Only adjust z position: z = bottom.z + cube_height_ + clearance (keep x, y unchanged)
    ideal_pose.position.z = bottom_pose.position.z + cube_height_ + placement_clearance_;
    // x and y remain unchanged from current_pose
    
    // Orientation remains unchanged from current_pose

    if (debug_logging_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[grounder] adjustObject: ON %s %s -> z=%.4f (x=%.4f, y=%.4f unchanged, bottom.z=%.4f + cube_height=%.4f + clearance=%.4f)",
        top_obj.c_str(), bottom_obj.c_str(),
        ideal_pose.position.z, ideal_pose.position.x, ideal_pose.position.y,
        bottom_pose.position.z, cube_height_, placement_clearance_);
    }
  } else if (pred_type == "HOLDING" || pred_type == "CLEAR" || pred_type == "HANDEMPTY") {
    // These predicates don't require position adjustment
    if (debug_logging_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "[grounder] adjustObject: Predicate '%s' does not require position adjustment for '%s'",
        pred_type.c_str(), object_id.c_str());
    }
    return true;  // No adjustment needed, but not an error
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "[grounder] adjustObject: Predicate type '%s' not implemented for object '%s'",
      pred_type.c_str(), object_id.c_str());
    return false;
  }

  // Apply the position adjustment
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = planning_frame_;
  collision_object.header.stamp = node_->get_clock()->now();
  collision_object.id = object_id;
  collision_object.operation = collision_object.ADD;  // ADD also works to update existing objects

  // Create a box primitive (assuming cubes are boxes)
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = cube_height_;  // width
  box.dimensions[1] = cube_height_;  // depth
  box.dimensions[2] = cube_height_;  // height
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(ideal_pose);

  // Apply the update to the planning scene
  bool success = planning_scene_interface_.applyCollisionObject(collision_object);
  
  if (debug_logging_) {
    if (success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[grounder] Successfully adjusted position of '%s' to (%.4f, %.4f, %.4f) based on '%s'",
        object_id.c_str(),
        ideal_pose.position.x, ideal_pose.position.y, ideal_pose.position.z,
        predicate.c_str());
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[grounder] Failed to adjust position of '%s' based on '%s'",
        object_id.c_str(), predicate.c_str());
    }
  }

  return success;
}

void PredicatesGrounder::adjustObjects()
{
  // Refresh object registry from MoveIt
  refresh_from_moveit();
  
  // Compute current grounded state
  GroundedState state = compute_state();
  
  // For each object, find all predicates involving it and adjust
  const auto& objects = registry_.objects();
  
  for (const auto& [object_id, obj] : objects) {
    // Skip ignored objects
    if (std::find(ignored_objects_.begin(), ignored_objects_.end(), object_id) != ignored_objects_.end()) {
      continue;
    }
    
    // Find all predicates involving this object
    for (const auto& predicate : state.predicates) {
      // Check if predicate involves this object
      if (predicate.find(object_id) != std::string::npos) {
        adjustObject(object_id, predicate);
      }
    }
  }
  
  if (debug_logging_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[grounder] adjustObjects: Completed adjustment for all objects");
  }
}

}  // namespace grounding


