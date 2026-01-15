#include <algorithm>
#include <cctype>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <interfaces/srv/pick_up.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <interfaces/srv/place.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include "pick_place_generator/cube.hpp"
#include "pick_place_generator/table.hpp"
#include "pick_place_generator/place_utils.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include "grasping_moveit/mtc_actions.hpp"
#include "grasping_moveit/task_parameters.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_action_server");

namespace mtc = moveit::task_constructor;

class MTCActionServer : public rclcpp::Node
{
public:
  MTCActionServer()
    : Node("mtc_action_server"),
      tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
      tf_listener_(tf_buffer_)
  {
    cube_size_ = this->declare_parameter<double>("cube_size");
    planning_group_ = this->declare_parameter<std::string>("planning_group");
    eef_group_ = this->declare_parameter<std::string>("eef_group");
    hand_frame_ = this->declare_parameter<std::string>("hand_frame");
    attach_link_ = this->declare_parameter<std::string>("attach_link");
    base_frame_ = this->declare_parameter<std::string>("base_frame");
    
    approach_object_min_dist_ = this->declare_parameter<double>("approach_object_min_dist");
    approach_object_max_dist_ = this->declare_parameter<double>("approach_object_max_dist");
    lift_object_min_dist_ = this->declare_parameter<double>("lift_object_min_dist");
    lift_object_max_dist_ = this->declare_parameter<double>("lift_object_max_dist");
    
    gripper_open_position_ = this->declare_parameter<double>("gripper_open_position");
    gripper_close_position_ = this->declare_parameter<double>("gripper_close_position");
    gripper_max_effort_ = this->declare_parameter<double>("gripper_max_effort");
    gripper_wait_time_ = this->declare_parameter<double>("gripper_wait_time");
    
    cartesian_max_velocity_scaling_ = this->declare_parameter<double>("cartesian_max_velocity_scaling");
    cartesian_max_acceleration_scaling_ = this->declare_parameter<double>("cartesian_max_acceleration_scaling");
    cartesian_step_size_ = this->declare_parameter<double>("cartesian_step_size");
    cartesian_min_fraction_ = this->declare_parameter<double>("cartesian_min_fraction");
    cartesian_jump_threshold_ = this->declare_parameter<double>("cartesian_jump_threshold"); 
    planner_id_ = this->declare_parameter<std::string>("planner_id");
    
    grasp_velocity_ = this->declare_parameter<double>("grasp_velocity");
    grasp_acceleration_ = this->declare_parameter<double>("grasp_acceleration");
    grasp_eef_step_ = this->declare_parameter<double>("grasp_eef_step");
    retract_velocity_ = this->declare_parameter<double>("retract_velocity");
    retract_acceleration_ = this->declare_parameter<double>("retract_acceleration");
    
    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gripper_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "gripper_controller/gripper_cmd",
      gripper_callback_group_);

    pick_up_service_ = this->create_service<interfaces::srv::PickUp>(
      "/pick_up",
      std::bind(&MTCActionServer::handle_pick_request, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    place_service_ = this->create_service<interfaces::srv::Place>(
      "/place",
      std::bind(&MTCActionServer::handle_place_request, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    RCLCPP_INFO(LOGGER, "MTC Action Server initialized");
  }

  void initializeRobotModel()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      RCLCPP_FATAL(LOGGER, "Failed to load robot model");
      throw std::runtime_error("Failed to load robot model");
    }
    RCLCPP_INFO(LOGGER, "Robot model loaded successfully");
  }

  grasping_moveit::MTCActions& getMTCActions()
  {
    if (!mtc_actions_) {
      auto params = GetTaskParameters();
      mtc_actions_ = std::make_unique<grasping_moveit::MTCActions>(
        params, shared_from_this(), gripper_action_client_);
    }
    return *mtc_actions_;
  }

private:
  grasping_moveit::TaskParameters GetTaskParameters()
  {
    grasping_moveit::TaskParameters params;
    params.planning_group = planning_group_;
    params.eef_group = eef_group_;
    params.hand_frame = hand_frame_;
    params.attach_link = attach_link_;
    params.base_frame = base_frame_;
    params.approach_object_min_dist = approach_object_min_dist_;
    params.approach_object_max_dist = approach_object_max_dist_;
    params.lift_object_min_dist = lift_object_min_dist_;
    params.lift_object_max_dist = lift_object_max_dist_;
    params.cartesian_max_velocity_scaling = cartesian_max_velocity_scaling_;
    params.cartesian_max_acceleration_scaling = cartesian_max_acceleration_scaling_;
    params.cartesian_step_size = cartesian_step_size_;
    params.cartesian_min_fraction = cartesian_min_fraction_;
    params.cartesian_jump_threshold = cartesian_jump_threshold_;
    params.planner_id = planner_id_;
    params.grasp_velocity = grasp_velocity_;
    params.grasp_acceleration = grasp_acceleration_;
    params.grasp_eef_step = grasp_eef_step_;
    params.retract_velocity = retract_velocity_;
    params.retract_acceleration = retract_acceleration_;
    
    std::string package_share = ament_index_cpp::get_package_share_directory("grasping_moveit");
    std::filesystem::path config_path = std::filesystem::path(package_share) / "config" / "home_pose.yaml";
    
    try {
      YAML::Node config = YAML::LoadFile(config_path.string());
      YAML::Node home_pose = config["home_pose"];
      
      if (!home_pose) {
        throw std::runtime_error("'home_pose' section not found in " + config_path.string());
      }
      
      double home_pose_x = home_pose["position"]["x"].as<double>();
      double home_pose_y = home_pose["position"]["y"].as<double>();
      double home_pose_z = home_pose["position"]["z"].as<double>();
      double home_pose_roll = home_pose["orientation"]["roll"].as<double>();
      double home_pose_pitch = home_pose["orientation"]["pitch"].as<double>();
      double home_pose_yaw = home_pose["orientation"]["yaw"].as<double>();
      std::string home_pose_frame_id = home_pose["frame_id"].as<std::string>();
      
      params.home_pose.header.frame_id = home_pose_frame_id;
      params.home_pose.header.stamp = this->now();
      params.home_pose.pose.position.x = home_pose_x;
      params.home_pose.pose.position.y = home_pose_y;
      params.home_pose.pose.position.z = home_pose_z;
      
      Eigen::AngleAxisd roll_home(home_pose_roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitch_home(home_pose_pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yaw_home(home_pose_yaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond home_quat = yaw_home * pitch_home * roll_home;
      params.home_pose.pose.orientation.x = home_quat.x();
      params.home_pose.pose.orientation.y = home_quat.y();
      params.home_pose.pose.orientation.z = home_quat.z();
      params.home_pose.pose.orientation.w = home_quat.w();

      params.home_pose = convertPoseToToolFrame(params.home_pose);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(LOGGER, "Failed to load home_pose.yaml: %s", e.what());
      throw;
    }
    
    return params;
  }

  void handle_pick_request(
    const std::shared_ptr<interfaces::srv::PickUp::Request> request,
    std::shared_ptr<interfaces::srv::PickUp::Response> response)
  {
    RCLCPP_INFO(LOGGER, "Received pick request for object '%s'", request->object_id.c_str());
    
    // Check if object exists in planning scene
    moveit::planning_interface::PlanningSceneInterface psi;
    auto objects = psi.getObjects({request->object_id});
    if (objects.find(request->object_id) == objects.end()) {
      response->success = false;
      response->message = "Object '" + request->object_id + "' not found in planning scene";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    std::string error;
    
    // Check if object name contains "cube" (case-insensitive)
    std::string object_id_lower = request->object_id;
    std::transform(object_id_lower.begin(), object_id_lower.end(), object_id_lower.begin(), ::tolower);

    std::unique_ptr<pick_place_generator::Object> objectA;
    
    if (object_id_lower.find("cube") == std::string::npos) {
      response->success = false;
      response->message = "Object type not implemented: object name must contain 'cube'";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    } else {
      objectA = std::move(std::make_unique<pick_place_generator::Cube>(request->object_id, cube_size_));
    }
    
    // Generate grasp candidate using Cube class
    auto grasp_candidates = objectA->sampleGrasp(
      base_frame_, approach_object_min_dist_, tf_buffer_, get_logger());
    if (grasp_candidates.empty()) {
      response->success = false;
      response->message = "Failed to generate grasp candidates";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }
    auto candidate = grasp_candidates[0];
    candidate.grasp_pose = convertPoseToToolFrame(candidate.grasp_pose);
    candidate.pre_grasp_pose = convertPoseToToolFrame(candidate.pre_grasp_pose);
    
    RCLCPP_INFO(LOGGER, "Pre-grasp: (%.4f, %.4f, %.4f)", candidate.pre_grasp_pose.pose.position.x,
                candidate.pre_grasp_pose.pose.position.y, candidate.pre_grasp_pose.pose.position.z);
    RCLCPP_INFO(LOGGER, "Grasp pose: (%.4f, %.4f, %.4f)", candidate.grasp_pose.pose.position.x,
                candidate.grasp_pose.pose.position.y, candidate.grasp_pose.pose.position.z);

    // Open gripper before grasping
    RCLCPP_INFO(LOGGER, "Opening gripper");
    if (!getMTCActions().commandGripper(gripper_open_position_, gripper_max_effort_, error)) {
      response->success = false;
      response->message = "Failed to open gripper: " + error;
      return;
    }

    mtc::Task grasp_task;
    try {
      grasp_task = getMTCActions().createGraspTask(
        request->object_id, candidate);
      grasp_task.init();
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to create grasp task: ") + e.what();
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "Planning grasp task...");
    if (!grasp_task.plan(5)) {
      response->success = false;
      response->message = "Grasp task planning failed";
      grasp_task.printState();
      RCLCPP_ERROR_STREAM(LOGGER, "Failure detail:\n" << grasp_task.explainFailure());
      return;
    }

    RCLCPP_INFO(LOGGER, "Executing grasp task...");
    auto grasp_result = grasp_task.execute(*grasp_task.solutions().front());
    if (grasp_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      response->success = false;
      response->message = "Grasp execution failed";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    // Close gripper after grasping
    RCLCPP_INFO(LOGGER, "Closing gripper");
    if (!getMTCActions().commandGripper(gripper_close_position_, gripper_max_effort_, error)) {
      response->success = false;
      response->message = "Failed to close gripper: " + error;
      return;
    }

    if (request->support_surface.empty()) {
      RCLCPP_ERROR(LOGGER, "Support surface not provided in request");
      response->success = false;
      response->message = "Support surface not provided in request. Check your .srv file/ service call";
      return;
    }
    std::string support_surface = request->support_surface;
    RCLCPP_INFO(LOGGER, "Using support surface '%s' for object '%s' (from action arguments)", 
                support_surface.c_str(), request->object_id.c_str());
    
    mtc::Task retract_task;
    try {
      retract_task = getMTCActions().createRetractGraspTask(
        request->object_id, support_surface);
      retract_task.init();
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to create retract task: ") + e.what();
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "Planning retract task...");
    if (!retract_task.plan(5)) {
      response->success = false;
      response->message = "Retract task planning failed";
      retract_task.printState();
      RCLCPP_ERROR_STREAM(LOGGER, "Failure detail:\n" << retract_task.explainFailure());
      return;
    }

    RCLCPP_INFO(LOGGER, "Executing retract task...");
    auto retract_result = retract_task.execute(*retract_task.solutions().front());
    if (retract_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      response->success = false;
      response->message = "Retract execution failed";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Pick-up task completed successfully";
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());
  }
  
  void handle_place_request(
    const std::shared_ptr<interfaces::srv::Place::Request> request,
    std::shared_ptr<interfaces::srv::Place::Response> response)
  {
    std::string error;
    
    moveit::planning_interface::PlanningSceneInterface psi;
    auto attached = psi.getAttachedObjects({request->object_id});
    if (attached.find(request->object_id) == attached.end()) {
      response->success = false;
      response->message = "Object '" + request->object_id + "' is not attached to the robot";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }
    if (attached[request->object_id].link_name != hand_frame_) {
      response->success = false;
      response->message = "Attached object '" + request->object_id + "' is not held by '" + hand_frame_ + "'";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }
    
    // Check if object names contain "cube" (case-insensitive)
    std::string object_id_lower = request->object_id;
    std::string support_surface_lower = request->support_surface;
    std::transform(object_id_lower.begin(), object_id_lower.end(), object_id_lower.begin(), ::tolower);
    std::transform(support_surface_lower.begin(), support_surface_lower.end(), support_surface_lower.begin(), ::tolower);
    
    std::unique_ptr<pick_place_generator::Object> objectA;
    std::unique_ptr<pick_place_generator::Surface> objectB;
    
    // Create objectA (object to be placed) - must be a Cube for now
    if (object_id_lower.find("cube") == std::string::npos) {
      response->success = false;
      response->message = "Place object type not implemented: place object name must contain 'cube'";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    } else {
      objectA = std::move(std::make_unique<pick_place_generator::Cube>(request->object_id, cube_size_));
    }
    
    if (support_surface_lower.find("table") != std::string::npos) {
      objectB = std::move(std::make_unique<pick_place_generator::Table>(request->support_surface));
    } else if (support_surface_lower.find("cube") != std::string::npos) {
      objectB = std::move(std::make_unique<pick_place_generator::Cube>(request->support_surface, cube_size_));
    } else {
      response->success = false;
      response->message = "Support surface type not implemented: must be 'cube' or 'table'";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }
    
    // Get place coordinates on objectB
    auto place_candidates = pick_place_generator::getPlaceCandidates(
      *objectA, *objectB, base_frame_, approach_object_min_dist_, 0.005, tf_buffer_, get_logger());
    if (place_candidates.empty()) {
      response->success = false;
      response->message = "Failed to generate place candidates";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }
    
    // Find first collision-free candidate (only check collisions for table placements)
    pick_place_generator::GraspCandidate place_candidate;
    bool found_valid = false;
    bool is_table_placement = (support_surface_lower.find("table") != std::string::npos);
    
    if (is_table_placement) {
      const int max_candidates_to_check = std::min(500, static_cast<int>(place_candidates.size()));
      
      // Creating temp planning scene to check collisions
      planning_scene::PlanningScene planning_scene(robot_model_);
      
      // Populating planning scene with current objects
      auto objects = psi.getObjects();
      for (const auto& [name, obj] : objects) {
        planning_scene.processCollisionObjectMsg(obj);
      }
      
      // Get joint model group and IK solver
      const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group_);
      if (!jmg) {
        response->success = false;
        response->message = "Invalid joint model group: " + planning_group_;
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        return;
      }
      
      // Get current state as seed for IK
      moveit::core::RobotState current_state = planning_scene.getCurrentState();
      
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      collision_request.group_name = planning_group_;
      
      RCLCPP_INFO(LOGGER, "Checking %d place candidates for collision-free poses (table placement)...", max_candidates_to_check);
      
      for (int i = 0; i < max_candidates_to_check; ++i) {
        auto& candidate = place_candidates[i];
        RCLCPP_INFO(LOGGER, "Checking candidate %d: grasp pose (%.4f, %.4f, %.4f), pre-grasp pose (%.4f, %.4f, %.4f)", 
                    i, candidate.grasp_pose.pose.position.x, candidate.grasp_pose.pose.position.y, candidate.grasp_pose.pose.position.z,
                    candidate.pre_grasp_pose.pose.position.x, candidate.pre_grasp_pose.pose.position.y, candidate.pre_grasp_pose.pose.position.z);
        
        candidate.grasp_pose = convertPoseToToolFrame(candidate.grasp_pose);
        candidate.pre_grasp_pose = convertPoseToToolFrame(candidate.pre_grasp_pose);
        
        moveit::core::RobotState grasp_state = current_state;  // Use current state as seed for IK
        bool ik_success = grasp_state.setFromIK(jmg, candidate.grasp_pose.pose, hand_frame_, 0.1);
        if (!ik_success || !grasp_state.satisfiesBounds(jmg)) {
          continue;
        }
        
        collision_result.clear();
        planning_scene.checkCollision(collision_request, collision_result, grasp_state);
        if (collision_result.collision) {
          continue;
        }
        
        // Check validity of pre-grasp pose as well
        moveit::core::RobotState pre_grasp_state = current_state;
        ik_success = pre_grasp_state.setFromIK(jmg, candidate.pre_grasp_pose.pose, hand_frame_, 0.1);
        if (!ik_success || !pre_grasp_state.satisfiesBounds(jmg)) {
          continue;
        }
        
        collision_result.clear();
        planning_scene.checkCollision(collision_request, collision_result, pre_grasp_state);
        if (collision_result.collision) {
          continue;
        }
        
        // Both poses are valid, so we can use this candidate
        place_candidate = candidate;
        found_valid = true;
        RCLCPP_INFO(LOGGER, "Found collision-free candidate at index %d", i);
        break;
      }
      
      if (!found_valid) {
        response->success = false;
        response->message = "No collision-free place candidate found after checking " + 
                            std::to_string(max_candidates_to_check) + " candidates";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        return;
      }
    } else {
      // For cube-on-cube placements, use first candidate without collision checking
      RCLCPP_INFO(LOGGER, "Skipping collision check for cube-on-cube placement, using first candidate");
      place_candidate = place_candidates[0];
      place_candidate.grasp_pose = convertPoseToToolFrame(place_candidate.grasp_pose);
      place_candidate.pre_grasp_pose = convertPoseToToolFrame(place_candidate.pre_grasp_pose);
      found_valid = true;
    }

    mtc::Task place_task;
    try {
      place_task = getMTCActions().createPlaceTask(
        request->object_id, place_candidate);
      place_task.init();
    } catch (const mtc::InitStageException& e) {
      response->success = false;
      response->message = std::string("Failed to create place task: ") + e.what();
      
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to create place task: ") + e.what();
      
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "Planning place task...");
    if (!place_task.plan(5)) {
      response->success = false;
      response->message = "Place task planning failed";
      place_task.printState();
      RCLCPP_ERROR_STREAM(LOGGER, "Failure detail:\n" << place_task.explainFailure());
      return;
    }

    RCLCPP_INFO(LOGGER, "Executing place task...");
    auto place_result = place_task.execute(*place_task.solutions().front());
    if (place_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      response->success = false;
      response->message = "Place task execution failed";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    // Open gripper to release the object after reaching place location
    RCLCPP_INFO(LOGGER, "Opening gripper to release object");
    if (!getMTCActions().commandGripper(gripper_open_position_, gripper_max_effort_, error)) {
      response->success = false;
      response->message = "Failed to open gripper: " + error;
      return;
    }

    mtc::Task retract_place_task;
    try {
      retract_place_task = getMTCActions().createRetractPlaceTask(request->object_id);
      retract_place_task.init();
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to create retract place task: ") + e.what();
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(LOGGER, "Planning retract place task...");
    if (!retract_place_task.plan(5)) {
      response->success = false;
      response->message = "Retract place planning failed";
      retract_place_task.printState();
      RCLCPP_ERROR_STREAM(LOGGER, "Failure detail:\n" << retract_place_task.explainFailure());
      return;
    }

    RCLCPP_INFO(LOGGER, "Executing retract place task...");
    auto retract_result = retract_place_task.execute(*retract_place_task.solutions().front());
    if (retract_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      response->success = false;
      response->message = "Retract place execution failed";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "Place completed successfully";
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());
  }

  rclcpp::Service<interfaces::srv::PickUp>::SharedPtr pick_up_service_;
  rclcpp::Service<interfaces::srv::Place>::SharedPtr place_service_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_callback_group_;
  
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  
  std::unique_ptr<grasping_moveit::MTCActions> mtc_actions_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  double cube_size_;
  std::string planning_group_;
  std::string eef_group_;
  std::string hand_frame_;
  std::string attach_link_;
  std::string base_frame_;
  
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;
  double gripper_open_position_;
  double gripper_close_position_;
  double gripper_max_effort_;
  double gripper_wait_time_;
  
  double cartesian_max_velocity_scaling_;
  double cartesian_max_acceleration_scaling_;
  double cartesian_step_size_;
  double cartesian_min_fraction_;
  double cartesian_jump_threshold_;
  
  std::string planner_id_;
  
  double grasp_velocity_;
  double grasp_acceleration_;
  double grasp_eef_step_;
  double retract_velocity_;
  double retract_acceleration_;
  
  geometry_msgs::msg::PoseStamped convertPoseToToolFrame(
    const geometry_msgs::msg::PoseStamped& ee_pose)
  {
    geometry_msgs::msg::PoseStamped tool_pose = ee_pose;
    
    try {
      auto ee_to_tool = tf_buffer_.lookupTransform("ee_link", "ur10_tool0", 
                                                    tf2::TimePointZero, 
                                                    tf2::durationFromSec(1.0));
      
      double offset_x = ee_to_tool.transform.translation.x;
      double offset_y = ee_to_tool.transform.translation.y;
      double offset_z = ee_to_tool.transform.translation.z;
      
      RCLCPP_DEBUG(get_logger(), "Tool to EE offset: (%.4f, %.4f, %.4f)", offset_x, offset_y, offset_z);
      
      tool_pose.pose.position.x -= offset_x;
      tool_pose.pose.position.y -= offset_y;
      tool_pose.pose.position.z -= offset_z;
      
      RCLCPP_DEBUG(get_logger(), "Converted pose from ee_link to tool0: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
                  ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z,
                  tool_pose.pose.position.x, tool_pose.pose.position.y, tool_pose.pose.position.z);
      
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "Could not get transform from ee_link to tool0: %s", ex.what());
    }
    
    return tool_pose;
  }
  
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<MTCActionServer>();
  // Initialize robot model loader after node is in shared_ptr
  node->initializeRobotModel();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
