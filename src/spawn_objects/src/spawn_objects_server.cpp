#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/subscription_options.hpp>
#include <rmw/qos_profiles.h>

#include <interfaces/msg/detection_array.hpp>
#include <interfaces/srv/clear_objects.hpp>
#include <interfaces/srv/project_to3_d.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "spawn_objects/spawn_objects.hpp"

namespace spawn_objects
{

class SpawnObjectsServer : public rclcpp::Node
{
public:
  SpawnObjectsServer()
  : Node("spawn_objects_server"),
    planning_scene_interface_(""),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    double cube_size = this->declare_parameter<double>("cube_size");
    
    subscription_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = subscription_group_;

    // Use transient local so late-joiners get the last detections immediately.
    rclcpp::QoS detections_qos{rclcpp::KeepLast(1)};
    detections_qos.transient_local().reliable();

    detections_sub_ = create_subscription<interfaces::msg::DetectionArray>(
      "/detections",
      detections_qos,
      std::bind(&SpawnObjectsServer::DetectionsCallback, this, std::placeholders::_1),
      sub_options);

    project_client_ = create_client<interfaces::srv::ProjectTo3D>(
      "/project_to_3d",
      rmw_qos_profile_services_default,
      client_group_);

    logic_ = std::make_unique<SpawnObjects>(
      planning_scene_interface_, tf_buffer_, project_client_, cube_size, get_logger());

    clear_service_ = create_service<interfaces::srv::ClearObjects>(
      "/clear_objects",
      std::bind(&SpawnObjectsServer::ClearObjectsCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Waiting for project_to_3d service...");
    project_client_->wait_for_service(std::chrono::seconds(10));
    RCLCPP_INFO(get_logger(), "Services available");

    RCLCPP_INFO(get_logger(), "Spawn Objects Server started");
    RCLCPP_INFO(get_logger(), "Subscribed to /detections topic");
    RCLCPP_INFO(get_logger(), "Clear objects service available at: /clear_objects");
    RCLCPP_INFO(get_logger(), "Waiting for detections on /detections topic");
  }

private:
  void DetectionsCallback(const interfaces::msg::DetectionArray::SharedPtr msg)
  {
    if (!msg->success) {
      RCLCPP_WARN(get_logger(), "Received detections with success=false: %s", msg->message.c_str());
      return;
    }

    if (msg->detections.empty()) {
      RCLCPP_INFO(get_logger(), "Received no detections");
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Received %zu detections from /detections topic", msg->detections.size());
    logic_->SpawnFromDetections(msg->detections);
  }

  void ClearObjectsCallback(
    const std::shared_ptr<interfaces::srv::ClearObjects::Request> request,
    std::shared_ptr<interfaces::srv::ClearObjects::Response> response)
  {
    (void)request;
    int cleared_count = logic_->ClearAll();
    response->success = cleared_count > 0;
    response->cleared_count = cleared_count;
    
    if (response->success) {
      response->message = std::to_string(cleared_count) + " objects cleared from planning scene";
    } else {
      response->message = "No objects to clear or clearing failed";
    }
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Client<interfaces::srv::ProjectTo3D>::SharedPtr project_client_;
  std::unique_ptr<SpawnObjects> logic_;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Service<interfaces::srv::ClearObjects>::SharedPtr clear_service_;
  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
};

}  // namespace spawn_objects

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<spawn_objects::SpawnObjectsServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Spawn Objects Server running. Waiting for detections on /detections topic...");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}