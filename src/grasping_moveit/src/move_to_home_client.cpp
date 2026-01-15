#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/srv/move_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

class MoveToHomeClient : public rclcpp::Node
{
public:
  MoveToHomeClient() : Node("move_to_home_client")
  {
    client_ = this->create_client<interfaces::srv::MoveToPose>("/move_to_pose");
    RCLCPP_INFO(this->get_logger(), "Move to home client ready");
  }

  bool move_to_home()
  {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Move to pose service not available");
      return false;
    }

    std::string package_share = ament_index_cpp::get_package_share_directory("grasping_moveit");
    std::filesystem::path config_path = std::filesystem::path(package_share) / "config" / "home_pose.yaml";

    try {
        YAML::Node config = YAML::LoadFile(config_path.string());
        YAML::Node home_pose = config["home_pose"];
        
        if (!home_pose) {
          RCLCPP_FATAL(this->get_logger(), "'home_pose' section not found in %s", config_path.string().c_str());
          return false;
        }
        
        home_pose_x_ = home_pose["position"]["x"].as<double>();
        home_pose_y_ = home_pose["position"]["y"].as<double>();
        home_pose_z_ = home_pose["position"]["z"].as<double>();
        home_pose_roll_ = home_pose["orientation"]["roll"].as<double>();
        home_pose_pitch_ = home_pose["orientation"]["pitch"].as<double>();
        home_pose_yaw_ = home_pose["orientation"]["yaw"].as<double>();
        home_pose_frame_id_ = home_pose["frame_id"].as<std::string>();
        
        RCLCPP_INFO(this->get_logger(), "Loaded home pose: (%.3f, %.3f, %.3f) [%.3f, %.3f, %.3f]",
                    home_pose_x_, home_pose_y_, home_pose_z_,
                    home_pose_roll_, home_pose_pitch_, home_pose_yaw_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load home_pose.yaml: %s", e.what());
        return false;
    }

    auto request = std::make_shared<interfaces::srv::MoveToPose::Request>();
    
    request->target_pose.header.frame_id = home_pose_frame_id_;
    request->target_pose.header.stamp = this->now();
    request->target_pose.pose.position.x = home_pose_x_;
    request->target_pose.pose.position.y = home_pose_y_;
    request->target_pose.pose.position.z = home_pose_z_;
    
    tf2::Quaternion q;
    q.setRPY(home_pose_roll_, home_pose_pitch_, home_pose_yaw_);
    request->target_pose.pose.orientation = tf2::toMsg(q);
    
    RCLCPP_INFO(this->get_logger(), "Calling move to pose service with home pose");
    
    auto future = client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(60)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
      return false;
    }
    
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully moved to home: %s", response->message.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to home: %s", response->message.c_str());
      return false;
    }
  }

private:
  double home_pose_x_;
  double home_pose_y_;
  double home_pose_z_;
  double home_pose_roll_;
  double home_pose_pitch_;
  double home_pose_yaw_;
  std::string home_pose_frame_id_;
  
  rclcpp::Client<interfaces::srv::MoveToPose>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToHomeClient>();
  
  bool success = node->move_to_home();
  
  rclcpp::shutdown();
  return success ? 0 : 1;
}