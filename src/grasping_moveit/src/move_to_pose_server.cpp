#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/srv/move_to_pose.hpp>
#include "grasping_moveit/moveit_utils.hpp"

class MoveToPoseServer : public rclcpp::Node
{
public:
  MoveToPoseServer() : Node("move_to_pose_server")
  {
    tool_length_offset_ = this->declare_parameter<double>("tool_length_offset");
    planning_group_ = this->declare_parameter<std::string>("planning_group");
    velocity_scaling_ = this->declare_parameter<double>("velocity_scaling");
    acceleration_scaling_ = this->declare_parameter<double>("acceleration_scaling");
    eef_step_ = this->declare_parameter<double>("eef_step");
    
    service_ = this->create_service<interfaces::srv::MoveToPose>("/move_to_pose",
      std::bind(&MoveToPoseServer::handle_move_to_pose, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Move to pose service ready");
    RCLCPP_INFO(this->get_logger(), "Tool offset: %.3f m", tool_length_offset_);
  }

private:
  void handle_move_to_pose(
    const std::shared_ptr<interfaces::srv::MoveToPose::Request> request,
    std::shared_ptr<interfaces::srv::MoveToPose::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received move to pose request");
    
    try {

      using moveit::planning_interface::MoveGroupInterface;
      MoveGroupInterface move_group_interface(shared_from_this(), planning_group_);

      move_group_interface.setPoseReferenceFrame("ur10_base");
      geometry_msgs::msg::Pose target_pose = request->target_pose.pose;
      
      // Apply tool offset
      target_pose.position.z += tool_length_offset_;
      
      RCLCPP_INFO(this->get_logger(), 
                  "Target pose: (%.3f, %.3f, %.3f) (adjusted Z: %.3f)", 
                  request->target_pose.pose.position.x,
                  request->target_pose.pose.position.y,
                  request->target_pose.pose.position.z,
                  target_pose.position.z);
      
      // Use MoveGroupUtilities for straight-line Cartesian motion (like RTDE's moveL)
      RCLCPP_INFO(this->get_logger(), "Moving to target pose using Cartesian path");
      grasping_moveit::MoveGroupUtilities planner(move_group_interface, shared_from_this(), this->get_logger());
      bool success = planner.moveL(target_pose, velocity_scaling_, acceleration_scaling_, eef_step_);
      
      if (success) {
        response->success = true;
        response->message = "Successfully moved to target pose";
        RCLCPP_INFO(this->get_logger(), "Move to pose completed successfully");
      } 
      else {
        // Fallback to joint-space planning if Cartesian path fails
        RCLCPP_WARN(this->get_logger(), 
                   "Cartesian path failed, falling back to joint-space planning");
        
        move_group_interface.setMaxVelocityScalingFactor(velocity_scaling_);
        move_group_interface.setMaxAccelerationScalingFactor(acceleration_scaling_);
        move_group_interface.setPoseTarget(target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_success = static_cast<bool>(move_group_interface.plan(plan));
        
        if (plan_success) {
          RCLCPP_INFO(this->get_logger(), "Joint-space plan found, executing");
          auto result = move_group_interface.execute(plan);
          
          if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            response->success = true;
            response->message = "Successfully moved to target pose using joint-space planning";
            RCLCPP_INFO(this->get_logger(), "Move to pose completed successfully");
          } else {
            response->success = false;
            response->message = "Failed to execute joint-space trajectory";
            RCLCPP_ERROR(this->get_logger(), "Execution failed");
          }
        } else {
          response->success = false;
          response->message = "Both Cartesian and joint-space planning failed";
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
      }
    } 
    catch (const std::exception& e) {
      response->success = false;
      std::string error_msg = e.what();

      
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", error_msg.c_str());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  double tool_length_offset_;
  std::string planning_group_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double eef_step_;
  
  rclcpp::Service<interfaces::srv::MoveToPose>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


