#include <rclcpp/rclcpp.hpp>
#include <interfaces/srv/clear_objects.hpp>
#include <chrono>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("clear_objects_client");
  auto client = node->create_client<interfaces::srv::ClearObjects>("/clear_objects");
  
  if (!client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "Service /clear_objects not available");
    return 1;
  }
  
  auto request = std::make_shared<interfaces::srv::ClearObjects::Request>();
  auto future = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call clear_objects service");
    return 1;
  }
  
  auto response = future.get();
  if (response->success)
  {
    RCLCPP_INFO(node->get_logger(), "Successfully cleared %d objects", response->cleared_count);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Clear objects failed: %s", response->message.c_str());
  }
  
  rclcpp::shutdown();
  return 0;
}


