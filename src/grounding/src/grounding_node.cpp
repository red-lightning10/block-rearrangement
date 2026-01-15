#include "grounding/predicates_grounder.hpp"

#include <interfaces/srv/get_grounding.hpp>
#include <interfaces/srv/adjust_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

using GroundStateSrv = interfaces::srv::GetGrounding;
using AdjustObjectsSrv = interfaces::srv::AdjustObjects;

class GroundingNode : public rclcpp::Node
{
public:
  GroundingNode()
    : Node("grounding_node", rclcpp::NodeOptions().allow_undeclared_parameters(true)),
      grounder_(std::make_shared<grounding::PredicatesGrounder>(this))
  {
    grounding_service_ = this->create_service<GroundStateSrv>(
      "/grounding/get_state",
      std::bind(&GroundingNode::handle_grounding_request, this, std::placeholders::_1, std::placeholders::_2));

    adjust_service_ = this->create_service<AdjustObjectsSrv>(
      "/grounding/adjust_objects",
      std::bind(&GroundingNode::handle_adjust_objects_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_grounding_request(
    const GroundStateSrv::Request::SharedPtr /*request*/,
    GroundStateSrv::Response::SharedPtr response)
  {
    grounder_->refresh_from_moveit();
    auto state = grounder_->compute_state();
    response->grounded_predicates = state.predicates;
    response->success = true;
    response->message = "grounded";
  }

  void handle_adjust_objects_request(
    const AdjustObjectsSrv::Request::SharedPtr /*request*/,
    AdjustObjectsSrv::Response::SharedPtr response)
  {
    try {
      grounder_->adjustObjects();
      response->success = true;
      response->message = "Objects adjusted successfully";
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to adjust objects: ") + e.what();
    }
  }

  std::shared_ptr<grounding::PredicatesGrounder> grounder_;
  rclcpp::Service<GroundStateSrv>::SharedPtr grounding_service_;
  rclcpp::Service<AdjustObjectsSrv>::SharedPtr adjust_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
