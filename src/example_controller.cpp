#include "px4_mpc/example_controller.hpp"

using namespace std::chrono_literals;

ExampleController::ExampleController(rclcpp::Node::SharedPtr node) : ControllerAbstract(node)
{
  actuator_normalized_[0] = 0.1;
  actuator_normalized_[1] = 0.2;
  actuator_normalized_[2] = 0.3;
  actuator_normalized_[3] = 0.4;
  RCLCPP_INFO(node_->get_logger(), "EXAMPLE CONTROLLER Created!");

}

ExampleController::~ExampleController() {}


