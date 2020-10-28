#include "px4_mpc/example_controller.hpp"

using namespace std::chrono_literals;

ExampleController::ExampleController(rclcpp::Node::SharedPtr node) : ControllerAbstract(node)
{
  actuator_normalized_[0] = -1.0;
  actuator_normalized_[1] = -1.0;
  actuator_normalized_[2] = -1.0;
  actuator_normalized_[3] = -1.0;
  RCLCPP_INFO(node_->get_logger(), "EXAMPLE CONTROLLER Created!");

  counter_ = 0;
  motor_idx_ = 0;

  timer_ = node_->create_wall_timer(2s, std::bind(&ExampleController::writeControls, this));
}

ExampleController::~ExampleController() {}

void ExampleController::writeControls()
{
  RCLCPP_INFO(node_->get_logger(), "Writing motor: %d", motor_idx_);

  actuator_normalized_[0] = -1.0;
  actuator_normalized_[1] = -1.0;
  actuator_normalized_[2] = -1.0;
  actuator_normalized_[3] = -1.0;

  actuator_normalized_[motor_idx_] = 0.0;

  motor_idx_++;
  motor_idx_ = motor_idx_ > 3 ? 0 : motor_idx_;
}
