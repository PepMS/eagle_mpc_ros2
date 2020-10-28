#ifndef PX4_MPC_EXAMPLE_CONTROLLER_HPP
#define PX4_MPC_EXAMPLE_CONTROLLER_HPP

#include "px4_mpc/controller_base.hpp"

class ExampleController : public ControllerAbstract
{
public:
  explicit ExampleController(rclcpp::Node::SharedPtr node);
  virtual ~ExampleController();

  void writeControls();

  rclcpp::TimerBase::SharedPtr timer_;

private:
  std::size_t counter_;
  std::size_t motor_idx_;
};

#endif
