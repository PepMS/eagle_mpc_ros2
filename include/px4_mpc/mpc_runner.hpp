#ifndef PX4_MPC_MPC_RUNNER_HPP
#define PX4_MPC_MPC_RUNNER_HPP

#include "px4_mpc/controller_base.hpp"

class MpcRunner : public ControllerAbstract
{
public:
  explicit MpcRunner(rclcpp::Node::SharedPtr node);
  virtual ~MpcRunner();

private:
  // MPC related
  // multicopter_mpc::MpcMain mpc_main_;
};

#endif
