#include "px4_mpc/mpc_runner.hpp"

using namespace std::chrono_literals;

MpcRunner::MpcRunner(rclcpp::Node::SharedPtr node) : ControllerAbstract(node)
{
  // mpc_main_ = multicopter_mpc::MpcMain(multicopter_mpc::MultiCopterTypes::Iris, multicopter_mpc::MissionTypes::Hover, multicopter_mpc::SolverTypes::BoxFDDP);
}

MpcRunner::~MpcRunner() {}
