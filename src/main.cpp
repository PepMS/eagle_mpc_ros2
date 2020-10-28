#include <rclcpp/rclcpp.hpp>

#include "px4_mpc/controller_base.hpp"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Starting odometry listener..." << std::endl;
  auto n = rclcpp::Node::make_shared("mpc_runner");
  // MpcRunner mpc_runner(n);
  const ControllerType::Type controller_type = ControllerType::ExampleController;
  std::shared_ptr<ControllerAbstract> controller = ControllerAbstract::createController(controller_type, n);

  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}
