#include <rclcpp/rclcpp.hpp>

#include "px4_mpc/mpc_runner.hpp"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Starting odometry listener..." << std::endl;
  auto n = rclcpp::Node::make_shared("mpc_runner");
  MpcRunner mpc_runner(n);
  
  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}
