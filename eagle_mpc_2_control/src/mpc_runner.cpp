////////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 Pep Martí Saumell
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc_2_control/mpc_runner.hpp"

using namespace std::chrono_literals;

MpcRunner::MpcRunner(const std::string& node_name) : ControllerAbstract(node_name) {
  declare_parameter<bool>("enable_controller", false);

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  param_callback_handle_ = param_subscriber_->add_parameter_callback(
      "enable_controller", std::bind(&MpcRunner::enableControllerCallback, this, std::placeholders::_1));

  vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);

  running_controller_ = false;
}

MpcRunner::~MpcRunner() {}

void MpcRunner::enableControllerCallback(const rclcpp::Parameter& p) {
  bool param = p.as_bool();
  if (param && !running_controller_) {
    enabling_procedure();
  } else if (!param && running_controller_) {
    disabling_procedure();
  }
}

void MpcRunner::enabling_procedure() {
  RCLCPP_WARN(get_logger(), "MPC Controller state: ENABLING (Safety Checks)");
  // here do all safety checks related to the MPC controller configuration
  // such as to check whether the initial state for the trajectory 
  //is the same as the current state of the robot

  // If all safety checks are correct, change flihgt mode, arm and start mission
  actuator_normalized_[0] = -1.0;
  actuator_normalized_[1] = -1.0;
  actuator_normalized_[2] = -1.0;
  actuator_normalized_[3] = 0.5;
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10); // 10 = PX4_MOTOR_CONTROL_MODE
  arm();
  RCLCPP_WARN(get_logger(), "MPC Controller state: ENABLED");
  running_controller_ = true;
}

void MpcRunner::disabling_procedure() {
  RCLCPP_WARN(get_logger(), "MPC Controller. DISABLING");
  // Change to a safe flight mode

  // If all safety checks are correct, change flihgt mode, arm and start mission
  running_controller_ = false;
  RCLCPP_WARN(get_logger(), "MPC Controller state: DISABLED");
}

void MpcRunner::arm() const {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(get_logger(), "Arm command send");
}

void MpcRunner::disarm() const {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(get_logger(), "Disarm command send");
}

void MpcRunner::publish_vehicle_command(uint16_t command, float param1, float param2) const {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = timestamp_.load();
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  vehicle_command_publisher_->publish(msg);
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MpcRunner> controller = std::make_shared<MpcRunner>("MpcRunner");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}