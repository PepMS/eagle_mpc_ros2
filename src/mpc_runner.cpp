#include "px4_mpc/mpc_runner.hpp"

using namespace std::chrono_literals;

MpcRunner::MpcRunner(rclcpp::Node::SharedPtr node) : node_(node)
{
  odometry_subs_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>("VehicleOdometry_PubSubTopic", std::bind(&MpcRunner::odometryCallback, this, std::placeholders::_1));
  ctrl_mode_subs_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>("VehicleControlMode_PubSubTopic", std::bind(&MpcRunner::vehicleCtrlModeCallback, this, std::placeholders::_1));

  actuator_outputs_pub_ = node->create_publisher<px4_msgs::msg::ActuatorOutputs>("ActuatorOutputs_PubSubTopic");
  offboard_mode_pub_ = node->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic");

  actuator_outputs_timer_ = node_->create_wall_timer(1ms, std::bind(&MpcRunner::pubActuatorOutputsTimerCallback, this));

  // Variable initialization
  offboard_mode_enabled_ = false;

  offboard_mode_msg_.ignore_thrust = true;
  offboard_mode_msg_.ignore_attitude = true;
  offboard_mode_msg_.ignore_bodyrate_x = true;
  offboard_mode_msg_.ignore_bodyrate_y = true;
  offboard_mode_msg_.ignore_bodyrate_z = true;
  offboard_mode_msg_.ignore_position = true;
  offboard_mode_msg_.ignore_velocity = true;
  offboard_mode_msg_.ignore_acceleration_force = true;
  // offboard_mode_msg_.ignore_alt_hold = true;
  offboard_mode_msg_.ignore_mixed_outputs = true;

  state_ = Eigen::VectorXd::Zero(13);
}

void MpcRunner::odometryCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {

}

void MpcRunner::vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  offboard_mode_enabled_ = msg->flag_control_offboard_enabled;
  if (offboard_mode_enabled_)
      std::cout << "Offboard mode enabled" << std::endl;
  
}

void MpcRunner::pubActuatorOutputsTimerCallback()
{
  offboard_mode_msg_.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

  offboard_mode_pub_->publish(offboard_mode_msg_);
  if (offboard_mode_enabled_)
  {
    actuator_outputs_msg_.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    actuator_outputs_msg_.noutputs = 4;
    actuator_outputs_msg_.output[0] = 900;
    actuator_outputs_msg_.output[1] = 1100;
    actuator_outputs_msg_.output[2] = 1100;
    actuator_outputs_msg_.output[3] = 1200;
    actuator_outputs_pub_->publish(actuator_outputs_msg_);
  }
}
