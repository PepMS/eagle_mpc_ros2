#ifndef PX4_MPC_HPP
#define PX4_MPC_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

class MpcRunner
{
public:
explicit MpcRunner(rclcpp::Node::SharedPtr node);

private:
  // Node handler
  rclcpp::Node::SharedPtr node_;
  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr ctrl_mode_subs_;
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  
  rclcpp::TimerBase::SharedPtr actuator_outputs_timer_;
  
  // Class variables
  bool offboard_mode_enabled_;
  px4_msgs::msg::OffboardControlMode offboard_mode_msg_;
  px4_msgs::msg::ActuatorOutputs actuator_outputs_msg_;

  void odometryCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);

  void pubActuatorOutputsTimerCallback();

};

#endif

