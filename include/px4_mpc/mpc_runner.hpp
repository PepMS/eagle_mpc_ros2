#ifndef PX4_MPC_HPP
#define PX4_MPC_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <Eigen/Dense>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/timesync.hpp>

#include <px4_ros_com/frame_transforms.h>

// #include "yaml_parser/parser_yaml.hpp"
// #include "yaml_parser/params_server.hpp"
#include "multicopter_mpc/mpc-main.hpp"


class MpcRunner
{
public:

explicit MpcRunner(rclcpp::Node::SharedPtr node);

private:
  // Node handler
  rclcpp::Node::SharedPtr node_;
  // State subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_subs_;  
  // Other subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr ctrl_mode_subs_;
  
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  
  // State subscribers callbacks
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
  void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg);
  void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg);
  
  void vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);

  // Publishers timer callback
  void pubActuatorOutputsTimerCallback();
  
  // Timer publishers
  rclcpp::TimerBase::SharedPtr actuator_outputs_timer_;

  // MPC related
  multicopter_mpc::MpcMain mpc_main_;
  // multicopter_mpc::ProblemMission pmission_;

  // Class variables
  Eigen::VectorXd state_;
  Eigen::Vector3d vel_ned_;
  Eigen::Vector3d vel_frd_;
  Eigen::Quaterniond q_ned_frd_;
  Eigen::Quaterniond q_nwu_flu_;
  bool offboard_mode_enabled_;
  px4_msgs::msg::OffboardControlMode offboard_mode_msg_;
  px4_msgs::msg::ActuatorOutputs actuator_outputs_msg_;
  px4_msgs::msg::PositionSetpointTriplet position_setpoint_triplet_msg_;

  // Transformation tools
  const Eigen::Quaterniond FRD_FLU_Q = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);
  const Eigen::Quaterniond NWU_NED_Q = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

};

#endif

