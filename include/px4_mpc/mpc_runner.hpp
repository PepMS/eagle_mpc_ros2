#ifndef PX4_MPC_HPP
#define PX4_MPC_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <Eigen/Dense>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position_groundtruth.hpp>
#include <px4_msgs/msg/vehicle_attitude_groundtruth.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity_groundtruth.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_direct_control.hpp>

#include <px4_ros_com/frame_transforms.h>

// #include "multicopter_mpc/mpc-main.hpp"

class MpcRunner
{
public:

explicit MpcRunner(rclcpp::Node::SharedPtr node);

private:
  // Node handler
  rclcpp::Node::SharedPtr node_;
  // State subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPositionGroundtruth>::SharedPtr local_position_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeGroundtruth>::SharedPtr attitude_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocityGroundtruth>::SharedPtr angular_velocity_subs_;  
  // Other subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr ctrl_mode_subs_;
  
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::ActuatorDirectControl>::SharedPtr actuator_direct_control_pub_;
  
  // State subscribers callbacks
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPositionGroundtruth::UniquePtr msg);
  void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitudeGroundtruth::UniquePtr msg);
  void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocityGroundtruth::UniquePtr msg);
  
  void vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);

  // Publishers timer callback
  void pubActuatorOutputsTimerCallback();
  void publishControls();
  
  // Timer publishers
  rclcpp::TimerBase::SharedPtr actuator_direct_control_timer_;

  // MPC related
  // multicopter_mpc::MpcMain mpc_main_;

  // Class variables
  Eigen::VectorXd state_;
  Eigen::Vector3d vel_ned_;
  Eigen::Vector3d vel_frd_;
  Eigen::Vector3d actuator_normalized_;
  Eigen::Quaterniond q_ned_frd_;
  Eigen::Quaterniond q_nwu_flu_;
  bool offboard_mode_enabled_;

  px4_msgs::msg::ActuatorDirectControl actuator_direct_control_msg_;

  // Transformation tools
  const Eigen::Quaterniond FRD_FLU_Q = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);
  const Eigen::Quaterniond NWU_NED_Q = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

};

#endif

