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

#ifndef PX4_MPC_CONTROLLER_BASE_HPP
#define PX4_MPC_CONTROLLER_BASE_HPP

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


struct ControllerType {
  enum Type {MpcController, ExampleController, NbControllerTypes};
};

class ControllerAbstract
{
public:
  explicit ControllerAbstract(rclcpp::Node::SharedPtr node);
  virtual ~ControllerAbstract();

  static std::shared_ptr<ControllerAbstract> createController(const ControllerType::Type& controller_type, rclcpp::Node::SharedPtr node);
protected:
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
  virtual void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPositionGroundtruth::UniquePtr msg);
  virtual void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitudeGroundtruth::UniquePtr msg);
  virtual void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocityGroundtruth::UniquePtr msg);

  virtual void vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);

  // Publishers timer callback
  virtual void pubActuatorOutputsTimerCallback();
  virtual void publishControls();

  // Timer publishers
  rclcpp::TimerBase::SharedPtr actuator_direct_control_timer_;
  
  // Message to publish motor commands
  px4_msgs::msg::ActuatorDirectControl actuator_direct_control_msg_;

  // Class variables
  Eigen::VectorXd state_; // local pos in inertial frame (NWU), quaternion, lin. velocity and ang. velocity in FLU base frame
  Eigen::Vector3d vel_ned_;
  Eigen::Vector3d vel_frd_;
  Eigen::Quaterniond q_ned_frd_;
  Eigen::Quaterniond q_nwu_flu_;
  
  // Motor speed commands in range [-1, 1]
  Eigen::Vector4d actuator_normalized_; 
  bool motor_control_mode_enabled_;


  // Transformation tools
  const Eigen::Quaterniond FRD_FLU_Q = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);
  const Eigen::Quaterniond NWU_NED_Q = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);
};

#endif
