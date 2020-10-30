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

#include "px4_mpc/controller_base.hpp"

using namespace std::chrono_literals;

ControllerAbstract::ControllerAbstract(rclcpp::Node::SharedPtr node) : node_(node)
{
  // State subscribers
  local_position_subs_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPositionGroundtruth>("VehicleLocalPositionGroundtruth_PubSubTopic", 10, std::bind(&ControllerAbstract::vehicleLocalPositionCallback, this, std::placeholders::_1));
  attitude_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAttitudeGroundtruth>("VehicleAttitudeGroundtruth_PubSubTopic", 10, std::bind(&ControllerAbstract::vehicleAttitudeCallback, this, std::placeholders::_1));
  angular_velocity_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAngularVelocityGroundtruth>("VehicleAngularVelocityGroundtruth_PubSubTopic", 10, std::bind(&ControllerAbstract::vehicleAngularVelocityCallback, this, std::placeholders::_1));

  ctrl_mode_subs_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>("VehicleControlMode_PubSubTopic", 10, std::bind(&ControllerAbstract::vehicleCtrlModeCallback, this, std::placeholders::_1));

  actuator_direct_control_pub_ = node->create_publisher<px4_msgs::msg::ActuatorDirectControl>("ActuatorDirectControl_PubSubTopic", 10);

  actuator_direct_control_timer_ = node_->create_wall_timer(4ms, std::bind(&ControllerAbstract::pubActuatorOutputsTimerCallback, this));

  // Variable initialization
  state_ = Eigen::VectorXd::Zero(13);
  vel_ned_ = Eigen::Vector3d::Zero();
  actuator_normalized_ = Eigen::Vector4d::Zero();
}

ControllerAbstract::~ControllerAbstract() {}

void ControllerAbstract::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPositionGroundtruth::UniquePtr msg)
{
  // In Crocoddyl the local inertial frame is NWU
  // Position in the inertial frame
  state_(0) = msg->x;
  state_(1) = -msg->y;
  state_(2) = -msg->z;

  // Velocity in the body frame (FLU)
  vel_ned_(0) = msg->vx;
  vel_ned_(1) = msg->vy;
  vel_ned_(2) = msg->vz;
  vel_frd_ = px4_ros_com::frame_transforms::transform_frame(vel_ned_, q_ned_frd_.conjugate());
  state_(7) = vel_frd_(0);
  state_(8) = -vel_frd_(1);
  state_(9) = -vel_frd_(2);
}

void ControllerAbstract::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitudeGroundtruth::UniquePtr msg)
{
  q_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  q_nwu_flu_ = NWU_NED_Q * q_ned_frd_ * FRD_FLU_Q;
  state_(3) = q_nwu_flu_.x();
  state_(4) = q_nwu_flu_.y();
  state_(5) = q_nwu_flu_.z();
  state_(6) = q_nwu_flu_.w();
}

void ControllerAbstract::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocityGroundtruth::UniquePtr msg)
{
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];
}

void ControllerAbstract::vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  if (msg->flag_control_motors_enabled && !motor_control_mode_enabled_)
  {
    RCLCPP_INFO(node_->get_logger(), "Direct control enabled!");
  }
  motor_control_mode_enabled_ = msg->flag_control_motors_enabled;
}

void ControllerAbstract::pubActuatorOutputsTimerCallback()
{
  publishControls();
}

void ControllerAbstract::publishControls()
{
  actuator_direct_control_msg_.timestamp = (uint64_t)std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

  actuator_direct_control_msg_.output[0] = actuator_normalized_(0);
  actuator_direct_control_msg_.output[1] = actuator_normalized_(1);
  actuator_direct_control_msg_.output[2] = actuator_normalized_(2);
  actuator_direct_control_msg_.output[3] = actuator_normalized_(3);
  
  actuator_direct_control_pub_->publish(actuator_direct_control_msg_);
}

#include "px4_mpc/mpc_runner.hpp"
#include "px4_mpc/example_controller.hpp"

std::shared_ptr<ControllerAbstract> ControllerAbstract::createController(const ControllerType::Type &controller_type, rclcpp::Node::SharedPtr node) {
  switch (controller_type)
  {
  case ControllerType::MpcController:
    return std::make_shared<MpcRunner>(node);
    break;
  
  case ControllerType::ExampleController:
  default:
    return std::make_shared<ExampleController>(node);
    break;
  }
}