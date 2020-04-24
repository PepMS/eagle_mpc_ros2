#include "px4_mpc/mpc_runner.hpp"


using namespace std::chrono_literals;

MpcRunner::MpcRunner(rclcpp::Node::SharedPtr node) : node_(node)
{
  // State subscribers
  local_position_subs_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>("VehicleLocalPosition_PubSubTopic", std::bind(&MpcRunner::vehicleLocalPositionCallback, this, std::placeholders::_1));
  attitude_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>("VehicleAttitude_PubSubTopic", std::bind(&MpcRunner::vehicleAttitudeCallback, this, std::placeholders::_1));
  angular_velocity_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("VehicleAngularVelocity_PubSubTopic", std::bind(&MpcRunner::vehicleAngularVelocityCallback, this, std::placeholders::_1));
  
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
  vel_ned_ = Eigen::Vector3d::Zero(3);
}

void MpcRunner::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
  // In Crocoddyl the local inertial frame is ENU
  // Position in the intertial frame
  state_(0) = msg->y;
  state_(1) = msg->x;
  state_(2) = -msg->z;

  // Velocity in the body frame (NED)
  vel_ned_(0) = msg->vx;
  vel_ned_(1) = msg->vy;
  vel_ned_(2) = msg->vz;
  vel_body_ned_ = px4_ros_com::frame_transforms::transform_frame(vel_ned_, q_body_ned_.conjugate());
  state_(7) = vel_body_ned_(0);
  state_(8) = -vel_body_ned_(1);
  state_(9) = -vel_body_ned_(2);
}

void MpcRunner::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg)
{
  q_body_ned_.w() = msg->q[0];
  q_body_ned_.x() = msg->q[1];
  q_body_ned_.y() = msg->q[2];
  q_body_ned_.z() = msg->q[3];
  state_(3) = q_body_ned_.x();
  state_(4) = q_body_ned_.y();
  state_(5) = q_body_ned_.z();
  state_(6) = q_body_ned_.w();
}

void MpcRunner::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg)
{
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];
  std::cout << "This is the state: " << std::endl << state_ << std::endl;
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
