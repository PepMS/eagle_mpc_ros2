#include "px4_mpc/mpc_runner.hpp"

using namespace std::chrono_literals;

MpcRunner::MpcRunner(rclcpp::Node::SharedPtr node) : node_(node)
{
  // State subscribers
  local_position_subs_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPositionGroundtruth>("VehicleLocalPositionGroundtruth_PubSubTopic", 10, std::bind(&MpcRunner::vehicleLocalPositionCallback, this, std::placeholders::_1));
  attitude_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAttitudeGroundtruth>("VehicleAttitudeGroundtruth_PubSubTopic", 10, std::bind(&MpcRunner::vehicleAttitudeCallback, this, std::placeholders::_1));
  angular_velocity_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAngularVelocityGroundtruth>("VehicleAngularVelocityGroundtruth_PubSubTopic", 10, std::bind(&MpcRunner::vehicleAngularVelocityCallback, this, std::placeholders::_1));

  ctrl_mode_subs_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>("VehicleControlMode_PubSubTopic", 10, std::bind(&MpcRunner::vehicleCtrlModeCallback, this, std::placeholders::_1));

  actuator_direct_control_pub_ = node->create_publisher<px4_msgs::msg::ActuatorDirectControl>("ActuatorDirectControl_PubSubTopic", 10);

  // actuator_direct_control_timer_ = node_->create_wall_timer(4ms, std::bind(&MpcRunner::pubActuatorOutputsTimerCallback, this));

  // Variable initialization
  state_ = Eigen::VectorXd::Zero(13);
  vel_ned_ = Eigen::Vector3d::Zero(3);
  actuator_normalized_ = Eigen::Vector3d::Zero(4);

  // mpc_main_ = multicopter_mpc::MpcMain(multicopter_mpc::MultiCopterTypes::Iris, multicopter_mpc::MissionTypes::Hover, multicopter_mpc::SolverTypes::BoxFDDP);
}

void MpcRunner::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPositionGroundtruth::UniquePtr msg)
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

void MpcRunner::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitudeGroundtruth::UniquePtr msg)
{
  q_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  q_nwu_flu_ = NWU_NED_Q * q_ned_frd_ * FRD_FLU_Q;
  state_(3) = q_nwu_flu_.x();
  state_(4) = q_nwu_flu_.y();
  state_(5) = q_nwu_flu_.z();
  state_(6) = q_nwu_flu_.w();
}

void MpcRunner::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocityGroundtruth::UniquePtr msg)
{
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];
  // std::cout << "This is the state: " << std::endl
  //           << state_ << std::endl;
  // mpc_main_.setInitialState(state_);
  // mpc_main_.solve();
  publishControls();
  // std::cout << "Control output: \n"
  //           << mpc_main_.getActuatorControlsNormalized() << std::endl;
}

void MpcRunner::vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  offboard_mode_enabled_ = msg->flag_control_motors_enabled;
  if (offboard_mode_enabled_)
    std::cout << "Direct control enabled!" << std::endl;
}

void MpcRunner::pubActuatorOutputsTimerCallback()
{
  std::cout << "Publishing inside timer" << std::endl;
  actuator_direct_control_msg_.timestamp = (uint64_t)std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  // actuator_normalized_ = mpc_main_.getActuatorControlsNormalized();

  actuator_direct_control_msg_.output[0] = actuator_normalized_(0);
  actuator_direct_control_msg_.output[1] = actuator_normalized_(1);
  actuator_direct_control_msg_.output[2] = actuator_normalized_(2);
  actuator_direct_control_msg_.output[3] = actuator_normalized_(3);
  actuator_direct_control_pub_->publish(actuator_direct_control_msg_);
}

void MpcRunner::publishControls()
{
  actuator_direct_control_msg_.timestamp = (uint64_t)std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  // actuator_normalized_ = mpc_main_.getActuatorControlsNormalized();

  actuator_direct_control_msg_.output[0] = actuator_normalized_(0);
  actuator_direct_control_msg_.output[1] = actuator_normalized_(1);
  actuator_direct_control_msg_.output[2] = actuator_normalized_(2);
  actuator_direct_control_msg_.output[3] = actuator_normalized_(3);
  actuator_direct_control_pub_->publish(actuator_direct_control_msg_);
}
