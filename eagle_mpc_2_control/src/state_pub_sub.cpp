#include "eagle_mpc_2_control/state_pub_sub.hpp"

using namespace std::chrono_literals;

StatePubSub::StatePubSub(rclcpp::Node::SharedPtr node, const bool& sub, const bool& pub) : node_(node) {
  if (!sub && !pub) {
    // throw error if both are false
  }

  platform_state_publisher_ = node_->create_publisher<eagle_mpc_2_msgs::msg::PlatformState>("PlatformState", 10);

  timesync_sub_ = node_->create_subscription<px4_msgs::msg::Timesync>(
      "Timesync_PubSubTopic", 10, std::bind(&StatePubSub::timeSyncCallback, this, std::placeholders::_1));
  local_position_subs_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "VehicleLocalPosition_PubSubTopic", 10,
      std::bind(&StatePubSub::vehicleLocalPositionCallback, this, std::placeholders::_1));
  attitude_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "VehicleAttitude_PubSubTopic", 10,
      std::bind(&StatePubSub::vehicleAttitudeCallback, this, std::placeholders::_1));
  angular_velocity_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "VehicleAngularVelocity_PubSubTopic", 10,
      std::bind(&StatePubSub::vehicleAngularVelocityCallback, this, std::placeholders::_1));

  // State variable init
  state_ = Eigen::VectorXd::Zero(13);
}

StatePubSub::~StatePubSub() {}

void StatePubSub::timeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
  timestamp_.store(msg->timestamp);
}

void StatePubSub::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
  // We use the local inertial frame: NWU
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

void StatePubSub::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
  q_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  q_nwu_flu_ = NWU_NED_Q * q_ned_frd_ * FRD_FLU_Q;
  state_(3) = q_nwu_flu_.x();
  state_(4) = q_nwu_flu_.y();
  state_(5) = q_nwu_flu_.z();
  state_(6) = q_nwu_flu_.w();
}

void StatePubSub::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg) {
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];

  publish_platform_state();
}

void StatePubSub::publish_platform_state() {
  // platform_state_msg_.header.frame_id = 0
  platform_state_msg_.header.stamp = node_->now();

  platform_state_msg_.pose.position.x = state_(0);
  platform_state_msg_.pose.position.y = state_(1);
  platform_state_msg_.pose.position.z = state_(2);
  platform_state_msg_.pose.orientation.x = state_(3);
  platform_state_msg_.pose.orientation.y = state_(4);
  platform_state_msg_.pose.orientation.z = state_(5);
  platform_state_msg_.pose.orientation.w = state_(6);

  platform_state_msg_.motion.linear.x = state_(7);
  platform_state_msg_.motion.linear.y = state_(8);
  platform_state_msg_.motion.linear.z = state_(9);
  platform_state_msg_.motion.angular.x = state_(10);
  platform_state_msg_.motion.angular.y = state_(11);
  platform_state_msg_.motion.angular.z = state_(12);

  platform_state_publisher_->publish(platform_state_msg_);
}