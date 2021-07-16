#include "eagle_mpc_2_control/state_pub_sub.hpp"

using namespace std::chrono_literals;

StatePubSub::StatePubSub(const std::string& node_name, const bool& pub) : rclcpp::Node(node_name), pub_enabled_(pub) {
  if (pub_enabled_) {
    platform_state_publisher_ = create_publisher<eagle_mpc_2_msgs::msg::PlatformState>("PlatformState", 10);
  }
  
  callback_group_loader_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_sender_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opt_loader = rclcpp::SubscriptionOptions();
  rclcpp::SubscriptionOptions sub_opt_sender = rclcpp::SubscriptionOptions();
  sub_opt_loader.callback_group = callback_group_loader_;
  sub_opt_sender.callback_group = callback_group_sender_;

  timesync_sub_ = create_subscription<px4_msgs::msg::Timesync>(
      "Timesync_PubSubTopic", rclcpp::QoS(10), std::bind(&StatePubSub::timeSyncCallback, this, std::placeholders::_1),
      sub_opt_loader);
  local_position_subs_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "VehicleLocalPosition_PubSubTopic", rclcpp::QoS(10),
      std::bind(&StatePubSub::vehicleLocalPositionCallback, this, std::placeholders::_1), sub_opt_loader);
  attitude_subs_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "VehicleAttitude_PubSubTopic", rclcpp::QoS(10),
      std::bind(&StatePubSub::vehicleAttitudeCallback, this, std::placeholders::_1), sub_opt_loader);
  angular_velocity_subs_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "VehicleAngularVelocity_PubSubTopic", rclcpp::QoS(10),
      std::bind(&StatePubSub::vehicleAngularVelocityCallback, this, std::placeholders::_1), sub_opt_loader);

  // State variable init
  state_ = Eigen::VectorXd::Zero(13);
}

StatePubSub::~StatePubSub() {}

void StatePubSub::timeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) { timestamp_.store(msg->timestamp); }

void StatePubSub::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  // We use the local inertial frame: NWU
  // Velocity in the body frame (FLU)
  vel_ned_(0) = msg->vx;
  vel_ned_(1) = msg->vy;
  vel_ned_(2) = msg->vz;
  vel_frd_ = px4_ros_com::frame_transforms::transform_frame(vel_ned_, q_ned_frd_.conjugate());

  // Position in the inertial frame
  mut_state_.lock();
  state_(0) = msg->x;
  state_(1) = -msg->y;
  state_(2) = -msg->z;
  state_(7) = vel_frd_(0);
  state_(8) = -vel_frd_(1);
  state_(9) = -vel_frd_(2);
  mut_state_.unlock();
}

void StatePubSub::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  q_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  q_nwu_flu_ = NWU_NED_Q * q_ned_frd_ * FRD_FLU_Q;

  mut_state_.lock();
  state_(3) = q_nwu_flu_.x();
  state_(4) = q_nwu_flu_.y();
  state_(5) = q_nwu_flu_.z();
  state_(6) = q_nwu_flu_.w();
  mut_state_.unlock();
}

void StatePubSub::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
  mut_state_.lock();
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];
  mut_state_.unlock();

  if (pub_enabled_) {
    publish_platform_state();
  }
}

void StatePubSub::publish_platform_state() {
  // platform_state_msg_.header.frame_id = 0
  platform_state_msg_.header.stamp = now();

  mut_state_.lock();
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
  mut_state_.unlock();

  platform_state_publisher_->publish(platform_state_msg_);
}