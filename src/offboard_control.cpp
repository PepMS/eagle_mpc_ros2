#include "px4_mpc/offboard_control.hpp"

using namespace std::chrono_literals;

OffboardControl::OffboardControl(rclcpp::Node::SharedPtr node) : node_(node)
{
  offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
  trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
  vehicle_command_publisher_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);
  platform_state_publisher_ = node_->create_publisher<eagle_interfaces::msg::PlatformState>("PlatformState", 10);

  timesync_sub_ = node_->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10, std::bind(&OffboardControl::timeSyncCallback, this, std::placeholders::_1));
  local_position_subs_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>("VehicleLocalPosition_PubSubTopic", 10, std::bind(&OffboardControl::vehicleLocalPositionCallback, this, std::placeholders::_1));
  attitude_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>("VehicleAttitude_PubSubTopic", 10, std::bind(&OffboardControl::vehicleAttitudeCallback, this, std::placeholders::_1));
  angular_velocity_subs_ = node_->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("VehicleAngularVelocity_PubSubTopic", 10, std::bind(&OffboardControl::vehicleAngularVelocityCallback, this, std::placeholders::_1));

  offboard_setpoint_counter_ = 0;
  timer_ = node_->create_wall_timer(100ms, std::bind(&OffboardControl::timerCallback, this));
  timer_wp_ = node_->create_wall_timer(5s, std::bind(&OffboardControl::timerWpCallback, this));

  // Load wps
  px4_msgs::msg::TrajectorySetpoint wp;
  wp.x = 0;
  wp.y = 0;
  wp.z = -2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  wp.x = 1;
  wp.y = 0;
  wp.z = -2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  wp.x = 1;
  wp.y = -1;
  wp.z = -2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  wp.x = 0;
  wp.y = -1;
  wp.z = -2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  waypoint_active_ = 0;

  // State variable init
  state_ = Eigen::VectorXd::Zero(13);
}

OffboardControl::~OffboardControl() {}

void OffboardControl::timeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg)
{
  timestamp_.store(msg->timestamp);
}

void OffboardControl::timerCallback()
{
  if (offboard_setpoint_counter_ == 10)
  {
    // Change to Offboard mode after 10 setpoints
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

    // Arm the vehicle
    this->arm();
  }

  // offboard_control_mode needs to be paired with trajectory_setpoint
  publish_offboard_control_mode();
  publish_trajectory_setpoint();

  // stop the counter after reaching 11
  if (offboard_setpoint_counter_ < 11)
  {
    offboard_setpoint_counter_++;
  }
}

void OffboardControl::timerWpCallback()
{
  waypoint_active_++;
  waypoint_active_ = waypoint_active_ == waypoints_.size() ? waypoints_.size() - 1 : waypoint_active_;
}

void OffboardControl::arm() const
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(node_->get_logger(), "Arm command send");
}

void OffboardControl::disarm() const
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(node_->get_logger(), "Disarm command send");
}

void OffboardControl::publish_offboard_control_mode() const
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = timestamp_.load();
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() const
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = timestamp_.load();
  msg.x = waypoints_[waypoint_active_].x;
  msg.y = waypoints_[waypoint_active_].y;
  msg.z = waypoints_[waypoint_active_].z;
  msg.yaw = waypoints_[waypoint_active_].yaw; // [-PI:PI]

  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
                                              float param2) const
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = timestamp_.load();
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  vehicle_command_publisher_->publish(msg);
}

void OffboardControl::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
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

void OffboardControl::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg)
{
  q_ned_frd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  q_nwu_flu_ = NWU_NED_Q * q_ned_frd_ * FRD_FLU_Q;
  state_(3) = q_nwu_flu_.x();
  state_(4) = q_nwu_flu_.y();
  state_(5) = q_nwu_flu_.z();
  state_(6) = q_nwu_flu_.w();
}

void OffboardControl::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg)
{
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];

  publish_platform_state();
}

void OffboardControl::publish_platform_state()  {
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


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto n = rclcpp::Node::make_shared("mpc_runner");
  std::shared_ptr<OffboardControl> controller = std::make_shared<OffboardControl>(n);

  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}