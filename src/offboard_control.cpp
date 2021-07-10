#include "px4_mpc/offboard_control.hpp"

using namespace std::chrono_literals;

OffboardControl::OffboardControl(rclcpp::Node::SharedPtr node) : node_(node)
{
  offboard_control_mode_publisher_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
  trajectory_setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
  vehicle_command_publisher_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);

  timesync_sub_ = node_->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10, std::bind(&OffboardControl::timeSyncCallback, this, std::placeholders::_1));

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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto n = rclcpp::Node::make_shared("mpc_runner");
  std::shared_ptr<OffboardControl> controller = std::make_shared<OffboardControl>(n);

  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}