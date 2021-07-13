#include "eagle_mpc_2_control/decentralized_controller.hpp"

using namespace std::chrono_literals;

DecentralizedController::DecentralizedController(rclcpp::Node::SharedPtr node) : StatePubSub(node) {
  offboard_control_mode_publisher_ =
      node_->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
  trajectory_setpoint_publisher_ =
      node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
  vehicle_command_publisher_ =
      node_->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);

  offboard_setpoint_counter_ = 0;
  timer_ = node_->create_wall_timer(100ms, std::bind(&DecentralizedController::timerCallback, this));

  local_position_subs_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "VehicleLocalPosition_PubSubTopic", 10,
      std::bind(&DecentralizedController::vehicleLocalPositionCallback, this, std::placeholders::_1));

  // Load wps (stored in NWU)
  px4_msgs::msg::TrajectorySetpoint wp;
  wp.x = 0;
  wp.y = 0;
  wp.z = 2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  wp.x = 1;
  wp.y = 0;
  wp.z = 2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  wp.x = 1;
  wp.y = 1;
  wp.z = 2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  wp.x = 0;
  wp.y = 1;
  wp.z = 2;
  wp.yaw = 0;
  waypoints_.push_back(wp);

  waypoint_active_ = 0;

  // State variable init
  state_ = Eigen::VectorXd::Zero(13);
}

DecentralizedController::~DecentralizedController() {}

void DecentralizedController::timerCallback() {
  if (offboard_setpoint_counter_ == 10) {
    // Change to Offboard mode after 10 setpoints
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

    // Arm the vehicle
    this->arm();
  }

  // offboard_control_mode needs to be paired with trajectory_setpoint
  publish_offboard_control_mode();
  publish_trajectory_setpoint();

  // stop the counter after reaching 11
  if (offboard_setpoint_counter_ < 11) {
    offboard_setpoint_counter_++;
  }
}

void DecentralizedController::arm() const {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(node_->get_logger(), "Arm command send");
}

void DecentralizedController::disarm() const {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(node_->get_logger(), "Disarm command send");
}

void DecentralizedController::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  StatePubSub::vehicleLocalPositionCallback(msg);

  Eigen::Vector3d pos_des;
  Eigen::Vector3d pos_err;

  pos_des << waypoints_[waypoint_active_].x, waypoints_[waypoint_active_].y, waypoints_[waypoint_active_].z;
  pos_err = pos_des - state_.head(3);

  if (pos_err.norm() < 0.05 && waypoint_active_ < waypoints_.size() - 1) {
    waypoint_active_++;
    RCLCPP_INFO(node_->get_logger(), "Waypoint %d activated", waypoint_active_);
  }
}

void DecentralizedController::publish_offboard_control_mode() const {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = timestamp_.load();
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  offboard_control_mode_publisher_->publish(msg);
}

void DecentralizedController::publish_trajectory_setpoint() const {
  px4_msgs::msg::TrajectorySetpoint msg{};

  // Waypoints must be in NED
  msg.timestamp = timestamp_.load();
  msg.x = waypoints_[waypoint_active_].x;
  msg.y = -waypoints_[waypoint_active_].y;
  msg.z = -waypoints_[waypoint_active_].z;
  msg.yaw = waypoints_[waypoint_active_].yaw;  // [-PI:PI]

  trajectory_setpoint_publisher_->publish(msg);
}

void DecentralizedController::publish_vehicle_command(uint16_t command, float param1, float param2) const {
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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto n = rclcpp::Node::make_shared("mpc_runner");
  std::shared_ptr<DecentralizedController> controller = std::make_shared<DecentralizedController>(n);

  rclcpp::spin(n);
  rclcpp::shutdown();

  return 0;
}