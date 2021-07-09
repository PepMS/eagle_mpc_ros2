#ifndef PX4_MPC_OFFBOARD_CONTROL_HPP
#define PX4_MPC_OFFBOARD_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

class OffboardControl
{
public:
  explicit OffboardControl(rclcpp::Node::SharedPtr node);
  virtual ~OffboardControl();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<uint64_t> timestamp_; //!< common synced timestamped
  std::size_t offboard_setpoint_counter_;

  // pubs
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  // subs
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

  // callbacks
  void timeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void timerCallback();

  // methods
  void arm() const;
  void disarm() const;
  void publish_offboard_control_mode() const;
  void publish_trajectory_setpoint() const;
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
};
#endif
