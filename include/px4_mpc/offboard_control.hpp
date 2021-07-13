#ifndef PX4_MPC_OFFBOARD_CONTROL_HPP
#define PX4_MPC_OFFBOARD_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>

#include <Eigen/Dense>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <px4_ros_com/frame_transforms.h>

#include "eagle_interfaces/msg/platform_state.hpp"

// Transformation tools
const Eigen::Quaterniond FRD_FLU_Q =
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);
const Eigen::Quaterniond NWU_NED_Q =
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

class OffboardControl {
 public:
  explicit OffboardControl(rclcpp::Node::SharedPtr node);
  virtual ~OffboardControl();

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_wp_;

  std::atomic<uint64_t> timestamp_;  //!< common synced timestamped
  std::size_t offboard_setpoint_counter_;

  std::vector<px4_msgs::msg::TrajectorySetpoint> waypoints_;
  std::size_t waypoint_active_;

  // pubs
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<eagle_interfaces::msg::PlatformState>::SharedPtr platform_state_publisher_;

  // subs
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_subs_;

  // Class variables
  Eigen::VectorXd state_;  // local pos in inertial frame (NWU), quaternion (x,y,z,w. From FLU to NWU), lin. velocity
                           // and ang. velocity (FLU base frame)
  Eigen::Vector3d vel_ned_;
  Eigen::Vector3d vel_frd_;
  Eigen::Quaterniond q_ned_frd_;
  Eigen::Quaterniond q_nwu_flu_;

  // callbacks
  void timeSyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void timerCallback();
  void timerWpCallback();
  // State subscribers callbacks
  virtual void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
  virtual void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg);
  virtual void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg);

  // methods
  void arm() const;
  void disarm() const;
  void publish_offboard_control_mode() const;
  void publish_trajectory_setpoint() const;
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
  void publish_platform_state();

  // msgs
  eagle_interfaces::msg::PlatformState platform_state_msg_;
};
#endif
