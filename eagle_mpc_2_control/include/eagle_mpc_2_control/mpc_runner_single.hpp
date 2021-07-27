#ifndef EAGLE_MPC_2_CONTROL_MPC_RUNNER_SINGLE_HPP
#define EAGLE_MPC_2_CONTROL_MPC_RUNNER_SINGLE_HPP

#include <pinocchio/multibody/model.hpp>

#include <chrono>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

// #include <px4_msgs/msg/timesync.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_direct_control.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>

#include <px4_ros_com/frame_transforms.h>

#include "eagle_mpc_2_msgs/msg/platform_state.hpp"

#include "eagle_mpc/trajectory.hpp"
#include "eagle_mpc/mpc-controllers/carrot-mpc.hpp"
#include "eagle_mpc/mpc-controllers/rail-mpc.hpp"
#include "eagle_mpc/mpc-controllers/weighted-mpc.hpp"
#include "eagle_mpc/mpc-base.hpp"
#include "eagle_mpc/utils/tools.hpp"

// Transformation tools
const Eigen::Quaterniond FRD_FLU_Q =
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);
const Eigen::Quaterniond NWU_NED_Q =
    px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

class MpcTest : public rclcpp::Node {
 public:
  explicit MpcTest(const std::string& node_name);
  virtual ~MpcTest();

 protected:
  std::atomic<uint64_t> timestamp_;  //!< common synced timestamped

  // Class variables
  std::mutex mut_state_;
  Eigen::VectorXd state_;  // local pos in inertial frame (NWU), quaternion (x,y,z,w. From FLU to NWU), lin. velocity
                           // and ang. velocity (FLU base frame)

 protected:
  // subs
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_subs_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr ctrl_mode_subs_;

  // State subscribers callbacks
  virtual void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
  void vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);

  // pubs
  rclcpp::Publisher<px4_msgs::msg::ActuatorDirectControl>::SharedPtr actuator_direct_control_pub_;

  px4_msgs::msg::ActuatorDirectControl actuator_direct_control_msg_;
  Eigen::VectorXd actuator_normalized_;

 private:
  // ROS2-Node related objects

  bool running_controller_;
  bool motor_control_mode_enabled_;

  uint64_t counter_;
  uint64_t last_timestamp_;
  uint64_t first_timestamp_;
  bool is_first_;
  double motor_value_;

//   rclcpp::Duration time_elapsed_;
  rclcpp::Time last_velocity_time_;
};
#endif
