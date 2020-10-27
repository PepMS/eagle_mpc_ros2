#ifndef PX4_MPC_HPP
#define PX4_MPC_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <Eigen/Dense>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position_groundtruth.hpp>
#include <px4_msgs/msg/vehicle_attitude_groundtruth.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity_groundtruth.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_direct_control.hpp>

#include <px4_ros_com/frame_transforms.h>

#include "controller_base.hpp"

class MpcRunner : public ControllerAbstract
{
public:
  explicit MpcRunner(rclcpp::Node::SharedPtr node);
  virtual ~MpcRunner();

private:
  // MPC related
  // multicopter_mpc::MpcMain mpc_main_;
};

#endif
