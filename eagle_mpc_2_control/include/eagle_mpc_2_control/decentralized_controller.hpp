#ifndef EAGLE_MPC_CONTROL_2_DECENTRALIZED_CONTROLLER_HPP
#define EAGLE_MPC_CONTROL_2_DECENTRALIZED_CONTROLLER_HPP

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

#include "eagle_mpc_2_control/state_pub_sub.hpp"

namespace eagle_mpc_ros2 {

class DecentralizedController : public StatePubSub {
    public:
    explicit DecentralizedController(const std::string& node_name);
    virtual ~DecentralizedController();

    private:
    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_wps_;

    std::size_t offboard_setpoint_counter_;

    std::vector<px4_msgs::msg::TrajectorySetpoint> waypoints_;
    uint64_t waypoint_active_;

    // pubs
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // callbacks
    void timerOffboardCallback();
    void timerWaypointCallback();

    // methods
    void arm() const;
    void disarm() const;
    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint() const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
};
}  // namespace eagle_mpc_ros2
#endif
