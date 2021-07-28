////////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 Pep Martí Saumell
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_2_CONTROL_CONTROLLER_BASE_HPP
#define EAGLE_MPC_2_CONTROL_CONTROLLER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <Eigen/Dense>

#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_direct_control.hpp>

#include "eagle_mpc_2_control/state_pub_sub.hpp"

class ControllerAbstract : public StatePubSub {
    public:
    explicit ControllerAbstract(const std::string& node_name);
    virtual ~ControllerAbstract();

    protected:
    // Other subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr ctrl_mode_subs_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ActuatorDirectControl>::SharedPtr actuator_direct_control_pub_;

    // State subscribers callbacks
    void vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    virtual void handleVehicleCtrlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);

    // Publishers timer callback
    void timerComputeControlsCallback();
    virtual void computeControls() = 0;
    virtual void publishControls();

    // Timer publishers
    rclcpp::TimerBase::SharedPtr compute_controls_timer_;

    // Message to publish motor commands
    px4_msgs::msg::ActuatorDirectControl actuator_direct_control_msg_;

    // Motor speed commands in range [-1, 1]
    Eigen::VectorXd actuator_normalized_;
    bool platform_motor_control_enabled_;
    bool platform_armed_;
};

#endif
