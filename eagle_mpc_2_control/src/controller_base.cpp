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

#include "eagle_mpc_2_control/controller_base.hpp"

using namespace std::chrono_literals;

ControllerAbstract::ControllerAbstract(const std::string& node_name) : StatePubSub(node_name) {
    // Publisher
    actuator_direct_control_pub_ =
        create_publisher<px4_msgs::msg::ActuatorDirectControl>("ActuatorDirectControl_PubSubTopic", 1);

    rclcpp::SubscriptionOptions sub_opt_other = rclcpp::SubscriptionOptions();
    sub_opt_other.callback_group = callback_group_other_;

    ctrl_mode_subs_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
        "VehicleControlMode_PubSubTopic", rclcpp::QoS(1),
        std::bind(&ControllerAbstract::vehicleCtrlModeCallback, this, std::placeholders::_1), sub_opt_other);

    compute_controls_timer_ = create_wall_timer(
        4ms, std::bind(&ControllerAbstract::timerComputeControlsCallback, this), callback_group_commands_);

    // Variable initialization
    actuator_normalized_ = -Eigen::VectorXd::Ones(4);
}

ControllerAbstract::~ControllerAbstract() {}

void ControllerAbstract::vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
    handleVehicleCtrlMode(msg);
}

void ControllerAbstract::handleVehicleCtrlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
    if (msg->flag_control_motors_enabled && !platform_motor_control_enabled_) {
        RCLCPP_WARN(get_logger(), "Direct control enabled!");
    }
    platform_motor_control_enabled_ = msg->flag_control_motors_enabled;
    platform_armed_ = msg->flag_armed;
}

void ControllerAbstract::timerComputeControlsCallback() {
    computeControls();
    publishControls();
}

void ControllerAbstract::publishControls() {
    actuator_direct_control_msg_.timestamp = timestamp_.load();
    actuator_direct_control_msg_.noutputs = 4;

    actuator_direct_control_msg_.output[0] = actuator_normalized_(0);
    actuator_direct_control_msg_.output[1] = actuator_normalized_(1);
    actuator_direct_control_msg_.output[2] = actuator_normalized_(2);
    actuator_direct_control_msg_.output[3] = actuator_normalized_(3);

    actuator_direct_control_pub_->publish(actuator_direct_control_msg_);
}