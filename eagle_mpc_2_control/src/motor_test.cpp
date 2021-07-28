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

#include "eagle_mpc_2_control/motor_test.hpp"

using namespace std::chrono_literals;

namespace eagle_mpc_ros2 {

MotorTest::MotorTest(const std::string& node_name) : ControllerAbstract(node_name) {
    actuator_normalized_[0] = -1.0;
    actuator_normalized_[1] = -1.0;
    actuator_normalized_[2] = -1.0;
    actuator_normalized_[3] = -1.0;

    RCLCPP_INFO(get_logger(), "MOTOR TEST Created!");

    counter_ = 0;
    motor_idx_ = 0;

    change_motor_timer_ =
        create_wall_timer(5s, std::bind(&MotorTest::changeMotorCallback, this), callback_group_other_);

    rclcpp::SubscriptionOptions sub_opt_commands = rclcpp::SubscriptionOptions();
    sub_opt_commands.callback_group = callback_group_commands_;

    angular_velocity_subs_ = nullptr;
    angular_velocity_subs_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        "VehicleAngularVelocity_PubSubTopic", rclcpp::QoS(1),
        std::bind(&MotorTest::vehicleAngularVelocityCallback, this, std::placeholders::_1), sub_opt_commands);

    motor_value_ = -1.0;
}

MotorTest::~MotorTest() {}

void MotorTest::computeControls() {}

void MotorTest::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
    mut_state_.lock();
    state_(10) = msg->xyz[0];
    state_(11) = -msg->xyz[1];
    state_(12) = -msg->xyz[2];
    mut_state_.unlock();

    timestamp_.store(msg->timestamp);

    if (platform_motor_control_enabled_ && motor_value_ < -0.975) {
        // actuator_normalized_[0] = motor_value_;
        // actuator_normalized_[1] = motor_value_;
        // actuator_normalized_[2] = motor_value_;
        // actuator_normalized_[3] = motor_value_;

        // actuator_direct_control_msg_.timestamp = timestamp_.load();
        actuator_direct_control_msg_.timestamp = msg->timestamp;
        actuator_direct_control_msg_.output[0] = motor_value_;
        actuator_direct_control_msg_.output[1] = motor_value_;
        actuator_direct_control_msg_.output[2] = motor_value_;
        actuator_direct_control_msg_.output[3] = motor_value_;

        actuator_direct_control_pub_->publish(actuator_direct_control_msg_);

        RCLCPP_INFO(get_logger(), "Sent motor value: %f", motor_value_);
        motor_value_ += 0.001;
    }
}

void MotorTest::publishControls() {}

void MotorTest::changeMotorCallback() {
    // RCLCPP_INFO(get_logger(), "Writing motor: %ld", motor_idx_);

    motor_idx_++;
    motor_idx_ = motor_idx_ > 3 ? 0 : motor_idx_;
}

}  // namespace eagle_mpc_ros2

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<eagle_mpc_ros2::MotorTest> controller = std::make_shared<eagle_mpc_ros2::MotorTest>("MotorTest");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}