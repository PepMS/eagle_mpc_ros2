#include "eagle_mpc_2_control/mpc_runner_single.hpp"
#include <inttypes.h>

using namespace std::chrono_literals;

MpcTest::MpcTest(const std::string& node_name) : rclcpp::Node(node_name) {
  actuator_direct_control_pub_ =
      create_publisher<px4_msgs::msg::ActuatorDirectControl>("ActuatorDirectControl_PubSubTopic", 1);

  angular_velocity_subs_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "VehicleAngularVelocity_PubSubTopic", rclcpp::QoS(10),
      std::bind(&MpcTest::vehicleAngularVelocityCallback, this, std::placeholders::_1));

  ctrl_mode_subs_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      "VehicleControlMode_PubSubTopic", rclcpp::QoS(10),
      std::bind(&MpcTest::vehicleCtrlModeCallback, this, std::placeholders::_1));

  // State variable init
  state_ = Eigen::VectorXd::Zero(13);
  actuator_normalized_ = Eigen::VectorXd::Zero(4);
  counter_ = 0;
  motor_value_ = -1.0;
  last_timestamp_ = 0;
  last_velocity_time_ = get_clock()->now();
  is_first_ = true;
}

MpcTest::~MpcTest() {}

void MpcTest::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
  mut_state_.lock();
  state_(10) = msg->xyz[0];
  state_(11) = -msg->xyz[1];
  state_(12) = -msg->xyz[2];
  mut_state_.unlock();

  timestamp_.store(msg->timestamp);

  if (motor_control_mode_enabled_) {
    if (is_first_) {
      first_timestamp_ = msg->timestamp;
      is_first_ = false;
    }
    actuator_normalized_[0] = motor_value_;
    actuator_normalized_[1] = motor_value_;
    actuator_normalized_[2] = motor_value_;
    actuator_normalized_[3] = motor_value_;

    actuator_direct_control_msg_.timestamp = msg->timestamp;
    actuator_direct_control_msg_.output[0] = actuator_normalized_[0];
    actuator_direct_control_msg_.output[1] = actuator_normalized_[1];
    actuator_direct_control_msg_.output[2] = actuator_normalized_[2];
    actuator_direct_control_msg_.output[3] = actuator_normalized_[3];

    if (counter_ < 1000) {
      actuator_direct_control_pub_->publish(actuator_direct_control_msg_);
      printf("Timestamp: %" PRIu64 "\n" , msg->timestamp - first_timestamp_);
      if (msg->timestamp - last_timestamp_ > 4000) {
        RCLCPP_WARN(get_logger(), "Too much time");
        // time_elapsed_ = ;
        RCLCPP_WARN(get_logger(), "Time in node: %ld", (get_clock()->now() - last_velocity_time_).nanoseconds());
      }
      printf("Value: %lf \n\n\n", motor_value_);
      last_timestamp_ = msg->timestamp;
      counter_++;
      motor_value_ += 0.001;
      last_velocity_time_ = get_clock()->now();
    }

    if (motor_value_ >= 1) {
      motor_value_ = -1.0;
    }
  }
}

void MpcTest::vehicleCtrlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
  if (msg->flag_control_motors_enabled && !motor_control_mode_enabled_) {
    RCLCPP_WARN(get_logger(), "Direct control enabled!");
  }
  motor_control_mode_enabled_ = msg->flag_control_motors_enabled;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MpcTest> controller = std::make_shared<MpcTest>("MpcTest");

  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(controller);

  // executor.spin();
  rclcpp::spin(controller);
  rclcpp::shutdown();

  return 0;
}