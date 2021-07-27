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

#include <iostream>

#include "eagle_mpc_2_control/mpc_runner.hpp"

using namespace std::chrono_literals;

MpcRunner::MpcRunner(const std::string& node_name) : ControllerAbstract(node_name) {
  declareParameters();

  vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 1);

  running_controller_ = false;
  RCLCPP_WARN(get_logger(), "ORIGINAL MPC RUNNER");
}

MpcRunner::~MpcRunner() {}

void MpcRunner::declareParameters() {
  // Node parameter
  declare_parameter<bool>("enable_controller", false);

  // Trajectory
  declare_parameter<std::string>("trajectory_config_path", "");
  declare_parameter<int>("trajectory_dt", 10);
  declare_parameter<std::string>("trajectory_solver", "");
  declare_parameter<std::string>("trajectory_integration", "");

  // Mpc Controller
  declare_parameter<std::string>("mpc_config_path", "");
  declare_parameter<std::string>("mpc_type", "");

  // Callbacks
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  param_callback_handle_ = param_subscriber_->add_parameter_callback(
      "enable_controller", std::bind(&MpcRunner::enableControllerCallback, this, std::placeholders::_1));
}

void MpcRunner::enableControllerCallback(const rclcpp::Parameter& p) {
  bool param = p.as_bool();
  if (param && !running_controller_) {
    enablingProcedure();
  } else if (!param && running_controller_) {
    disablingProcedure();
  }
}

void MpcRunner::enablingProcedure() {
  RCLCPP_WARN(get_logger(), "MPC Controller state: ENABLING (Safety Checks)");
  // To Check:
  // 1. we are receiving the state from the PX4 side

  RCLCPP_INFO(get_logger(), "MPC Controller state: ENABLING (Loading Parameters)");
  dumpParameters();

  initializeMpcController();

  RCLCPP_INFO(get_logger(), "MPC Controller state: ENABLING (Switching to Motor Control Mode & Arming)");
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10);  // 10 = PX4_MOTOR_CONTROL_MODE
  arm();

  RCLCPP_WARN(get_logger(), "MPC Controller state: ENABLED");
  running_controller_ = true;
  controller_start_time_ = timestamp_.load();
}

void MpcRunner::dumpParameters() {
  // Trajectory
  get_parameter("trajectory_config_path", node_params_.trajectory_config_path);
  get_parameter("trajectory_dt", node_params_.trajectory_dt);
  node_params_.trajectory_solver = eagle_mpc::SolverTypes_map.at(get_parameter("trajectory_solver").as_string());
  get_parameter("trajectory_integration", node_params_.trajectory_integration);
  node_params_.trajectory_squash = node_params_.trajectory_solver == eagle_mpc::SolverTypes::SolverSbFDDP;

  // MpcController
  get_parameter("mpc_config_path", node_params_.mpc_config_path);
  get_parameter("mpc_type", node_params_.mpc_type);
}

void MpcRunner::initializeMpcController() {
  RCLCPP_INFO(get_logger(), "MPC Controller state: ENABLING (Initializing trajectory)");

  trajectory_ = eagle_mpc::Trajectory::create();
  trajectory_->autoSetup(node_params_.trajectory_config_path);

  boost::shared_ptr<crocoddyl::ShootingProblem> problem = trajectory_->createProblem(
      node_params_.trajectory_dt, node_params_.trajectory_squash, node_params_.trajectory_integration);

  boost::shared_ptr<crocoddyl::SolverFDDP> solver;
  switch (node_params_.trajectory_solver) {
    case eagle_mpc::SolverTypes::SolverSbFDDP:
      solver = boost::make_shared<eagle_mpc::SolverSbFDDP>(problem, trajectory_->get_squash());
      break;
    case eagle_mpc::SolverTypes::SolverBoxFDDP:
      solver = boost::make_shared<crocoddyl::SolverBoxFDDP>(problem);
      break;
  }
  std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> callbacks;
  callbacks.push_back(boost::make_shared<crocoddyl::CallbackVerbose>());
  solver->setCallbacks(callbacks);

  // TODO: Check that the current state is close to the initial state of the trajectory
  // Then solve
  solver->solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR);

  RCLCPP_INFO(get_logger(), "MPC Controller state: ENABLING (Initializing MpcController)");
  switch (eagle_mpc::MpcTypes_map.at(node_params_.mpc_type)) {
    case eagle_mpc::MpcTypes::Carrot:
      mpc_controller_ = boost::make_shared<eagle_mpc::CarrotMpc>(
          trajectory_, solver->get_xs(), node_params_.trajectory_dt, node_params_.mpc_config_path);
      break;
    case eagle_mpc::MpcTypes::Rail:
      mpc_controller_ = boost::make_shared<eagle_mpc::RailMpc>(solver->get_xs(), node_params_.trajectory_dt,
                                                               node_params_.mpc_config_path);
      break;
    case eagle_mpc::MpcTypes::Weighted:
      mpc_controller_ = boost::make_shared<eagle_mpc::WeightedMpc>(trajectory_, node_params_.trajectory_dt,
                                                                   node_params_.mpc_config_path);
  }

  if (mpc_controller_->get_solver_type() == eagle_mpc::SolverTypes::SolverSbFDDP) {
    boost::static_pointer_cast<eagle_mpc::SolverSbFDDP>(mpc_controller_->get_solver())->set_convergence_init(1e-3);
  }

  control_command_ = Eigen::VectorXd::Zero(mpc_controller_->get_actuation()->get_nu());
  thrust_command_ = Eigen::VectorXd::Zero(mpc_controller_->get_platform_params()->n_rotors_);
}

void MpcRunner::computeControls() {
  if (running_controller_) {
    mut_state_.lock();
    mpc_controller_->get_problem()->set_x0(state_);
    mut_state_.unlock();

    controller_time_ = timestamp_.load() - controller_start_time_;
    controller_instant_ = (uint64_t)(controller_time_ / 1000.0);
    mpc_controller_->updateProblem(controller_instant_);

    mpc_controller_->get_solver()->solve(mpc_controller_->get_solver()->get_xs(),
                                         mpc_controller_->get_solver()->get_us(), mpc_controller_->get_iters());

    // PROPERLY HANDLE SOLVER TYPE!
    control_command_ =
        boost::static_pointer_cast<eagle_mpc::SolverSbFDDP>(mpc_controller_->get_solver())->getSquashControls()[0];

    thrust_command_ = control_command_.head(thrust_command_.size());
    eagle_mpc::Tools::thrustToSpeedNormalized(thrust_command_, mpc_controller_->get_platform_params(),
                                              actuator_normalized_);
  }
}

void MpcRunner::arm() const {
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(get_logger(), "Arm command send");
}

void MpcRunner::disablingProcedure() {
  RCLCPP_WARN(get_logger(), "MPC Controller. DISABLING");
  // Change to a safe flight mode

  // If all safety checks are correct, change flihgt mode, arm and start mission
  running_controller_ = false;
  RCLCPP_WARN(get_logger(), "MPC Controller state: DISABLED");
}

void MpcRunner::disarm() const {
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(get_logger(), "Disarm command send");
}


void MpcRunner::publishVehicleCommand(uint16_t command, float param1, float param2) const {
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MpcRunner> controller = std::make_shared<MpcRunner>("MpcRunner");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}