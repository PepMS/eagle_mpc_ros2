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

namespace eagle_mpc_ros2 {

MpcRunner::MpcRunner(const std::string& node_name) : ControllerAbstract(node_name) {
    vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);

    std::string service_name = std::string(this->get_name()) + "/state_transition";
    service_sm_transition_ = create_service<eagle_mpc_2_msgs::srv::MpcControllerTransition>(
        service_name.c_str(),
        std::bind(&MpcRunner::transitionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_other_);

    start_countdown_timer_ =
        create_wall_timer(1s, std::bind(&MpcRunner::timerStartCountdownCallback, this), callback_group_other_);
    start_countdown_timer_->cancel();

    declareParameters();

    sm_active_phase_ = IDLE;
}

MpcRunner::~MpcRunner() {}

const std::map<std::string, int> MpcRunner::SmTransitions = MpcRunner::createSmTransitionsMap();
const std::map<int, std::string> MpcRunner::SmStates = MpcRunner::createSmStatesMap();

void MpcRunner::transitionRequest(
    const std::shared_ptr<eagle_mpc_2_msgs::srv::MpcControllerTransition::Request> request,
    const std::shared_ptr<eagle_mpc_2_msgs::srv::MpcControllerTransition::Response> response) {
    int command = -1;
    if (SmTransitions.find(request->command) != SmTransitions.end()) {
        command = SmTransitions.at(request->command);
    }

    std::string message;
    switch (command) {
        case ENABLE:
            if (sm_active_phase_ == IDLE) {
                response->accepted = true;
                response->result = smEnable(message);
                response->message = message;
                if (response->result) {
                    sm_active_phase_ = ENABLED;
                } else {
                    RCLCPP_WARN(get_logger(), response->message.c_str());
                }
            } else {
                response->accepted = false;
                response->message = "Cannot ENABLE the controller. It is not in the IDLE state.";
                RCLCPP_WARN(get_logger(), response->message.c_str());
            }
            break;

        case START:
            if (sm_active_phase_ == ENABLED) {
                response->accepted = true;
                response->result = smStart(message);
                response->message = message;
                if (response->result) {
                    start_countdown_counter_ = 0;
                    start_countdown_timer_->reset();
                } else {
                    RCLCPP_WARN(get_logger(), response->message.c_str());
                }
            } else {
                response->accepted = false;
                response->message = "Cannot START the controller. It is not in the ENABLED state.";
                RCLCPP_WARN(get_logger(), response->message.c_str());
            }
            break;

        case DISABLE:
            if (sm_active_phase_ == RUNNING) {
                response->accepted = true;
                response->result = smDisable(message);
                response->message = message;
                RCLCPP_WARN(get_logger(), response->message.c_str());
                if (response->result) {
                    sm_active_phase_ = IDLE;
                }
            } else {
                response->accepted = false;
                response->message = "Cannot DISABLE controller. It is not in the RUNNING state.";
                RCLCPP_WARN(get_logger(), response->message.c_str());
            }
            break;

        default:
            response->accepted = false;
            response->result = false;
            response->message =
                "Command: " + request->command + " does not exist. Possible options are: 'enable'|'start'|'disable'";
            RCLCPP_WARN(get_logger(), response->message.c_str());
            break;
    }

    message = "MPC Controller state: " + SmStates.at(sm_active_phase_);
    RCLCPP_INFO(get_logger(), message.c_str());
}

bool MpcRunner::smEnable(std::string& message) {
    // Check that MicroRTPS Agent is publishing
    if (local_position_subs_->get_publisher_count() == 0 || attitude_subs_->get_publisher_count() == 0 ||
        angular_velocity_subs_->get_publisher_count() == 0) {
        message = "Cannot enable MPC Controller. Check MicroRTPS Agent is running.";
        return false;
    }

    // Check PX4 is running
    for (int i = 0; i < state_.size(); ++i) {
        if (state_(i) == 0) {
            message = "Cannot enable MPC Controller. Check PX4 is publishing the platform state.";
            return false;
        }
    }

    dumpParameters();

    if (!initializeMpcController(message)) {
        return false;
    }

    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10.0);
    arm();

    return true;
}

bool MpcRunner::smStart(std::string& message) {
    if (!platform_armed_) {
        message = "Arm the platform before starting the controller";
        return false;
    }
    if (!platform_motor_control_enabled_) {
        message = "The platform must be in motor control mode to start the controller";
        return false;
    }
    return true;
}

void MpcRunner::handleVehicleCtrlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
    if (msg->flag_control_motors_enabled && !platform_motor_control_enabled_) {
        RCLCPP_WARN(get_logger(), "Direct control enabled!");
    }

    if (platform_armed_ && !msg->flag_armed) {
        sm_active_phase_ = IDLE;
        RCLCPP_INFO(get_logger(), "MPC Controller state: %s", SmStates.at(sm_active_phase_).c_str());
    }
    platform_motor_control_enabled_ = msg->flag_control_motors_enabled;
    platform_armed_ = msg->flag_armed;
}

bool MpcRunner::smDisable(std::string& message) {
    // Pos Ctl
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);

    return true;
}

void MpcRunner::timerStartCountdownCallback() {
    if (start_countdown_counter_ == 0) {
        RCLCPP_WARN(get_logger(), "MPC Controller will start in...");
    }
    RCLCPP_WARN(get_logger(), "%ld...", get_parameter("start_countdown_limit").as_int() - start_countdown_counter_);

    if (start_countdown_counter_ >= get_parameter("start_countdown_limit").as_int()) {
        start_countdown_timer_->cancel();
        // one last check to go
        if (platform_motor_control_enabled_ && platform_armed_ && sm_active_phase_ == ENABLED) {
            controller_start_time_ = timestamp_.load();
            sm_active_phase_ = RUNNING;
        }
    }
    start_countdown_counter_++;
}

void MpcRunner::declareParameters() {
    // Node parameter
    declare_parameter<bool>("enable_controller", false);
    declare_parameter<double>("trajectory_initial_state_thres", 5.0);
    declare_parameter<int>("start_countdown_limit", 3);

    // Trajectory
    declare_parameter<std::string>("trajectory_config_path", "");
    declare_parameter<int>("trajectory_dt", 10);
    declare_parameter<std::string>("trajectory_solver", "");
    declare_parameter<std::string>("trajectory_integration", "");

    // Mpc Controller
    declare_parameter<std::string>("mpc_config_path", "");
    declare_parameter<std::string>("mpc_type", "");
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

bool MpcRunner::initializeMpcController(std::string& message) {
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

    Eigen::VectorXd state_diff = Eigen::VectorXd::Zero(trajectory_->get_robot_state()->get_ndx());
    trajectory_->get_robot_state()->diff(state_, trajectory_->get_initial_state(), state_diff);
    if (state_diff.norm() > get_parameter("trajectory_initial_state_thres").as_double()) {
        message = "Cannot enable MPC Controller. Current state too different from specified trajectory initial state.";
        return false;
    }

    solver->solve(crocoddyl::DEFAULT_VECTOR, crocoddyl::DEFAULT_VECTOR);

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

    return true;
}

void MpcRunner::computeControls() {
    if (sm_active_phase_ == RUNNING) {
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
    } else {
        actuator_normalized_ = -Eigen::VectorXd::Ones(actuator_normalized_.size());
    }
}

void MpcRunner::arm() const {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(get_logger(), "Arm command sent");
}

void MpcRunner::disablingProcedure() {
    RCLCPP_WARN(get_logger(), "MPC Controller. DISABLING");
    // Change to a safe flight mode

    // If all safety checks are correct, change flihgt mode, arm and start mission
    // running_controller_ = false;
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

}  // namespace eagle_mpc_ros2

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<eagle_mpc_ros2::MpcRunner> controller = std::make_shared<eagle_mpc_ros2::MpcRunner>("MpcRunner");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}