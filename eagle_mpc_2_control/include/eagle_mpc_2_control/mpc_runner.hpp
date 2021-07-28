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

#ifndef EAGLE_MPC_2_CONTROL_MPC_RUNNER_HPP
#define EAGLE_MPC_2_CONTROL_MPC_RUNNER_HPP

#include <pinocchio/multibody/model.hpp>

#include <map>
#include <string>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "eagle_mpc/trajectory.hpp"
#include "eagle_mpc/mpc-controllers/carrot-mpc.hpp"
#include "eagle_mpc/mpc-controllers/rail-mpc.hpp"
#include "eagle_mpc/mpc-controllers/weighted-mpc.hpp"
#include "eagle_mpc/mpc-base.hpp"
#include "eagle_mpc/utils/tools.hpp"

#include "eagle_mpc_2_control/controller_base.hpp"
#include "eagle_mpc_2_interfaces/srv/mpc_controller_transition.hpp"

namespace eagle_mpc_ros2 {

enum sm_states { IDLE, ENABLED, RUNNING };
enum sm_transitions { ENABLE, START, DISABLE };

const double TRAJECTORY_INITIAL_STATE_THRES = 0.5;

class MpcRunner : public ControllerAbstract {
    public:
    explicit MpcRunner(const std::string& node_name);
    virtual ~MpcRunner();

    private:
    // ROS2-Node related objects
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;
    rclcpp::Service<eagle_mpc_2_interfaces::srv::MpcControllerTransition>::SharedPtr service_sm_transition_;

    rclcpp::TimerBase::SharedPtr start_countdown_timer_;
    int start_countdown_counter_;

    // MPC Related Objects
    struct NodeParams {
        std::string trajectory_config_path;
        std::size_t trajectory_dt;
        eagle_mpc::SolverTypes trajectory_solver;
        std::string trajectory_integration;
        bool trajectory_squash;

        std::string mpc_config_path;
        std::string mpc_type;

    } node_params_;

    boost::shared_ptr<eagle_mpc::Trajectory> trajectory_;
    boost::shared_ptr<eagle_mpc::MpcAbstract> mpc_controller_;

    uint64_t controller_start_time_;
    uint64_t controller_time_;
    uint64_t controller_instant_;

    Eigen::VectorXd control_command_;
    Eigen::VectorXd thrust_command_;
    
    // State machine objects
    int sm_active_phase_;
    static const std::map<std::string, int> SmTransitions;
    static const std::map<int, std::string> SmStates;

    // ROS2 Callbacks & Methods
    void transitionRequest(const std::shared_ptr<eagle_mpc_2_interfaces::srv::MpcControllerTransition::Request> request,
                           const std::shared_ptr<eagle_mpc_2_interfaces::srv::MpcControllerTransition::Response> response);
    virtual void handleVehicleCtrlMode(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) override;
    
    void declareParameters();
    void dumpParameters();
    void timerStartCountdownCallback();

    // MPC Related methods
    bool initializeMpcController(std::string& message);
    virtual void computeControls();


    // State machine methods
    bool smEnable(std::string& message);
    bool smStart(std::string& message);
    bool smDisable(std::string& message);

    static std::map<std::string, int> createSmTransitionsMap() {
        std::map<std::string, int> m;
        m["enable"] = ENABLE;
        m["start"] = START;
        m["disable"] = DISABLE;

        return m;
    }

    static std::map<int, std::string> createSmStatesMap() {
        std::map<int, std::string> m;
        m[IDLE] = "Idle";
        m[ENABLED] = "Enabled";
        m[RUNNING] = "Running";

        return m;
    }

    // PX4-related methods
    void arm() const;
    void disarm() const;
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;

};
}  // namespace eagle_mpc_ros2
#endif
