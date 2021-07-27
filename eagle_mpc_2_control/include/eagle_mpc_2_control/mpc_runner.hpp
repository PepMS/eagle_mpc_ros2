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

#include <px4_msgs/msg/vehicle_command.hpp>

#include "eagle_mpc/trajectory.hpp"
#include "eagle_mpc/mpc-controllers/carrot-mpc.hpp"
#include "eagle_mpc/mpc-controllers/rail-mpc.hpp"
#include "eagle_mpc/mpc-controllers/weighted-mpc.hpp"
#include "eagle_mpc/mpc-base.hpp"
#include "eagle_mpc/utils/tools.hpp"

#include "eagle_mpc_2_control/controller_base.hpp"

class MpcRunner : public ControllerAbstract {
 public:
  explicit MpcRunner(const std::string& node_name);
  virtual ~MpcRunner();

 private:
  // ROS2-Node related objects

  bool running_controller_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;

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

  double motor_value_;
  void declareParameters();
  void dumpParameters();

  void enablingProcedure();
  void disablingProcedure();

  void initializeMpcController();
  virtual void computeControls();
  virtual void publishControls();
  
  void arm() const;
  void disarm() const;
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
  
  void computeControls2();


  void enableControllerCallback(const rclcpp::Parameter& p);
  virtual void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);

};

#endif
