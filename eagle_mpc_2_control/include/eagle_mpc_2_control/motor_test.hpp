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

#ifndef EAGLE_MPC_2_CONTROL_MOTOR_TEST_HPP
#define EAGLE_MPC_2_CONTROL_MOTOR_TEST_HPP

#include "eagle_mpc_2_control/controller_base.hpp"

class MotorTest : public ControllerAbstract
{
public:
  explicit MotorTest(const std::string& node_name);
  virtual ~MotorTest();

  // virtual void timerComputeControlsCallback() override;

private:
  virtual void vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);


  virtual void computeControls();
  virtual void publishControls();

  void changeMotorCallback();

  rclcpp::TimerBase::SharedPtr change_motor_timer_;

  std::size_t counter_;
  std::size_t motor_idx_;

  double motor_value_;
};

#endif
