// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ros2_control_demo_example_2/visibility_control.h"

namespace ros2_control_demo_example_2
{

/*
hardware_interface::SystemInterface 是 ROS 2 中的一个接口类，用于定义机器人硬件接口的系统级别功能。

hardware_interface::SystemInterface
定义了与硬件交互的基本功能，包括初始化、激活、读取状态和写入命令等。这个接口类是 ROS 2
控制系统中硬件接口的核心组成部分，它提供了一种标准化的方式来定义和实现与硬件通信的接口。

下面是 hardware_interface::SystemInterface 接口类的主要成员函数：

hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo
&info)：在初始化硬件接口时调用的回调函数。该函数用于执行硬件接口的初始化操作，并返回初始化的结果。

std::vector<hardware_interface::StateInterface>
export_state_interfaces()：导出机器人硬件的状态接口。该函数返回一个包含所有状态接口的向量，每个接口包含状态名称、接口类型和指向状态数据的指针。

std::vector<hardware_interface::CommandInterface>
export_command_interfaces()：导出机器人硬件的命令接口。该函数返回一个包含所有命令接口的向量，每个接口包含命令名称、接口类型和指向命令数据的指针。

hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State
&previous_state)：在激活硬件接口时调用的回调函数。该函数用于执行硬件接口的激活操作，并返回激活的结果。

hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State
&previous_state)：在停用硬件接口时调用的回调函数。该函数用于执行硬件接口的停用操作，并返回停用的结果。

hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration
&period)：读取硬件状态的函数。该函数在每个控制周期调用，用于读取硬件的状态信息，并更新状态数据。

hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration
&period)：写入命令到硬件的函数。该函数在每个控制周期调用，用于将命令数据写入到硬件接口，控制机器人的运动。

hardware_interface::SystemInterface
是一个抽象的接口类，用于定义自定义机器人硬件接口的行为。在实际使用中，需要创建一个继承自
hardware_interface::SystemInterface 的子类，并实现其中的成员函数来定义特定硬件接口的行为逻辑。
*/
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace ros2_control_demo_example_2

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
