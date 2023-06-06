# ros2_control_demo_example_1

   *RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html).

# Example 1: RRBot

RRBot，或称为“Revolute-Revolute Manipulator Robot”，是一个简单的3节链接、2个关节的机械臂，我们将使用它来演示各种功能。

它本质上是一个双倒立摆，展示了一些有趣的控制概念，最初是为Gazebo教程而引入的。

在example_1中，实现了仅具有一个接口的硬件接口插件。

- 通信使用专有API与机器人控制箱进行通信。
- 所有关节的数据一次性交换。
- 示例：KUKA RSI

RRBot的URDF文件可以在description/urdf文件夹中找到。

教程步骤
使用以下启动命令检查RRBot描述是否正常工作：

```
ros2 launch ros2_control_demo_example_1 view_robot.launch.py
```

注意： 在终端中获得以下输出是正常的：“Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist”。这是因为joint_state_publisher_gui节点需要一些时间来启动。

joint_state_publisher_gui提供了一个GUI来更改RRbot的配置。它会立即显示在RViz中。

Revolute-Revolute Manipulator Robot

可以按照以下步骤重新创建RViz设置：

使用RobotModel显示器使用/robot_description主题可视化机器人模型。
或者您可以手动打开description/rviz文件夹中的配置，或者直接执行以下命令：
shell
Copy code
rviz2 --display-config `ros2 pkg prefix ros2_control_demo_example_1`/share/ros2_control_demo_example_1/rviz/rrbot.rviz
要启动RRBot示例，打开一个终端，进入您的ROS2工作空间并执行其启动文件：

shell
Copy code
ros2 launch ros2_control_demo_example_1 rrbot.launch.py
启动文件会加载并启动机器人硬件、控制器，并打开RViz。在起始终端中，您将看到来自硬件实现的大量输出，显示其内部状态。这仅供示例目的，应在硬件接口实现中尽量避免。

如果您在RViz中看到两个橙色矩形和一个黄色矩形，表示一切正常启动。但是为了确保，让我们在移动RRBot之前先检查控制系统。

通过打开另一个终端并执行以下命令，检查硬件接口是否正确加载：

shell
Copy code
ros2 control list_hardware_interfaces
您应该看到以下输出：

arduino
Copy code
hardware_interface::PositionJointInterface
这表示RRBot硬件接口已正确加载。

现在，我们将使用RRBot的控制器来移动机器人。通过在新的终端中执行以下命令，启动控制器：

shell
Copy code
ros2 control list_controllers
您应该看到以下输出：

Copy code
position_controller
这表示RRBot的位置控制器已经启动。

最后，我们将发布一个目标位置给RRBot的位置控制器，以使机器人移动到指定的位置。在新的终端中执行以下命令：

shell
Copy code
ros2 topic pub /rrbot/joint1_position_controller/command std_msgs/msg/Float64 "data: 1.0"
这将向RRBot的位置控制器发布一个目标位置值。机器人将移动到1.0弧度的位置。

您还可以尝试发布其他目标位置值，以使机器人移动到不同的位置。

这些步骤将引导您通过演示RRBot示例来了解如何使用ros2_control框架。您可以根据需要进行修改和扩展以适应您自己的机器人。
