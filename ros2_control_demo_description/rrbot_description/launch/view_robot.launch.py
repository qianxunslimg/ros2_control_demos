# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# the package name is "rrbot_description"
def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rrbot_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rrbot.urdf.xacro",
            #default_value="rrbot_system_position_only.ros2_control.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # 在这段代码中，description_package 是一个 LaunchConfiguration 对象，
            # 但是 FindPackageShare 函数期望接收一个字符串作为参数。所以为了将 description_package 转换为字符串，
            # 它被传递给了 FindPackageShare 函数的参数列表中。
            # 在这种情况下，description_package 对象会在运行时解析为实际的值，
            # 然后作为字符串传递给 FindPackageShare 函数。
            # 这是因为 PathJoinSubstitution 期望接收字符串参数，而 LaunchConfiguration 对象在执行时会返回参数的实际值。
            # 因此，尽管 description_package 是一个 LaunchConfiguration 对象，
            # 但在这个上下文中，它会被解析为字符串，以满足 FindPackageShare 函数的要求。
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            # the default param is string
            # PathJoinSubstitution(
            #     [FindPackageShare("rrbot_description"), "urdf", description_file]
            # ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "rrbot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # publish the urdf model

    # parameters用于在节点中传递静态的参数配置，并且可以在节点运行时进行修改。
    # 这些参数可以通过参数服务器进行设置，或者在节点启动时进行配置。
    # parameters可以包含任何类型的数据，并且可以在节点运行时动态修改。
    # 节点可以使用这些参数来配置其行为、调整算法参数等。

    # arguments用于在节点启动时传递命令行参数，它们是在每次运行节点时通过命令行传递的。
    # 这些参数通常用于传递临时的、影响节点行为的参数，例如标志参数、文件路径、调试选项等。
    # 命令行参数在节点启动时传递，不能在节点运行过程中动态修改。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],  # a command which return a urdf model
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file], # int argc, char** argv
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

# to debug
if __name__ == "__main__":
    generate_launch_description()