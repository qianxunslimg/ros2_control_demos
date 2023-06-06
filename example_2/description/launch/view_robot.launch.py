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


def generate_launch_description():
    # Declare arguments
    #参数的声明是使用DeclareLaunchArgument类来定义参数的名称、默认值和描述。
    # 通过声明参数，我们告诉Launch文件用户可以在运行时提供这些参数的值。
    # 声明参数是为了给用户提供可配置的选项，以便在启动节点时根据需要进行自定义设置。
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package", #描述机器人URDF/xacro文件的软件包名称
            default_value="ros2_control_demo_example_2",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file", #机器人的URDF/XACRO描述文件
            default_value="diffbot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",  #关节名称的前缀，对于多机器人设置很有用。默认为空字符串
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    #然后，通过LaunchConfiguration类初始化这些参数，以便在后面的节点配置中使用。
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    # 这个命令的目的是使用 xacro 工具来处理 URDF 文件，
    # 将其中的参数替换成具体的值，并生成最终的 URDF 内容。
    # prefix 参数用于给机器人的关节名称添加一个前缀，以实现多机器人设置时的区分。
    # xacro [xacro可执行文件路径] [URDF文件路径] prefix:= [prefix参数的值]
    
    # [xacro可执行文件路径] 是通过 FindExecutable 查找到的 xacro 可执行文件的路径。
    # [URDF文件路径] 是通过 FindPackageShare 查找到的机器人描述软件包的共享路径下的 urdf 目录中的 description_file 参数的值所对应的 URDF 文件的路径。
    # prefix:=[prefix参数的值] 是一个命令行参数，表示指定的 prefix 参数的值。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "diffbot_view.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
