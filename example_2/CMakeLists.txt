cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ros2_control_demo_example_2
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(ros2_control_demo_example_2 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_2>
)
ament_target_dependencies(
  ros2_control_demo_example_2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_2.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_2
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/ros2_control_demo_example_2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_2
)
install(TARGETS ros2_control_demo_example_2
  EXPORT export_ros2_control_demo_example_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()



# 这个CMake文件的目的是为了构建和安装ros2_control_demo_example_2项目。以下是该文件的主要功能：

# cmake_minimum_required(VERSION 3.16)：指定CMake的最低版本要求为3.16。

# project(ros2_control_demo_example_2 LANGUAGES CXX)：定义项目名称为ros2_control_demo_example_2，使用的编程语言为C++。

# if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")：如果C++编译器为GNU或Clang，则添加编译选项-Wall -Wextra，用于启用更严格的警告。

# set(THIS_PACKAGE_INCLUDE_DEPENDS ...)：定义项目所依赖的包。

# find_package(ament_cmake REQUIRED)和find_package(${Dependency} REQUIRED)：查找并加载所需的依赖包。

# add_library(ros2_control_demo_example_2 SHARED ...)：定义名为ros2_control_demo_example_2的共享库，该库包含hardware/diffbot_system.cpp文件的源代码。

# target_compile_features(ros2_control_demo_example_2 PUBLIC cxx_std_17)：指定ros2_control_demo_example_2库使用C++17标准。

# target_include_directories(ros2_control_demo_example_2 ...)：设置ros2_control_demo_example_2库的头文件搜索路径。

# ament_target_dependencies(ros2_control_demo_example_2 ...)：指定ros2_control_demo_example_2库所依赖的其他库。

# target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")：定义一个编译宏，用于标识正在构建该项目的动态链接库。

# pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_2.xml)：导出硬件插件的描述文件。

# install(...)：安装项目的文件和目录到指定的位置，包括头文件、启动文件、配置文件和库文件。

# if(BUILD_TESTING)和find_package(ament_cmake_gtest REQUIRED)：如果启用了测试构建，则查找测试所需的依赖包。

# ament_export_targets(export_ros2_control_demo_example_2 HAS_LIBRARY_TARGET)和ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})：导出项目的构建目标和依赖关系。

# ament_package()：生成一个ament_package配置文件，用于描述项目的元信息和依赖关系。

# 总体而言，该CMake文件负责设置项目的编译选项、依赖关系、目标构建、安装规则和导出配置，以便于构建、安装和使用ros2_control_demo_example_2项目。