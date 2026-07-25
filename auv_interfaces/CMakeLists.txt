cmake_minimum_required(VERSION 3.8)
project(auv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake: the ROS 2 build system wrapper around CMake.
find_package(ament_cmake REQUIRED)

# rosidl: the interface code generator. Turns .msg files into C++/Python.
find_package(rosidl_default_generators REQUIRED)

# std_msgs: needed because ActuatorCommand embeds std_msgs/Header.
find_package(std_msgs REQUIRED)

# The core of this file: generate code for our interfaces.
# Every new .msg, .srv, or .action must be added to this list, or it silently
# will not exist. This is the single most common interface-package mistake.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Downstream packages that use our messages need the rosidl runtime.
ament_export_dependencies(rosidl_default_runtime)

# Must be the last call. Installs package metadata so ROS can find us.
ament_package()