cmake_minimum_required(VERSION 3.5)
project(rbq_msgs)

if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/HighLevelCommand.msg"
    "msg/RobotState.msg"
    "msg/BatteryState.msg"
    "msg/FootState.msg"
    "msg/FootStates.msg"
    DEPENDENCIES
        builtin_interfaces
        common_interfaces
        geometry_msgs
        std_msgs
        sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
