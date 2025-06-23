cmake_minimum_required(VERSION 3.22)
project(rbq_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rbq_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core)
find_package(cv_bridge REQUIRED)
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

add_executable(depth_to_pcl src/depth_to_pcl.cpp)
ament_target_dependencies(depth_to_pcl rclcpp sensor_msgs cv_bridge pcl_conversions pcl_ros OpenCV)

install(TARGETS
    depth_to_pcl
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
