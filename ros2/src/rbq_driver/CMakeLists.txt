cmake_minimum_required(VERSION 3.22)
project(rbq_driver)

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
find_package(rbq_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core)
find_package(cv_bridge REQUIRED)
# The header 'cv_bridge/cv_bridge.hpp' was added in version 3.3.0. For older
# cv_bridge versions, we have to use the header 'cv_bridge/cv_bridge.h'.
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

find_package(Qt5 COMPONENTS Core Network REQUIRED)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/Publisher.h
    src/Subscriber.h
    src/rbq_podo_api.h
    src/RBTypes.hpp
    src/NetworkHandler.h
    src/NetworkHandler.cpp
    src/RobotApiHandler.h
    src/RobotApiHandler.cpp
    src/ipc.h
    src/ipc.cpp
    src/VisionPublisher.h
)

target_include_directories(
    ${PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include/
            ${EIGEN3_INCLUDE_DIR}
            ${QT_INCLUDES}
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE ${OpenCV_INCLUDE_DIRS}
)  # add OpenCV includes to the included dirs

ament_target_dependencies(
    ${PROJECT_NAME}
        rclcpp std_msgs sensor_msgs rbq_msgs
        ${EIGEN3_LIBRARIES}
        cv_bridge
        nav_msgs tf2 tf2_ros
)

target_link_libraries(
    ${PROJECT_NAME}
        Qt5::Core
        Qt5::Network
)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
