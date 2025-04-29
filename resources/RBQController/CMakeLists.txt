project(RBQController)
set(target RBQController)

find_package(Qt5Widgets REQUIRED)

option(BUILD_${target} "Building a ${target}" ON)
if(BUILD_${target})
    add_cnoid_simple_controller(${target} SHARED RBQController.cpp)
    target_link_libraries(${target} Qt5::Widgets)
    target_include_directories(${target} PRIVATE ${Qt5Widgets_INCLUDE_DIRS})
endif()
