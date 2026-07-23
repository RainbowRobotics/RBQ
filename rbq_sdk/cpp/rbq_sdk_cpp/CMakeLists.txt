cmake_minimum_required(VERSION 3.22)

project(rbq_sdk LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Wl,--gc-sections -s")

include(GNUInstallDirs)

find_package(CycloneDDS-CXX REQUIRED)
find_package(CycloneDDS REQUIRED)
find_program(IDLPP_EXECUTABLE idlc REQUIRED)
set(IDLC_ENV "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:CycloneDDS-CXX::idlcxx>:$<TARGET_FILE_DIR:CycloneDDS::ddsc>")

set(IDL_FINAL_DIR ${CMAKE_CURRENT_BINARY_DIR}/idl)
set(IDL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/idl)

file(MAKE_DIRECTORY ${IDL_FINAL_DIR}/ros2)
set(ROS2_IDL_FILES
    # std_msgs
    ${IDL_SOURCE_DIR}/ros2/Bool_.idl
    ${IDL_SOURCE_DIR}/ros2/Char_.idl
    ${IDL_SOURCE_DIR}/ros2/Float_.idl
    ${IDL_SOURCE_DIR}/ros2/MultiArrayDimension_.idl
    ${IDL_SOURCE_DIR}/ros2/MultiArrayLayout_.idl
    ${IDL_SOURCE_DIR}/ros2/Float64MultiArray_.idl
    ${IDL_SOURCE_DIR}/ros2/Int8_.idl
    ${IDL_SOURCE_DIR}/ros2/Int8MultiArray_.idl
    ${IDL_SOURCE_DIR}/ros2/String_.idl
    ${IDL_SOURCE_DIR}/ros2/Time_.idl
    ${IDL_SOURCE_DIR}/ros2/Header_.idl
    # geometry_msgs
    ${IDL_SOURCE_DIR}/ros2/Point_.idl
    ${IDL_SOURCE_DIR}/ros2/Point32_.idl
    ${IDL_SOURCE_DIR}/ros2/PointStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/Quaternion_.idl
    ${IDL_SOURCE_DIR}/ros2/QuaternionStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/Pose_.idl
    ${IDL_SOURCE_DIR}/ros2/PoseStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/PoseWithCovariance_.idl
    ${IDL_SOURCE_DIR}/ros2/PoseWithCovarianceStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/Pose2D_.idl
    ${IDL_SOURCE_DIR}/ros2/Vector3_.idl
    ${IDL_SOURCE_DIR}/ros2/Twist_.idl
    ${IDL_SOURCE_DIR}/ros2/TwistStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/TwistWithCovariance_.idl
    ${IDL_SOURCE_DIR}/ros2/TwistWithCovarianceStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/Transform_.idl
    ${IDL_SOURCE_DIR}/ros2/TransformStamped_.idl
    ${IDL_SOURCE_DIR}/ros2/TFMessage_.idl
    # sensor_msgs
    ${IDL_SOURCE_DIR}/ros2/Imu_.idl
    ${IDL_SOURCE_DIR}/ros2/Joy_.idl
    ${IDL_SOURCE_DIR}/ros2/JointState_.idl
    ${IDL_SOURCE_DIR}/ros2/PointField_.idl
    ${IDL_SOURCE_DIR}/ros2/PointCloud2_.idl
    ${IDL_SOURCE_DIR}/ros2/RegionOfInterest_.idl
    ${IDL_SOURCE_DIR}/ros2/CameraInfo_.idl
    ${IDL_SOURCE_DIR}/ros2/CompressedImage_.idl
    # nav_msgs
    ${IDL_SOURCE_DIR}/ros2/Odometry_.idl
    ${IDL_SOURCE_DIR}/ros2/MapMetaData_.idl
    ${IDL_SOURCE_DIR}/ros2/OccupancyGrid_.idl
)
foreach(idl_file ${ROS2_IDL_FILES})
    get_filename_component(idl_name ${idl_file} NAME_WE)
    file(SHA256 ${idl_file} idl_hash)
    set(hash_file ${IDL_FINAL_DIR}/ros2/${idl_name}.idl.hash)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/idl_template.hash.in
        ${hash_file}
        @ONLY
    )
    set(gen_file ${IDL_FINAL_DIR}/ros2/${idl_name}.hpp ${IDL_FINAL_DIR}/ros2/${idl_name}.cpp)
    add_custom_command(
        OUTPUT ${gen_file}
        COMMAND ${CMAKE_COMMAND} -E env ${IDLC_ENV} ${IDLPP_EXECUTABLE} -l cxx -I ${IDL_SOURCE_DIR}/ros2 -x final ${idl_file}
        WORKING_DIRECTORY ${IDL_FINAL_DIR}/ros2
        DEPENDS ${idl_file} ${hash_file}
        COMMENT "Generating C++ from ${idl_file}"
        VERBATIM
    )
    list(APPEND SOURCES ${gen_file})
endforeach()

set(IDL_DIR api)
file(MAKE_DIRECTORY ${IDL_FINAL_DIR}/${IDL_DIR})
set(IDL_FILES
    ${IDL_SOURCE_DIR}/${IDL_DIR}/RequestLease_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/RequestPolicy_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/RequestIdentity_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/RequestHeader_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/Request_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/ResponseStatus_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/ResponseHeader_.idl
    ${IDL_SOURCE_DIR}/${IDL_DIR}/Response_.idl
)
foreach(idl_file ${IDL_FILES})
    get_filename_component(idl_name ${idl_file} NAME_WE)
    file(SHA256 ${idl_file} idl_hash)
    set(hash_file ${IDL_FINAL_DIR}/${IDL_DIR}/${idl_name}.idl.hash)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/idl_template.hash.in
        ${hash_file}
        @ONLY
    )
    set(gen_file ${IDL_FINAL_DIR}/${IDL_DIR}/${idl_name}.hpp ${IDL_FINAL_DIR}/${IDL_DIR}/${idl_name}.cpp)
    add_custom_command(
        OUTPUT ${gen_file}
        COMMAND ${CMAKE_COMMAND} -E env ${IDLC_ENV} ${IDLPP_EXECUTABLE} -l cxx -I ${IDL_SOURCE_DIR}/${IDL_DIR} -x final ${idl_file}
        WORKING_DIRECTORY ${IDL_FINAL_DIR}/${IDL_DIR}
        DEPENDS ${idl_file} ${hash_file}
        COMMENT "Generating C++ from ${idl_file}"
        VERBATIM
    )
    list(APPEND SOURCES ${gen_file})
endforeach()

file(MAKE_DIRECTORY ${IDL_FINAL_DIR}/rbq)
set(IDL_FILES
    ${IDL_SOURCE_DIR}/rbq/RbqConstants_.idl
    ${IDL_SOURCE_DIR}/rbq/JointOwnershipCmd_.idl
    ${IDL_SOURCE_DIR}/rbq/JointInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/LegJointInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/WhlJointInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/ArmJointInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/JointRef_.idl
    ${IDL_SOURCE_DIR}/rbq/TaskInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/ContactInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/SimInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/MotionRef_.idl
    ${IDL_SOURCE_DIR}/rbq/PduInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/MotorStatus_.idl
    ${IDL_SOURCE_DIR}/rbq/MotorInfo_.idl
    ${IDL_SOURCE_DIR}/rbq/RobotParameters_.idl
    ${IDL_SOURCE_DIR}/rbq/BatteryState_.idl
    ${IDL_SOURCE_DIR}/rbq/FootStates_.idl
    ${IDL_SOURCE_DIR}/rbq/HighLevelCommand_.idl
    ${IDL_SOURCE_DIR}/rbq/RobotStatus_.idl
    ${IDL_SOURCE_DIR}/rbq/UserCommand_.idl
    ${IDL_SOURCE_DIR}/rbq/SystemLog_.idl
    ${IDL_SOURCE_DIR}/rbq/TargetGo_.idl
)
foreach(idl_file ${IDL_FILES})
    get_filename_component(idl_name ${idl_file} NAME_WE)
    file(SHA256 ${idl_file} idl_hash)
    set(hash_file ${IDL_FINAL_DIR}/rbq/${idl_name}.idl.hash)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/idl_template.hash.in
        ${hash_file}
        @ONLY
    )
    set(gen_file ${IDL_FINAL_DIR}/rbq/${idl_name}.hpp ${IDL_FINAL_DIR}/rbq/${idl_name}.cpp)
    add_custom_command(
        OUTPUT ${gen_file}
        COMMAND ${CMAKE_COMMAND} -E env ${IDLC_ENV} ${IDLPP_EXECUTABLE} -l cxx -I ${IDL_SOURCE_DIR}/rbq -x final ${idl_file}
        WORKING_DIRECTORY ${IDL_FINAL_DIR}/rbq
        DEPENDS ${idl_file} ${hash_file}
        COMMENT "Generating C++ from ${idl_file}"
        VERBATIM
    )
    list(APPEND SOURCES ${gen_file})
endforeach()

file(MAKE_DIRECTORY ${IDL_FINAL_DIR}/go2)
set(IDL_FILES
    ${IDL_SOURCE_DIR}/go2/AudioData_.idl
    ${IDL_SOURCE_DIR}/go2/BmsCmd_.idl
    ${IDL_SOURCE_DIR}/go2/BmsState_.idl
    ${IDL_SOURCE_DIR}/go2/ConfigChangeStatus_.idl
    ${IDL_SOURCE_DIR}/go2/Error_.idl
    ${IDL_SOURCE_DIR}/go2/Go2FrontVideoData_.idl
    ${IDL_SOURCE_DIR}/go2/HeightMap_.idl
    ${IDL_SOURCE_DIR}/go2/IMUState_.idl
    ${IDL_SOURCE_DIR}/go2/InterfaceConfig_.idl
    ${IDL_SOURCE_DIR}/go2/LidarState_.idl
    ${IDL_SOURCE_DIR}/go2/LowCmd_.idl
    ${IDL_SOURCE_DIR}/go2/LowState_.idl
    ${IDL_SOURCE_DIR}/go2/MotorCmd_.idl
    ${IDL_SOURCE_DIR}/go2/MotorCmds_.idl
    ${IDL_SOURCE_DIR}/go2/MotorState_.idl
    ${IDL_SOURCE_DIR}/go2/MotorStates_.idl
    ${IDL_SOURCE_DIR}/go2/PathPoint_.idl
    ${IDL_SOURCE_DIR}/go2/Req_.idl
    ${IDL_SOURCE_DIR}/go2/Res_.idl
    ${IDL_SOURCE_DIR}/go2/SportModeCmd_.idl
    ${IDL_SOURCE_DIR}/go2/SportModeState_.idl
    ${IDL_SOURCE_DIR}/go2/TimeSpec_.idl
    ${IDL_SOURCE_DIR}/go2/UwbState_.idl
    ${IDL_SOURCE_DIR}/go2/UwbSwitch_.idl
    ${IDL_SOURCE_DIR}/go2/VoxelMapCompressed_.idl
    ${IDL_SOURCE_DIR}/go2/WirelessController_.idl
)
foreach(idl_file ${IDL_FILES})
    get_filename_component(idl_name ${idl_file} NAME_WE)
    file(SHA256 ${idl_file} idl_hash)
    set(hash_file ${IDL_FINAL_DIR}/go2/${idl_name}.idl.hash)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/idl_template.hash.in
        ${hash_file}
        @ONLY
    )
    set(gen_file ${IDL_FINAL_DIR}/go2/${idl_name}.hpp ${IDL_FINAL_DIR}/go2/${idl_name}.cpp)
    add_custom_command(
        OUTPUT ${gen_file}
        COMMAND ${CMAKE_COMMAND} -E env ${IDLC_ENV} ${IDLPP_EXECUTABLE} -l cxx -I ${IDL_SOURCE_DIR}/go2 -x final ${idl_file}
        WORKING_DIRECTORY ${IDL_FINAL_DIR}/go2
        DEPENDS ${idl_file} ${hash_file}
        COMMENT "Generating C++ from ${idl_file}"
        VERBATIM
    )
    list(APPEND SOURCES ${gen_file})
endforeach()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_library("${PROJECT_NAME}" SHARED ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rbq_sdk.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Filter.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/JointControl.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Joystick.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Policy.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Thread.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Timer.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dds/ChannelFactory.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dds/Publisher.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dds/Settings.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dds/SportClient.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dds/Subscriber.hpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    CycloneDDS-CXX::ddscxx
)

# Prepare share
if(WIN32 AND NOT CYGWIN AND NOT ${PROJECT_NAME}_VCPKG_BUILD)
    set(CMAKE_SHARE_DIR cmake)
else()
    set(CMAKE_SHARE_DIR lib/cmake/${PROJECT_NAME})
endif()
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_SHARE_DIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_dds.bash
    DESTINATION ${CMAKE_SHARE_DIR}
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION ${CMAKE_SHARE_DIR}
)
install(DIRECTORY ${IDL_FINAL_DIR}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
