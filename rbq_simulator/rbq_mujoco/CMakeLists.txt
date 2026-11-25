cmake_minimum_required(VERSION 3.22)

project(rbq_mujoco LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Wl,--gc-sections")

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mujoco REQUIRED CONFIG)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

find_package(rbq_sdk REQUIRED)
add_link_options(-Wl,--disable-new-dtags)
get_target_property(DDSC_RUNTIME_LOCATION CycloneDDS::ddsc IMPORTED_LOCATION_RELEASE)
get_target_property(DDSCXX_RUNTIME_LOCATION CycloneDDS-CXX::ddscxx IMPORTED_LOCATION_RELEASE)
install(FILES ${DDSCXX_RUNTIME_LOCATION} ${DDSC_RUNTIME_LOCATION}
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
get_filename_component(ddsc_filename ${DDSC_RUNTIME_LOCATION} NAME)
get_filename_component(ddscxx_filename ${DDSCXX_RUNTIME_LOCATION} NAME)
install(CODE "
    set(BIN_DIR \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}\")
    file(RENAME \"\${BIN_DIR}/${ddsc_filename}\" \"\${BIN_DIR}/libddsc.so\")
    file(RENAME \"\${BIN_DIR}/${ddscxx_filename}\" \"\${BIN_DIR}/libddscxx.so\")
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink libddsc.so libddsc.so.0
        WORKING_DIRECTORY \"\${BIN_DIR}\")
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink libddscxx.so libddscxx.so.0
        WORKING_DIRECTORY \"\${BIN_DIR}\")
")
get_target_property(RBQ_SDK_RUNTIME_LOCATION rbq_sdk::rbq_sdk IMPORTED_LOCATION_RELEASE)
install(FILES ${RBQ_SDK_RUNTIME_LOCATION}
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/utils/VisionPublisher.cpp
    src/utils/MjLidar.cpp
    src/utils/LidarPublisher.cpp
    src/utils/simulate.cc
    src/gl/platform_ui_adapter.cc
    src/gl/glfw_adapter.cc
    src/gl/glfw_dispatch.cc
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(${PROJECT_NAME}
    glfw
    OpenGL::GL
    m pthread dl
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    Eigen3::Eigen
    rbq_sdk::rbq_sdk
    mujoco::mujoco
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
