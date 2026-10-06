cmake_minimum_required(VERSION 3.22)

project(rbq_examples LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Wl,--gc-sections -s")

find_package(Eigen3 REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(onnxruntime REQUIRED)
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

get_target_property(ONNX_RUNTIME_LOCATION onnxruntime::onnxruntime IMPORTED_LOCATION_RELEASE)
get_filename_component(onnxruntime_so_basename "${ONNX_RUNTIME_LOCATION}" NAME)
install(FILES ${ONNX_RUNTIME_LOCATION}
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
install(CODE "
    set(BIN_DIR \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}\")
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink \"${onnxruntime_so_basename}\" libonnxruntime.so.1
        WORKING_DIRECTORY \"\${BIN_DIR}\")
")

add_executable(rbq_low_level src/rbq_low_level.cpp)
set_target_properties(rbq_low_level PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(rbq_low_level
    pthread
    rt
    Eigen3::Eigen
    onnxruntime::onnxruntime
    rbq_sdk::rbq_sdk
)
target_compile_definitions(rbq_low_level PRIVATE
    APP_NAME=\"rbq_low_level\"
)
install(TARGETS rbq_low_level
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(rbq_high_level src/rbq_high_level.cpp)
set_target_properties(rbq_high_level PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(rbq_high_level
    pthread
    rt
    Eigen3::Eigen
    rbq_sdk::rbq_sdk
)
target_compile_definitions(rbq_high_level PRIVATE
    APP_NAME=\"rbq_high_level\"
)
install(TARGETS rbq_high_level
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(rbq_sport_client_joy src/rbq_sport_client_joy.cpp)
set_target_properties(rbq_sport_client_joy PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(rbq_sport_client_joy
    pthread
    rt
    Eigen3::Eigen
    rbq_sdk::rbq_sdk
)
target_compile_definitions(rbq_sport_client_joy PRIVATE
    APP_NAME=\"rbq_sport_client_joy\"
)
install(TARGETS rbq_sport_client_joy
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(rbq_image_echo src/rbq_image_echo.cpp)
set_target_properties(rbq_image_echo PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(rbq_image_echo
    pthread
    rt
    rbq_sdk::rbq_sdk
)
target_compile_definitions(rbq_image_echo PRIVATE
    APP_NAME=\"rbq_image_echo\"
)
install(TARGETS rbq_image_echo
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(rbq_sport_client src/rbq_sport_client.cpp)
set_target_properties(rbq_sport_client PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(rbq_sport_client
    pthread
    rt
    Eigen3::Eigen
    rbq_sdk::rbq_sdk
)
target_compile_definitions(rbq_sport_client PRIVATE
    APP_NAME=\"rbq_sport_client\"
)
install(TARGETS rbq_sport_client
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(rbq_target_go src/rbq_target_go.cpp)
set_target_properties(rbq_target_go PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)
target_link_libraries(rbq_target_go
    pthread
    rt
    rbq_sdk::rbq_sdk
)
target_compile_definitions(rbq_target_go PRIVATE
    APP_NAME=\"rbq_target_go\"
)
install(TARGETS rbq_target_go
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
