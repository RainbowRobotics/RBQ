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

# The quadruped examples.
add_subdirectory(src/rbq)
