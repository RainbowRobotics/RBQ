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

# These examples build against the SDK installed on this PC (/usr/local), not the one next to them. Stop here when that
# install differs from this checkout's SDK - otherwise it shows up only as "ArmClient.hpp: No such file" (#1441).
# Only the copied headers are compared; zn/ is installed only with zenoh. Skipped when the examples are built on their own.
get_filename_component(RBQ_SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rbq_sdk_cpp" ABSOLUTE)
if(EXISTS "${RBQ_SDK_SOURCE_DIR}/include/rbq_sdk.hpp")
    file(GLOB_RECURSE rbq_sdk_headers RELATIVE "${RBQ_SDK_SOURCE_DIR}/include" "${RBQ_SDK_SOURCE_DIR}/include/*.hpp")
    list(FILTER rbq_sdk_headers EXCLUDE REGEX "^zn/")
    set(rbq_sdk_stale "")
    foreach(header ${rbq_sdk_headers})
        set(installed "${RBQ_SDK_INCLUDE_DIR}/rbq_sdk/${header}")
        if(NOT EXISTS "${installed}")
            list(APPEND rbq_sdk_stale "${header} (missing)")
            continue()
        endif()
        file(SHA256 "${RBQ_SDK_SOURCE_DIR}/include/${header}" source_hash)
        file(SHA256 "${installed}" installed_hash)
        if(NOT source_hash STREQUAL installed_hash)
            list(APPEND rbq_sdk_stale "${header}")
        endif()
    endforeach()
    if(rbq_sdk_stale)
        list(JOIN rbq_sdk_stale "\n    " rbq_sdk_stale_text)
        message(FATAL_ERROR
            "The SDK installed in ${RBQ_SDK_INCLUDE_DIR} is not this checkout's SDK. These headers differ:\n"
            "    ${rbq_sdk_stale_text}\n"
            "Install this checkout's SDK, then build again:\n"
            "    cd ${RBQ_SDK_SOURCE_DIR} && sudo bash scripts/build.bash")
    endif()
endif()

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
# The RBA-1 arm examples.
add_subdirectory(src/arm)
