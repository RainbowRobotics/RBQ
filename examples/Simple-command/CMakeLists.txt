cmake_minimum_required(VERSION 3.22)

project(Simple-command LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Wl,--gc-sections -s")

find_package(Eigen3 REQUIRED)
find_package(RBQ REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${RBQ_LIBRARY}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${RBQ_INCLUDE_DIR}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    APP_NAME=\"${PROJECT_NAME}\"
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
