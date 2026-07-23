cmake_minimum_required(VERSION 3.22)
project(sensor_bridge NONE)

find_package(ament_cmake REQUIRED)

# Rviz 
install(DIRECTORY config launch rviz
    DESTINATION share/${PROJECT_NAME}
)

# CycloneDDS xml ships in config/ (installed via the DIRECTORY rule above).

# Robot_description URDF (resources live at the package root: ros2/src/sensor_bridge -> ../../../)
set(_MODELS_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../../resources/models)
set(_PKG_MESHES ${CMAKE_CURRENT_SOURCE_DIR}/../../../resources/meshes)
foreach(_pair "rbq10:rbq")
    string(REPLACE ":" ";" _vp "${_pair}")
    list(GET _vp 0 _variant)
    list(GET _vp 1 _file)
    file(READ ${_MODELS_DIR}/${_variant}/${_file}.urdf _U)
    string(REPLACE "model://" "package://${PROJECT_NAME}/models/" _U "${_U}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${_variant}.urdf "${_U}")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${_variant}.urdf
        DESTINATION share/${PROJECT_NAME}/urdf)
endforeach()

# Mesh dirs
install(DIRECTORY ${_PKG_MESHES}/rbq DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY ${_PKG_MESHES}/sensor DESTINATION share/${PROJECT_NAME}/models)

ament_package()
