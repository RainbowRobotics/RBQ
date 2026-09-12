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

# The LiDAR fragments carry their link + fixed joint after the @SENSOR@ marker (robot level).
# Gazebo splices them at spawn (src/main.cpp); RViz reads the URDF generated here, so without the
# same splice the robot shows up in RViz with no LiDAR on it.
function(_lidar_links _frag _out)
    set(_V "")
    if(EXISTS ${_frag})
        file(READ ${_frag} _F)
        string(FIND "${_F}" "<!-- @SENSOR@ -->" _sp)
        if(_sp GREATER -1)
            math(EXPR _vs "${_sp} + 17")            # strlen("<!-- @SENSOR@ -->")
            string(SUBSTRING "${_F}" ${_vs} -1 _V)
        endif()
    endif()
    set(${_out} "${_V}" PARENT_SCOPE)
endfunction()

# Replace a whole marker comment (their text contains '>', so a regex would misfire).
function(_splice_marker _urdf _marker _content _out)
    set(_U "${_urdf}")
    string(FIND "${_U}" "${_marker}" _mp)
    if(_mp GREATER -1)
        string(SUBSTRING "${_U}" ${_mp} -1 _TAIL)
        string(FIND "${_TAIL}" "-->" _ep)
        if(_ep GREATER -1)
            math(EXPR _end "${_mp} + ${_ep} + 3")
            string(SUBSTRING "${_U}" 0 ${_mp} _HEAD)
            string(SUBSTRING "${_U}" ${_end} -1 _REST)
            set(_U "${_HEAD}${_content}${_REST}")
        endif()
    endif()
    set(${_out} "${_U}" PARENT_SCOPE)
endfunction()

foreach(_pair "rbq10:rbq" "rbq10_wheel:rbq")
    string(REPLACE ":" ";" _vp "${_pair}")
    list(GET _vp 0 _variant)
    list(GET _vp 1 _file)
    file(READ ${_MODELS_DIR}/${_variant}/${_file}.urdf _RAW)
    # One URDF per LiDAR build; the launch picks by its lidar:= arg (none | livox | ouster).
    foreach(_lidar "none" "livox" "ouster")
        set(_LINKS "")
        set(_suffix "_${_lidar}")
        if(_lidar STREQUAL "none")
            set(_suffix "")
        else()
            _lidar_links(${_MODELS_DIR}/sensor/${_lidar}.urdf _LINKS)
        endif()
        # Both markers: the sensor links land at robot level, and the in-trunk visual marker is
        # dropped so no stray comment survives.
        _splice_marker("${_RAW}" "<!-- @SENSOR_VISUAL@" "" _U)
        _splice_marker("${_U}" "<!-- @SENSORS@" "${_LINKS}" _U)
        # After the splice, so the LiDAR mesh URIs get rewritten too.
        string(REPLACE "model://" "package://${PROJECT_NAME}/models/" _U "${_U}")
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${_variant}${_suffix}.urdf "${_U}")
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${_variant}${_suffix}.urdf
            DESTINATION share/${PROJECT_NAME}/urdf)
    endforeach()
endforeach()

# Mesh dirs
install(DIRECTORY ${_PKG_MESHES}/rbq DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY ${_PKG_MESHES}/sensor DESTINATION share/${PROJECT_NAME}/models)

ament_package()
