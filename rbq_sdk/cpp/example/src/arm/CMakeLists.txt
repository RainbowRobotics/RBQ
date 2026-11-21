# RBA-1 arm examples, numbered in the order to try them (like RB-Y1's rby1-sdk examples).
# my_arm is yours to change - add your own programs to this list the same way. arm_cli is a test tool.
set(ARM_EXAMPLES
    00_arm_help
    01_arm_state
    02_arm_power
    03_arm_gripper
    04_arm_move_j
    05_arm_move_l
    06_arm_high_level
    07_arm_low_level
    08_arm_torque
    my_arm
    arm_cli
)

foreach(example IN LISTS ARM_EXAMPLES)
    add_executable(${example} ${example}.cpp)
    set_target_properties(${example} PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "$ORIGIN"
        SKIP_BUILD_RPATH FALSE
        BUILD_RPATH "$ORIGIN"
    )
    target_link_libraries(${example}
        pthread
        rt
        rbq_sdk::rbq_sdk
    )
    target_compile_definitions(${example} PRIVATE
        APP_NAME=\"${example}\"
    )
    install(TARGETS ${example}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endforeach()
