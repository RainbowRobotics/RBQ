# Quadruped (RBQ) examples.

add_executable(rbq_low_level rbq_low_level.cpp)
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


add_executable(rbq_high_level rbq_high_level.cpp)
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

add_executable(rbq_sport_client_joy rbq_sport_client_joy.cpp)
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

add_executable(rbq_image_echo rbq_image_echo.cpp)
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

add_executable(rbq_sport_client rbq_sport_client.cpp)
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

add_executable(rbq_target_go rbq_target_go.cpp)
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
