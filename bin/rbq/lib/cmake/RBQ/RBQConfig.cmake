set(RBQ_VERSION 1.4.155)

IF(CUSTOM_RBQ_PATH)
    FIND_PATH (RBQ_INCLUDE_DIR rbq/Api.h
        PATHS
        ${CUSTOM_RBQ_PATH}/include
        NO_DEFAULT_PATH
    )
    file(GLOB RBQ_LIBRARY
        "${CUSTOM_RBQ_PATH}/lib/*.so"
        "${CUSTOM_RBQ_PATH}/lib/*.so.*"
        "${CUSTOM_RBQ_PATH}/lib/*.a"
    )
ELSE(CUSTOM_RBQ_PATH)
    MESSAGE (SEND_ERROR " Could not find RBQ.")
ENDIF(CUSTOM_RBQ_PATH)

IF (RBQ_INCLUDE_DIR AND RBQ_LIBRARY)
    SET (RBQ_FOUND TRUE)
ELSE(RBQ_INCLUDE_DIR AND RBQ_LIBRARY)
    IF(RBQ_FIND_REQUIRED)
        MESSAGE (SEND_ERROR " Could not find RBQ.")
        MESSAGE (SEND_ERROR " Try setting CUSTOM_RBQ_PATH in FindRBQ.cmake force CMake to use the desired directory.")
    ELSE(RBQ_FIND_REQUIRED)
        MESSAGE (STATUS " Could not find RBQ.")
        MESSAGE (STATUS " Try setting CUSTOM_RBQ_PATH in FindRBQ.cmake force CMake to use the desired directory.")
    ENDIF(RBQ_FIND_REQUIRED)
ENDIF (RBQ_INCLUDE_DIR AND RBQ_LIBRARY)

IF (RBQ_FOUND)
    IF (NOT RBQ_FIND_QUIETLY)
        MESSAGE(STATUS "Found RBQ: ${RBQ_LIBRARY}")
    ENDIF (NOT RBQ_FIND_QUIETLY)
    foreach ( COMPONENT ${RBQ_FIND_COMPONENTS} )
        IF (RBQ_${COMPONENT}_FOUND)
            IF (NOT RBQ_FIND_QUIETLY)
                MESSAGE(STATUS "Found RBQ ${COMPONENT}: ${RBQ_${COMPONENT}_LIBRARY}")
            ENDIF (NOT RBQ_FIND_QUIETLY)
        ELSE (RBQ_${COMPONENT}_FOUND)
            MESSAGE(ERROR " Could not find RBQ ${COMPONENT}")
        ENDIF (RBQ_${COMPONENT}_FOUND)
    endforeach ( COMPONENT )
ENDIF (RBQ_FOUND)

MARK_AS_ADVANCED (
    RBQ_INCLUDE_DIR
    RBQ_LIBRARY
)
