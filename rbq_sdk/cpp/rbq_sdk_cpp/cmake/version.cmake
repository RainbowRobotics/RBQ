# Writes rbq_sdk/version.hpp (OUT) - see "SDK version" in CMakeLists.txt. Run with cmake -P on every build, so a
# re-install from an updated checkout carries the new version; the file is rewritten only when the version changes.
#   FIXED  -DRBQ_SDK_VERSION given at configure (the Debian package), else env RBQ_VERSION (release builds),
#          else git describe of this checkout, else 0.0.0-dev
#   SRC    the SDK source directory, for git describe
set(version "${FIXED}")
if(version STREQUAL "" AND DEFINED ENV{RBQ_VERSION})
    set(version "$ENV{RBQ_VERSION}")
endif()
if(version STREQUAL "")
    execute_process(
        COMMAND git describe --tags --dirty --always --match "v*"
        WORKING_DIRECTORY "${SRC}"
        OUTPUT_VARIABLE version OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
endif()
if(version STREQUAL "")
    set(version "0.0.0-dev")
endif()

set(text "#pragma once\n\n// Generated when the SDK was built (cmake/version.cmake): the RBQ version it was built from.\n#define RBQ_SDK_VERSION_STRING \"${version}\"\n")
set(old "")
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL text)
    file(WRITE "${OUT}" "${text}")
endif()
