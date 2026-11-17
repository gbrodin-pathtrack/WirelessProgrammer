# Copies the release files to the release folder, named after the revision in git_revision.h:
#   [DIRTY_]wireless_programmer_<GIT_DATE>_<GIT_HASH>.hex  MCUboot and the app, for programming over SWD
#   [DIRTY_]wireless_programmer_<GIT_DATE>_<GIT_HASH>.bin  the signed and encrypted app, for MCUboot serial recovery
# Run with cmake -P:
#   HEADER       git_revision.h, as written by git_revision.cmake
#   SOURCE_HEX   the merged hex to copy
#   SOURCE_BIN   the signed and encrypted app to copy
#   RELEASE_DIR  where to copy them
cmake_minimum_required(VERSION 3.20)

foreach(required_var HEADER SOURCE_HEX SOURCE_BIN RELEASE_DIR)
    if(NOT DEFINED ${required_var} OR "${${required_var}}" STREQUAL "")
        message(FATAL_ERROR "${required_var} is required")
    endif()
endforeach()

foreach(required_file HEADER SOURCE_HEX SOURCE_BIN)
    if(NOT EXISTS "${${required_file}}")
        message(FATAL_ERROR "${required_file} not found: ${${required_file}}")
    endif()
endforeach()

file(READ "${HEADER}" HEADER_CONTENT)

function(read_header_value pattern output_var)
    string(REGEX MATCH "${pattern}" value_match "${HEADER_CONTENT}")
    if(value_match STREQUAL "")
        message(FATAL_ERROR "Failed to find '${pattern}' in ${HEADER}")
    endif()
    set(${output_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
endfunction()

read_header_value("GIT_REVISION_HASH 0x([0-9A-F]+)u" GIT_HASH)
read_header_value("GIT_DATE \"([0-9]+)\"" GIT_DATE)
read_header_value("GIT_DIRTY ([01])" GIT_DIRTY)

if(GIT_DIRTY)
    set(DIRTY_PREFIX "DIRTY_")
else()
    set(DIRTY_PREFIX "")
endif()

set(RELEASE_STEM "${RELEASE_DIR}/${DIRTY_PREFIX}wireless_programmer_${GIT_DATE}_${GIT_HASH}")

file(MAKE_DIRECTORY "${RELEASE_DIR}")
foreach(release IN ITEMS "${SOURCE_HEX}|${RELEASE_STEM}.hex" "${SOURCE_BIN}|${RELEASE_STEM}.bin")
    string(REPLACE "|" ";" release "${release}")
    list(GET release 0 source)
    list(GET release 1 destination)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${source}" "${destination}"
        RESULT_VARIABLE copy_rc
    )
    if(NOT copy_rc EQUAL 0)
        message(FATAL_ERROR "Failed to copy ${source} to ${destination}")
    endif()
    message(STATUS "Packaged release: ${destination}")
endforeach()
