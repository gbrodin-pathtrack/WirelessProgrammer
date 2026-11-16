cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nordic_wp_protocol)

target_sources(app PRIVATE src/main.c)

# git_revision.h, with the commit sent to the host. Made while configuring so it exists for the first build, then
# refreshed before every build, only being rewritten when the revision changes.
set(GIT_REVISION_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GIT_REVISION_HEADER ${GIT_REVISION_DIR}/git_revision.h)
set(GIT_REVISION_COMMAND
    ${CMAKE_COMMAND}
    -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DOUTPUT=${GIT_REVISION_HEADER}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/git_revision.cmake
)

execute_process(COMMAND ${GIT_REVISION_COMMAND} RESULT_VARIABLE GIT_REVISION_RC)
if(NOT GIT_REVISION_RC EQUAL 0)
    message(FATAL_ERROR "Failed to generate ${GIT_REVISION_HEADER}")
endif()

add_custom_target(git_revision_refresh
    COMMAND ${GIT_REVISION_COMMAND}
    BYPRODUCTS ${GIT_REVISION_HEADER}
    COMMENT "Refreshing git revision"
    VERBATIM
)
add_dependencies(app git_revision_refresh)
target_include_directories(app PRIVATE ${GIT_REVISION_DIR})

# Copies the built hex to release/, named after the revision it was built from
add_custom_target(wp_release ALL
    COMMAND ${CMAKE_COMMAND}
            -DHEADER=${GIT_REVISION_HEADER}
            -DSOURCE_HEX=${PROJECT_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.hex
            -DRELEASE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/release
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/package_release.cmake
    COMMENT "Packaging wireless programmer release"
    VERBATIM
)
add_dependencies(wp_release zephyr_final git_revision_refresh)
