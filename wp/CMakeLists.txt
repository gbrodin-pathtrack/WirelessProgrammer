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

# Release packaging is done by sysbuild/CMakeLists.txt, once MCUboot and the app are merged
