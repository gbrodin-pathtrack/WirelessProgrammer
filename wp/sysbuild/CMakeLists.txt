# Wireless programmer sysbuild: the default MCUboot and app images, plus release packaging once both are built.

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sysbuild LANGUAGES)

# MCUboot and the app share one board target, so sysbuild's merged hex for that target holds the complete image
function(normalize_board_target output_var board_target)
    string(REPLACE "/" "_" board_target "${board_target}")
    string(REPLACE "." "_" board_target "${board_target}")
    string(REPLACE "@" "_" board_target "${board_target}")
    set(${output_var} "${board_target}" PARENT_SCOPE)
endfunction()

sysbuild_get(app_board_target IMAGE ${DEFAULT_IMAGE} VAR CONFIG_BOARD_TARGET KCONFIG)
sysbuild_get(app_kernel_name IMAGE ${DEFAULT_IMAGE} VAR CONFIG_KERNEL_BIN_NAME KCONFIG)
normalize_board_target(app_board_target "${app_board_target}")

set(app_binary_dir ${CMAKE_BINARY_DIR}/${DEFAULT_IMAGE})

# Copies the merged hex, for programming over SWD, and the signed and encrypted app, for serial recovery, to release/,
# named after the revision in the app's git_revision.h
add_custom_target(wp_release ALL
    COMMAND ${CMAKE_COMMAND}
            -DHEADER=${app_binary_dir}/generated/git_revision.h
            -DSOURCE_HEX=${CMAKE_BINARY_DIR}/merged_${app_board_target}.hex
            -DSOURCE_BIN=${app_binary_dir}/zephyr/${app_kernel_name}.signed.encrypted.bin
            -DRELEASE_DIR=${APP_DIR}/release
            -P ${APP_DIR}/cmake/package_release.cmake
    COMMENT "Packaging wireless programmer release"
    VERBATIM
)
add_dependencies(wp_release merged_${app_board_target}_target)
