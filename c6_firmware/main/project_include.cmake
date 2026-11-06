# Cyberdeck v5: replace libble_app.a with the copy containing the CIS hooks (description in cmake/ble_hooks.cmake).
# Only from here: idf_build_process includes the components' project_include.cmake after toolchain.cmake (which
# includes utilities.cmake again and would overwrite an earlier add_prebuilt_library definition), and before the
# components' CMakeLists (build.cmake: __build_process_project_includes, then add_subdirectory(${idf_path})).
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ble_hooks.cmake)
