# Cyberdeck v5: link a copy of libble_app.a with the CIS hooks, without changing ESP-IDF.
#
# bt/controller/CMakeLists.txt calls add_prebuilt_library(libble_app <IDF path> REQUIRES esp_phy bt). The IMPORTED
# target is visible only in the bt directory, so the project cannot change its path. CMake functions are global:
# the definition below replaces add_prebuilt_library from tools/cmake/utilities.cmake and substitutes, for
# libble_app, the copy built by patch_ble_lib.cmake. Included from main/project_include.cmake: earlier (before
# project()) it does not work, because toolchain.cmake includes utilities.cmake again and restores the original.
# The body apart from the substitution is a copy of the ESP-IDF 5.5.5 function (utilities.cmake:300).

set_property(GLOBAL PROPERTY CIS_BLE_HOOKS_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(add_prebuilt_library target_name lib_path)
	cmake_parse_arguments(_ "" "" "REQUIRES;PRIV_REQUIRES" ${ARGN})

	get_filename_component(lib_path "${lib_path}"
		ABSOLUTE BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")

	set(imported "${lib_path}")
	if(target_name STREQUAL "libble_app")
		get_filename_component(lib_name "${lib_path}" NAME)
		if(NOT lib_name STREQUAL "libble_app.a" OR NOT CONFIG_IDF_TARGET_ESP32C6)
			# The hooks are validated only for this library; silently running without them would pass for the v5 payload.
			message(FATAL_ERROR "cis hooks: expected esp32c6 libble_app.a, got ${lib_path}")
		endif()
		get_property(hooks_dir GLOBAL PROPERTY CIS_BLE_HOOKS_DIR)
		idf_build_get_property(build_dir BUILD_DIR)
		set(imported "${build_dir}/ble_hooked/libble_app.a")
		add_custom_command(OUTPUT "${imported}"
			COMMAND ${CMAKE_COMMAND}
				-DAR=${CMAKE_AR} -DOBJCOPY=${CMAKE_OBJCOPY} -DOBJDUMP=${CMAKE_OBJDUMP} -DNM=${CMAKE_NM}
				-DSRC=${lib_path} -DDST=${imported} -DHOOKS=${hooks_dir}/ble_hooks.txt
				-P ${hooks_dir}/patch_ble_lib.cmake
			DEPENDS "${lib_path}" "${hooks_dir}/ble_hooks.txt" "${hooks_dir}/patch_ble_lib.cmake"
			COMMENT "cis hooks: copy of libble_app.a with weakened symbols"
			VERBATIM)
		add_custom_target(cis_ble_hooked_lib DEPENDS "${imported}")
		message(STATUS "cis hooks: libble_app -> ${imported}")
	endif()

	add_library(${target_name} STATIC IMPORTED)
	set_property(TARGET ${target_name} PROPERTY IMPORTED_LOCATION ${imported})
	if(TARGET cis_ble_hooked_lib AND target_name STREQUAL "libble_app")
		# A dependency of an IMPORTED target propagates to everything that links it (the copy exists before the link).
		add_dependencies(${target_name} cis_ble_hooked_lib)
	endif()

	foreach(req ${__REQUIRES})
		idf_component_get_property(req_lib "${req}" COMPONENT_LIB)
		set_property(TARGET ${target_name} APPEND PROPERTY LINK_LIBRARIES "${req_lib}")
		set_property(TARGET ${target_name} APPEND PROPERTY INTERFACE_LINK_LIBRARIES "${req_lib}")
	endforeach()

	foreach(req ${__PRIV_REQUIRES})
		idf_component_get_property(req_lib "${req}" COMPONENT_LIB)
		set_property(TARGET ${target_name} APPEND PROPERTY LINK_LIBRARIES "${req_lib}")
		set_property(TARGET ${target_name} APPEND PROPERTY INTERFACE_LINK_LIBRARIES "$<LINK_ONLY:${req_lib}>")
	endforeach()
endfunction()
