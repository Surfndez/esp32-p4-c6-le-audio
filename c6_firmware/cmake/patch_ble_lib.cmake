# Cyberdeck v5: copy of libble_app.a with hooks (cmake -P). Never modifies the file in ESP-IDF.
# Input: -DAR= -DOBJCOPY= -DOBJDUMP= -DNM= -DSRC=<original> -DDST=<copy> -DHOOKS=<ble_hooks.txt>
# Recipe: for symbol X in object O
#   objcopy --weaken-symbol=X --add-symbol orig_X=<section of X>:<offset of X>,global,function O
#   ar r copy O   (same member name = replacement; a different name would append a duplicate -> "multiple definition")
# Calls in the library have an R_RISCV_CALL relocation against the global symbol X, so our strong X wins also
# within the same object; orig_X points to the original code.
cmake_minimum_required(VERSION 3.16)

foreach(v AR OBJCOPY OBJDUMP NM SRC DST HOOKS)
	if(NOT DEFINED ${v} OR "${${v}}" STREQUAL "")
		message(FATAL_ERROR "patch_ble_lib: missing -D${v}")
	endif()
endforeach()

get_filename_component(out_dir "${DST}" DIRECTORY)
set(work "${out_dir}/work")
file(REMOVE_RECURSE "${work}")
file(MAKE_DIRECTORY "${work}/repl")
set(log "${out_dir}/patch_ble_lib.log")
file(WRITE "${log}" "source: ${SRC}\n")

function(run_tool out_var)
	execute_process(COMMAND ${ARGN} WORKING_DIRECTORY "${work}"
		RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
	if(NOT rc EQUAL 0)
		message(FATAL_ERROR "patch_ble_lib: ${ARGN}\n${err}")
	endif()
	set(${out_var} "${out}" PARENT_SCOPE)
endfunction()

# List of hooks: object -> symbols.
file(STRINGS "${HOOKS}" lines REGEX "^[^#].*[^ ]")
set(objects)
foreach(line ${lines})
	string(REGEX MATCH "^([^ \t]+)[ \t]+([A-Za-z_][A-Za-z0-9_]*)[ \t]*$" m "${line}")
	if(NOT m)
		message(FATAL_ERROR "patch_ble_lib: bad line in ${HOOKS}: '${line}'")
	endif()
	set(obj "${CMAKE_MATCH_1}")
	set(sym "${CMAKE_MATCH_2}")
	list(APPEND objects "${obj}")
	list(APPEND syms_${obj} "${sym}")
endforeach()
list(REMOVE_DUPLICATES objects)

run_tool(members "${AR}" t "${SRC}")
string(REPLACE "\n" ";" members "${members}")
list(LENGTH members n_before)

file(COPY_FILE "${SRC}" "${work}/libble_app.a")
foreach(obj ${objects})
	# Exactly one member with this name, otherwise "ar x/r" would hit the wrong one.
	set(count 0)
	foreach(m ${members})
		if(m STREQUAL obj)
			math(EXPR count "${count} + 1")
		endif()
	endforeach()
	if(NOT count EQUAL 1)
		message(FATAL_ERROR "patch_ble_lib: member ${obj} occurs ${count} times in the archive")
	endif()
	run_tool(_ "${AR}" x "${SRC}" "${obj}")
	run_tool(table "${OBJDUMP}" -t "${obj}")
	set(args)
	foreach(sym ${syms_${obj}})
		# An objdump -t line: "<address> g     F <section>\t<size> <name>" - the function's own section (.text.X)
		# or a shared one (.iram1.N); the offset is the address within the section.
		string(REGEX MATCH "\n([0-9a-f]+) g     F ([^\t\n]+)\t[0-9a-f]+ ${sym}\n" m "\n${table}")
		if(NOT m)
			message(FATAL_ERROR "patch_ble_lib: ${sym} is not a global function in ${obj}")
		endif()
		set(off "${CMAKE_MATCH_1}")
		set(sec "${CMAKE_MATCH_2}")
		list(APPEND args "--weaken-symbol=${sym}" "--add-symbol" "orig_${sym}=${sec}:0x${off},global,function")
		file(APPEND "${log}" "${obj}: ${sym} in ${sec}+0x${off}\n")
	endforeach()
	run_tool(_ "${OBJCOPY}" ${args} "${obj}" "repl/${obj}")
	# Check in the object: X weak (W), orig_X global (T).
	run_tool(nm_out "${NM}" "repl/${obj}")
	foreach(sym ${syms_${obj}})
		if(NOT nm_out MATCHES "[0-9a-f]+ W ${sym}\n" OR NOT nm_out MATCHES "[0-9a-f]+ T orig_${sym}\n")
			message(FATAL_ERROR "patch_ble_lib: after objcopy ${sym}/orig_${sym} have wrong types in ${obj}")
		endif()
	endforeach()
	run_tool(_ "${AR}" r "libble_app.a" "repl/${obj}")
endforeach()

run_tool(members_after "${AR}" t "${work}/libble_app.a")
string(REPLACE "\n" ";" members_after "${members_after}")
list(LENGTH members_after n_after)
if(NOT n_before EQUAL n_after)
	message(FATAL_ERROR "patch_ble_lib: member count ${n_before} -> ${n_after} (ar appended instead of replacing)")
endif()
file(APPEND "${log}" "members: ${n_after}, replaced: ${objects}\n")
# The file name must stay libble_app.a: the bt linker fragment (linker_esp_ble_controller.lf, archive: libble_app.a)
# places sections by archive name.
file(COPY_FILE "${work}/libble_app.a" "${DST}")
