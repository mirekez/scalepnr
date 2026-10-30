cmake_minimum_required(VERSION 3.26...3.30)

if(WIN32)
set (CMAKE_LINKER "ld64.lld.exe")
set (CMAKE_C_COMPILER "x86_64-w64-mingw32-gcc")
set (CMAKE_CXX_COMPILER "x86_64-w64-mingw32-g++")
set (CMAKE_GENERATOR "Unix Makefiles")
set (CMAKE_MAKE_PROGRAM make)
set (CMAKE_C_COMPILER_FORCED "TRUE")
set (CMAKE_CXX_COMPILER_FORCED "TRUE")
else()
set (CMAKE_LINKER "x86_64-conda-linux-gnu-ld")
set (CMAKE_C_COMPILER "x86_64-conda-linux-gnu-gcc")
set (CMAKE_CXX_COMPILER "x86_64-conda-linux-gnu-g++")
set (CMAKE_GENERATOR "Unix Makefiles")
set (CMAKE_MAKE_PROGRAM make)
set (CMAKE_C_COMPILER_FORCED "TRUE")
set (CMAKE_CXX_COMPILER_FORCED "TRUE")
endif()

SET (CMAKE_C_FLAGS_INIT                "-Wall -std=c11")
SET (CMAKE_C_FLAGS_DEBUG_INIT          "-g")
SET (CMAKE_C_FLAGS_MINSIZEREL_INIT     "-Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE_INIT        "-O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")

SET (CMAKE_CXX_FLAGS_INIT                "-Wall -std=c++2c")
SET (CMAKE_CXX_FLAGS_DEBUG_INIT          "-g")
SET (CMAKE_CXX_FLAGS_MINSIZEREL_INIT     "-Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELEASE_INIT        "-O3 -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")

set(CMAKE_CXX_FLAGS "-Wall -std=c++2c -g -fno-strict-aliasing ")

project(
  scalepnr
  VERSION 1.0
  LANGUAGES CXX)

set(SCALEPNR_PNR_RUN_CELLS 1000 CACHE STRING "Target newly admitted cells per complete PnR run")
if(NOT SCALEPNR_PNR_RUN_CELLS MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "SCALEPNR_PNR_RUN_CELLS must be a positive integer")
endif()
add_compile_definitions(SCALEPNR_PNR_RUN_CELLS=${SCALEPNR_PNR_RUN_CELLS})

set(SCALEPNR_PLACE_REDISTRIBUTION_STEP 0.20 CACHE STRING
    "Maximum continuous register-attraction displacement scale per cooling pass")
if(NOT SCALEPNR_PLACE_REDISTRIBUTION_STEP MATCHES
        "^([0-9]+([.][0-9]*)?|[.][0-9]+)$"
    OR SCALEPNR_PLACE_REDISTRIBUTION_STEP LESS_EQUAL 0)
    message(FATAL_ERROR
        "SCALEPNR_PLACE_REDISTRIBUTION_STEP must be a positive number")
endif()
set_property(SOURCE src/pnr/place/PlaceDesign.cpp APPEND PROPERTY
    COMPILE_DEFINITIONS
        SCALEPNR_PLACE_REDISTRIBUTION_STEP=${SCALEPNR_PLACE_REDISTRIBUTION_STEP})

set(SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR 2.0 CACHE STRING
    "Placement cooling passes as a multiple of half the device width")
if(NOT SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR MATCHES
        "^([0-9]+([.][0-9]*)?|[.][0-9]+)$"
    OR SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR LESS_EQUAL 0)
    message(FATAL_ERROR
        "SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR must be a positive number")
endif()
set_property(SOURCE src/pnr/place/PlaceDesign.cpp APPEND PROPERTY
    COMPILE_DEFINITIONS
        SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR=${SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR})

add_definitions(-DSTATIC_BUILD)

add_subdirectory(libs)

set(SCALEPNR_RE2_ABSL_SOURCES
    libs/re2/re2/re2.cc
    libs/re2/re2/regexp.cc
    libs/re2/re2/compile.cc
    libs/re2/re2/bitmap256.cc
    libs/re2/re2/prog.cc
    libs/re2/re2/parse.cc
    libs/re2/re2/prefilter.cc
    libs/re2/re2/simplify.cc
    libs/re2/re2/dfa.cc
    libs/re2/re2/nfa.cc
    libs/re2/re2/tostring.cc
    libs/re2/re2/onepass.cc
    libs/re2/re2/unicode_groups.cc
    libs/re2/re2/unicode_casefold.cc
    libs/re2/re2/bitstate.cc
    libs/re2/re2/perl_groups.cc
    libs/re2/util/rune.cc
    libs/re2/util/strutil.cc
    libs/abseil-cpp/absl/container/internal/raw_hash_set.cc
    libs/abseil-cpp/absl/base/internal/cycleclock.cc
    libs/abseil-cpp/absl/strings/ascii.cc
    libs/abseil-cpp/absl/strings/numbers.cc
    libs/abseil-cpp/absl/strings/charconv.cc
    libs/abseil-cpp/absl/strings/match.cc
    libs/abseil-cpp/absl/strings/internal/charconv_bigint.cc
    libs/abseil-cpp/absl/strings/internal/charconv_parse.cc
    libs/abseil-cpp/absl/strings/internal/memutil.cc
    libs/abseil-cpp/absl/strings/internal/str_format/bind.cc
    libs/abseil-cpp/absl/strings/internal/str_format/output.cc
    libs/abseil-cpp/absl/strings/internal/str_format/parser.cc
    libs/abseil-cpp/absl/strings/internal/str_format/extension.cc
    libs/abseil-cpp/absl/strings/internal/str_format/arg.cc
    libs/abseil-cpp/absl/strings/internal/str_format/float_conversion.cc
    libs/abseil-cpp/absl/numeric/int128.cc
    libs/abseil-cpp/absl/time/duration.cc
    libs/abseil-cpp/absl/time/clock.cc
    libs/abseil-cpp/absl/hash/internal/hash.cc
    libs/abseil-cpp/absl/hash/internal/low_level_hash.cc
    libs/abseil-cpp/absl/hash/internal/city.cc
)

add_executable(scalepnr
    src/main.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/pnr.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    src/tcl/tcl_pnr.cpp
    src/tcl/get_tiles_cmd.cpp
    src/tcl/get_ports_cmd.cpp
    src/tcl/get_nets_cmd.cpp
    src/tcl/get_wires_cmd.cpp
    src/tcl/load_spec_cmd.cpp
    src/tcl/create_clock_cmd.cpp
    src/tcl/check_timing_cmd.cpp
    src/tcl/load_design_cmd.cpp
    src/tcl/open_design_cmd.cpp
    src/tcl/place_design_cmd.cpp
    src/tcl/route_design_cmd.cpp
    src/tcl/print_design_cmd.cpp
    src/tcl/set_property_cmd.cpp
    src/tcl/design_state_cmd.cpp
    libs/re2/re2/re2.cc
    libs/re2/re2/regexp.cc
    libs/re2/re2/compile.cc
    libs/re2/re2/bitmap256.cc
    libs/re2/re2/prog.cc
    libs/re2/re2/parse.cc
    libs/re2/re2/prefilter.cc
    libs/re2/re2/simplify.cc
    libs/re2/re2/dfa.cc
    libs/re2/re2/nfa.cc
    libs/re2/re2/tostring.cc
    libs/re2/re2/onepass.cc
    libs/re2/re2/unicode_groups.cc
    libs/re2/re2/unicode_casefold.cc
    libs/re2/re2/bitstate.cc
    libs/re2/re2/perl_groups.cc
    libs/re2/util/rune.cc
    libs/re2/util/strutil.cc
    libs/abseil-cpp/absl/container/internal/raw_hash_set.cc
    libs/abseil-cpp/absl/base/internal/cycleclock.cc
    libs/abseil-cpp/absl/strings/ascii.cc
    libs/abseil-cpp/absl/strings/numbers.cc
    libs/abseil-cpp/absl/strings/charconv.cc
    libs/abseil-cpp/absl/strings/match.cc
    libs/abseil-cpp/absl/strings/internal/charconv_bigint.cc
    libs/abseil-cpp/absl/strings/internal/charconv_parse.cc
    libs/abseil-cpp/absl/strings/internal/memutil.cc
    libs/abseil-cpp/absl/strings/internal/str_format/bind.cc
    libs/abseil-cpp/absl/strings/internal/str_format/output.cc
    libs/abseil-cpp/absl/strings/internal/str_format/parser.cc
    libs/abseil-cpp/absl/strings/internal/str_format/extension.cc
    libs/abseil-cpp/absl/strings/internal/str_format/arg.cc
    libs/abseil-cpp/absl/strings/internal/str_format/float_conversion.cc
    libs/abseil-cpp/absl/numeric/int128.cc
    libs/abseil-cpp/absl/time/duration.cc
    libs/abseil-cpp/absl/time/clock.cc
    libs/abseil-cpp/absl/hash/internal/hash.cc
    libs/abseil-cpp/absl/hash/internal/low_level_hash.cc
    libs/abseil-cpp/absl/hash/internal/city.cc
)

include_directories(
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/tcl
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/tcl8.6.14/generic
        libs/tcl8.6.14/win
        libs/tcl8.6.14/unix
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_library(tcl_imported STATIC IMPORTED)
set_target_properties(tcl_imported PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libs/tcl8.6.14/libtcl.a)
target_link_libraries(scalepnr PRIVATE tcl_imported)
add_dependencies(scalepnr tcl)

target_link_options(scalepnr PRIVATE -static-libgcc -static-libstdc++)

if(WIN32)

target_link_libraries(scalepnr PUBLIC ws2_32.lib)
target_link_libraries(scalepnr PUBLIC Netapi32.lib)
target_link_libraries(scalepnr PUBLIC Userenv.lib)
target_link_libraries(scalepnr PUBLIC stdc++exp.lib)

target_link_libraries(scalepnr PUBLIC libpthread.a)

else()

target_link_libraries(scalepnr PUBLIC stdc++exp)
target_link_libraries(scalepnr PUBLIC pthread)
target_link_libraries(scalepnr PUBLIC dl)

endif()

find_package(PNG REQUIRED)
target_link_libraries(scalepnr PUBLIC PNG::PNG)

enable_testing()

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME backend.scripts
        COMMAND ${Python3_EXECUTABLE} -m unittest discover
            -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/prjxray -p "*_test.py")
    set_tests_properties(backend.scripts PROPERTIES TIMEOUT 60)
endif()

add_executable(routing_test
    src/tests/fpga/routing_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(routing_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.routing COMMAND routing_test)
add_test(NAME fpga.routing_landing_owner COMMAND routing_test landing_owner)
add_test(NAME fpga.routing_discard_owner COMMAND routing_test discard_owner)
add_test(NAME fpga.routing_history_watch COMMAND routing_test history_watch)

add_executable(grounding_preemption
    src/tests/fpga/grounding_preemption.cpp
    src/tests/fpga/fixed_endpoint_recovery.cpp
    src/tests/fpga/fixed_endpoint_preemption.cpp
    src/tests/fpga/source_candidate_inputs.cpp
    src/tests/fpga/partial_preemption.cpp
    src/tests/fpga/fanout_trunk_handoff.cpp
    src/tests/fpga/fanout_local_trunk.cpp
    src/tests/fpga/fanout_shared_input_repair.cpp
    src/tests/fpga/fanout_shared_terminal_gap.cpp
    src/tests/fpga/fanout_search_budget.cpp
    src/tests/fpga/fanout_transit_local.cpp
    src/tests/fpga/basic_transit_local.cpp
    src/tests/fpga/basic_search_budget.cpp
    src/tests/fpga/basic_local_templates.cpp
    src/tests/fpga/fanout_complete_input.cpp
    src/tests/fpga/basic_remote_local.cpp
    src/tests/fpga/basic_detour.cpp
    src/tests/fpga/basic_free_detour.cpp
    src/tests/fpga/basic_grounding_detour.cpp
    src/tests/fpga/source_fallback_precedence.cpp
    src/tests/fpga/fanout_source_priority.cpp
    src/tests/fpga/fanout_numeric_seed.cpp
    src/tests/fpga/fanout_numeric_grounding.cpp
    src/tests/fpga/fanout_docking_owner.cpp
    src/tests/fpga/fanout_gap_replay.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(grounding_preemption
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.grounding_preemption COMMAND grounding_preemption)
foreach(case basic_transit_local basic_transit_local_direct basic_transit_local_blocked
             basic_search_budget basic_search_budget_control basic_search_budget_blocked
             fanout_source_priority fanout_source_priority_control fanout_source_priority_blocked
             fanout_numeric_seed fanout_numeric_seed_control fanout_numeric_seed_blocked fanout_numeric_seed_local
             fanout_numeric_seed_arrival)
    add_test(NAME fpga.${case} COMMAND grounding_preemption ${case})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.${case} PROPERTIES TIMEOUT 30
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
foreach(case transit_local transit_local_direct transit_local_blocked transit_local_reuse)
    add_test(NAME fpga.fanout_${case} COMMAND grounding_preemption ${case})
    set_tests_properties(fpga.fanout_${case} PROPERTIES TIMEOUT 30)
endforeach()
add_test(NAME fpga.fanout_island_landing COMMAND grounding_preemption island_landing)
foreach(mode local_templates local_templates_budget local_templates_blocked local_templates_control)
    add_test(NAME fpga.basic_${mode} COMMAND grounding_preemption ${mode})
    set_tests_properties(fpga.basic_${mode} PROPERTIES TIMEOUT 30)
endforeach()
foreach(mode complete_input complete_input_control complete_input_other)
    add_test(NAME fpga.fanout_${mode} COMMAND grounding_preemption ${mode})
    set_tests_properties(fpga.fanout_${mode} PROPERTIES TIMEOUT 60)
endforeach()
foreach(mode remote_local remote_local_budget remote_local_blocked)
    add_test(NAME fpga.basic_${mode} COMMAND grounding_preemption ${mode})
    set_tests_properties(fpga.basic_${mode} PROPERTIES TIMEOUT 60)
endforeach()
foreach(mode detour detour_budget detour_blocked detour_audit detour_failure_capture)
    add_test(NAME fpga.basic_${mode} COMMAND grounding_preemption ${mode})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/basic_${mode}")
    set_tests_properties(fpga.basic_${mode} PROPERTIES TIMEOUT 60
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/basic_${mode}")
endforeach()
foreach(mode grounding_detour grounding_detour_budget grounding_detour_protected grounding_detour_disconnected grounding_detour_recovery)
    add_test(NAME fpga.basic_${mode} COMMAND grounding_preemption ${mode})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/basic_${mode}")
    set_tests_properties(fpga.basic_${mode} PROPERTIES TIMEOUT 120
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/basic_${mode}")
endforeach()
add_test(NAME fpga.source_fallback_precedence COMMAND grounding_preemption source_fallback_precedence)
foreach(mode free_detour free_detour_budget free_detour_disconnected free_detour_limited)
    add_test(NAME fpga.basic_${mode} COMMAND grounding_preemption ${mode})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/basic_${mode}")
    set_tests_properties(fpga.basic_${mode} PROPERTIES TIMEOUT 180
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/basic_${mode}")
endforeach()
set_tests_properties(fpga.source_fallback_precedence PROPERTIES TIMEOUT 30)
foreach(mode cut_island cut_island_reverse)
    add_test(NAME fpga.fanout_${mode} COMMAND grounding_preemption ${mode})
    set_tests_properties(fpga.fanout_${mode} PROPERTIES TIMEOUT 30)
endforeach()
add_test(NAME fpga.fanout_fork_role COMMAND grounding_preemption fork_role)
set_tests_properties(fpga.fanout_fork_role PROPERTIES TIMEOUT 30)
foreach(mode numeric_grounding numeric_grounding_protected numeric_grounding_disconnected)
    add_test(NAME fpga.fanout_${mode} COMMAND grounding_preemption ${mode})
    set_tests_properties(fpga.fanout_${mode} PROPERTIES TIMEOUT 30)
endforeach()
add_test(NAME fpga.fanout_island_audit COMMAND grounding_preemption island_audit)
set_tests_properties(fpga.fanout_island_audit PROPERTIES TIMEOUT 30)
set_tests_properties(fpga.fanout_island_landing PROPERTIES TIMEOUT 30)
add_test(NAME fpga.partial_preemption COMMAND grounding_preemption partial)
add_test(NAME fpga.preserved_route_repair COMMAND grounding_preemption preserved_repair)
add_test(NAME fpga.fanout_trunk_handoff COMMAND grounding_preemption trunk_handoff)
foreach(mode takeoff_handoff takeoff_handoff_control)
    add_test(NAME fpga.fanout_${mode} COMMAND grounding_preemption ${mode})
    set_tests_properties(fpga.fanout_${mode} PROPERTIES TIMEOUT 30)
endforeach()
add_test(NAME fpga.fanout_local_trunk COMMAND grounding_preemption local_trunk)
add_test(NAME fpga.fanout_local_trunk_blocked COMMAND grounding_preemption local_trunk_blocked)
set_tests_properties(fpga.fanout_local_trunk fpga.fanout_local_trunk_blocked PROPERTIES TIMEOUT 30)
add_test(NAME fpga.fanout_trunk_handoff_control COMMAND grounding_preemption trunk_handoff_control)
set_tests_properties(fpga.fanout_trunk_handoff fpga.fanout_trunk_handoff_control
    PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
add_test(NAME fpga.fanout_shared_input_repair COMMAND grounding_preemption shared_input_repair)
foreach(case fixed_endpoint_recovery fixed_endpoint_control fixed_endpoint_blocked)
    add_test(NAME fpga.${case} COMMAND grounding_preemption ${case})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
foreach(case fixed_endpoint_transit fixed_endpoint_transit_control fixed_endpoint_takeoff_protected
             moving_wire_first_transit moving_wire_first_takeoff_protected
             moving_wire_first_full_fallback fixed_endpoint_source_probe
             fixed_endpoint_source_probe_full_fallback)
    add_test(NAME fpga.${case} COMMAND grounding_preemption ${case})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
foreach(case shared_terminal_gap shared_terminal_private shared_terminal_exact shared_terminal_blocked retained_src_rejoin retained_src_exact retained_src_protected)
    add_test(NAME fpga.fanout_${case} COMMAND grounding_preemption ${case})
    set_tests_properties(fpga.fanout_${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
endforeach()
add_test(NAME fpga.fanout_shared_input_repair_control COMMAND grounding_preemption shared_input_repair_control)
add_test(NAME fpga.fanout_shared_input_repair_blocked COMMAND grounding_preemption shared_input_repair_blocked)
set_tests_properties(fpga.fanout_shared_input_repair fpga.fanout_shared_input_repair_control fpga.fanout_shared_input_repair_blocked
    PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
foreach(case trunk_handoff trunk_handoff_control takeoff_handoff takeoff_handoff_control
        numeric_grounding numeric_grounding_protected numeric_grounding_disconnected
        transit_local transit_local_direct transit_local_blocked transit_local_reuse
        shared_input_repair shared_input_repair_control shared_input_repair_blocked)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.fanout_${case} PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
foreach(case search_budget search_budget_control search_budget_short search_budget_blocked)
    add_test(NAME fpga.fanout_${case} COMMAND grounding_preemption ${case})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.fanout_${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
add_test(NAME fpga.fanout_soft_failure_batch COMMAND grounding_preemption search_budget_blocked)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/fanout_soft_failure_batch")
set_tests_properties(fpga.fanout_soft_failure_batch PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
    ENVIRONMENT "SCALEPNR_ROUTE_HEARTBEAT=0"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/fanout_soft_failure_batch")
foreach(case private_grounding_repair private_grounding_repair_direct)
    add_test(NAME fpga.fanout_${case} COMMAND grounding_preemption ${case})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.fanout_${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
foreach(case docking_owner docking_owner_free docking_owner_transit docking_owner_removed docking_owner_grounding docking_forward docking_forward_free docking_forward_grounding docking_forward_blocked docking_forward_preserve retained_fork_landing fork_source_replacement)
    add_test(NAME fpga.fanout_${case} COMMAND grounding_preemption ${case})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
    set_tests_properties(fpga.fanout_${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tests/${case}")
endforeach()
add_test(NAME fpga.route_owner_query COMMAND grounding_preemption owner_query)
add_test(NAME fpga.fanout_gap_replay COMMAND grounding_preemption gap_replay)
add_test(NAME fpga.fanout_gap_budget COMMAND grounding_preemption gap_budget)
add_test(NAME fpga.fanout_gap_probe_budget COMMAND grounding_preemption gap_probe_budget)
set_tests_properties(fpga.fanout_gap_probe_budget PROPERTIES TIMEOUT 30)
add_test(NAME fpga.fanout_gap_failure_location COMMAND grounding_preemption gap_failure_location)
add_test(NAME fpga.fanout_gap_grounding COMMAND grounding_preemption gap_grounding)
add_test(NAME fpga.fanout_retained_suffix COMMAND grounding_preemption retained_suffix)
add_test(NAME fpga.fanout_private_grounding COMMAND grounding_preemption private_grounding)
add_test(NAME fpga.fanout_final_jump COMMAND grounding_preemption final_jump)
add_test(NAME fpga.fanout_two_jump_grounding COMMAND grounding_preemption two_jump_grounding)
set_tests_properties(fpga.fanout_two_jump_grounding PROPERTIES TIMEOUT 30)
add_test(NAME fpga.fanout_long_grounding COMMAND grounding_preemption long_grounding)
add_test(NAME fpga.fanout_remote_takeoff COMMAND grounding_preemption remote_takeoff)
add_test(NAME fpga.fanout_remote_takeoff_blocked COMMAND grounding_preemption remote_takeoff_blocked)
set_tests_properties(fpga.fanout_remote_takeoff fpga.fanout_remote_takeoff_blocked PROPERTIES TIMEOUT 30)
set_tests_properties(fpga.fanout_long_grounding PROPERTIES TIMEOUT 30)
add_test(NAME fpga.fanout_gap_owner_scan COMMAND grounding_preemption gap_owner_scan)
add_test(NAME fpga.fanout_retained_audit COMMAND grounding_preemption retained_audit)
add_test(NAME fpga.fanout_numeric_terminal COMMAND grounding_preemption numeric_terminal)
add_test(NAME fpga.resource_pin_audit COMMAND grounding_preemption resource_pin_audit)
set_tests_properties(fpga.fanout_numeric_terminal PROPERTIES TIMEOUT 30)
add_test(NAME fpga.fanout_retained_frontier COMMAND grounding_preemption retained_frontier)
add_test(NAME fpga.fanout_retained_frontier_capacity COMMAND grounding_preemption retained_frontier_capacity)
add_test(NAME fpga.fanout_retained_wide COMMAND grounding_preemption retained_wide)
add_test(NAME fpga.unbound_completion_frontier COMMAND grounding_preemption unbound_completion_frontier)
add_test(NAME fpga.unbound_deadline_frontier COMMAND grounding_preemption unbound_deadline_frontier)
add_test(NAME fpga.source_dominance_work COMMAND grounding_preemption source_dominance_work)
add_test(NAME fpga.docking_cut_checks COMMAND grounding_preemption docking_cut_checks)
add_test(NAME fpga.source_free_budget_escalation COMMAND grounding_preemption source_free_budget_escalation)
set_tests_properties(fpga.source_free_budget_escalation PROPERTIES TIMEOUT 30)
add_test(NAME fpga.source_grounding_budget_escalation COMMAND grounding_preemption source_grounding_budget_escalation)
set_tests_properties(fpga.source_grounding_budget_escalation PROPERTIES TIMEOUT 30)
add_test(NAME fpga.source_frontier_reuse COMMAND grounding_preemption source_frontier_reuse)
add_test(NAME fpga.gap_source_frontier_reuse COMMAND grounding_preemption gap_source_frontier_reuse)
set_tests_properties(fpga.gap_source_frontier_reuse PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
add_test(NAME fpga.moving_source_probe COMMAND grounding_preemption moving_source_probe)
set_tests_properties(fpga.moving_source_probe PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
add_test(NAME fpga.moving_fixed_local_cycle COMMAND grounding_preemption moving_fixed_local_cycle)
set_tests_properties(fpga.moving_fixed_local_cycle PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
add_test(NAME fpga.moving_deferred_fixed_wire COMMAND grounding_preemption moving_deferred_fixed_wire)
set_tests_properties(fpga.moving_deferred_fixed_wire PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
add_test(NAME fpga.moving_source_boundary_continuation COMMAND grounding_preemption moving_source_boundary_continuation)
set_tests_properties(fpga.moving_source_boundary_continuation PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
foreach(case source_candidate_inputs source_candidate_inputs_output_conflict source_current_wire source_two_ended source_resolved_local)
    add_test(NAME fpga.${case} COMMAND grounding_preemption ${case})
    set_tests_properties(fpga.${case} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
endforeach()
set_tests_properties(fpga.unbound_completion_frontier PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
set_tests_properties(fpga.unbound_deadline_frontier PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
set_tests_properties(fpga.fanout_retained_frontier fpga.fanout_retained_frontier_capacity fpga.fanout_retained_wide
    PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
set_tests_properties(fpga.fanout_gap_owner_scan PROPERTIES TIMEOUT 30)
add_test(NAME fpga.fanout_gap_owner_source COMMAND grounding_preemption gap_owner_source)
set_tests_properties(fpga.fanout_gap_owner_source PROPERTIES TIMEOUT 30)

target_link_libraries(grounding_preemption PUBLIC PNG::PNG)

add_executable(moving_test
    src/tests/fpga/moving_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(moving_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.moving COMMAND moving_test)
add_test(NAME fpga.bridge_preemption COMMAND moving_test bridge_preemption)
add_test(NAME fpga.fanout_search_stack COMMAND moving_test fanout_search_stack)
add_test(NAME fpga.whole_task_backtracking COMMAND moving_test whole_task_backtracking)
add_test(NAME fpga.whole_task_backtracking_blocked COMMAND moving_test whole_task_backtracking_blocked)
add_test(NAME fpga.whole_task_deep_backtracking COMMAND moving_test whole_task_deep_backtracking)
add_test(NAME fpga.whole_task_deep_backtracking_blocked COMMAND moving_test whole_task_deep_backtracking_blocked)
add_test(NAME fpga.whole_task_shared_prefix COMMAND moving_test whole_task_shared_prefix)
add_test(NAME fpga.whole_task_shared_prefix_blocked COMMAND moving_test whole_task_shared_prefix_blocked)
add_test(NAME fpga.moving_generic_prefix COMMAND moving_test moving_generic_prefix)
add_test(NAME fpga.moving_generic_prefix_blocked COMMAND moving_test moving_generic_prefix_blocked)
add_test(NAME fpga.moving_source_pin_backtrack COMMAND moving_test moving_source_pin_backtrack)
add_test(NAME fpga.moving_source_pin_backtrack_blocked COMMAND moving_test moving_source_pin_backtrack_blocked)
add_test(NAME fpga.moving_source_exit COMMAND moving_test moving_source_exit)
add_test(NAME fpga.moving_source_exit_blocked COMMAND moving_test moving_source_exit_blocked)
add_test(NAME fpga.moving_source_exit_prefers_branch COMMAND moving_test moving_source_exit_prefers_branch)
add_test(NAME fpga.moving_empty_binding COMMAND moving_test moving_empty_binding)
add_test(NAME fpga.moving_empty_binding_unbound_control COMMAND moving_test moving_empty_binding_unbound_control)
add_test(NAME fpga.moving_empty_binding_blocked COMMAND moving_test moving_empty_binding_blocked)
add_test(NAME fpga.moving_empty_binding_unbound_blocked COMMAND moving_test moving_empty_binding_unbound_blocked)
add_test(NAME fpga.moving_retired_binding COMMAND moving_test moving_retired_binding)
add_test(NAME fpga.moving_retired_binding_blocked COMMAND moving_test moving_retired_binding_blocked)
add_test(NAME fpga.fanout_tile_visits COMMAND moving_test fanout_tile_visits)
add_test(NAME fpga.moving_incomplete_sibling COMMAND moving_test incomplete_sibling)
add_test(NAME fpga.moving_unleased_source COMMAND moving_test unleased_source)
add_test(NAME fpga.moving_unleased_source_blocked COMMAND moving_test unleased_source_blocked)
add_test(NAME fpga.moving_detached_trigger COMMAND moving_test detached_trigger)
add_test(NAME fpga.moving_retained_search COMMAND moving_test moving_retained_search)
set_tests_properties(fpga.moving_retained_search PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_numeric_source_prefix COMMAND moving_test numeric_source_prefix)
add_test(NAME fpga.moving_branching_point COMMAND moving_test move_branching_point)
add_test(NAME fpga.moving_branch_takeoff_preemption COMMAND moving_test branch_takeoff_preemption)
set_tests_properties(fpga.moving_branch_takeoff_preemption PROPERTIES TIMEOUT 60)
set_tests_properties(fpga.moving_branching_point PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_gap_tail_anchor COMMAND moving_test gap_tail_anchor)
set_tests_properties(fpga.moving_gap_tail_anchor PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_shared_prefix_cleanup COMMAND moving_test shared_prefix_cleanup)
set_tests_properties(fpga.moving_shared_prefix_cleanup PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_shared_input COMMAND moving_test shared_input)
set_tests_properties(fpga.moving_shared_input PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_local_bridge COMMAND moving_test moving_local_bridge)
set_tests_properties(fpga.moving_local_bridge PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_fixed_destination COMMAND moving_test fixed_destination)
set_tests_properties(fpga.moving_fixed_destination PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_fixed_unbound COMMAND moving_test fixed_unbound)
set_tests_properties(fpga.moving_fixed_unbound PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_mapped_io_input COMMAND moving_test mapped_io_input)
add_test(NAME fpga.moving_nested_input_window COMMAND moving_test nested_input_window)
add_test(NAME fpga.moving_duplicate_input COMMAND moving_test duplicate_input)
add_test(NAME fpga.moving_stale_trigger COMMAND moving_test stale_trigger)
set_tests_properties(fpga.moving_duplicate_input fpga.moving_stale_trigger PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_forward_memory COMMAND moving_test forward_memory)
add_test(NAME fpga.moving_forward_blacklist COMMAND moving_test forward_blacklist)
add_test(NAME fpga.moving_backward_blacklist COMMAND moving_test backward_blacklist)
set_tests_properties(fpga.moving_forward_blacklist fpga.moving_backward_blacklist PROPERTIES TIMEOUT 60)
add_test(NAME fpga.moving_quiet_packing_replay COMMAND moving_test quiet_packing_replay)
set_tests_properties(fpga.moving_quiet_packing_replay PROPERTIES
    ENVIRONMENT "SCALEPNR_ROUTE_PACKING_TRACE=0" TIMEOUT 60)

target_link_libraries(moving_test PUBLIC PNG::PNG)

add_executable(packing_test
    src/tests/fpga/packing_test.cpp
    src/tests/fpga/incremental_grounding_packing.cpp
    src/tests/fpga/incremental_constant_packing.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(packing_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.packing COMMAND packing_test)
add_test(NAME fpga.moving_blacklist_packing COMMAND packing_test moving_blacklist)
set_tests_properties(fpga.moving_blacklist_packing PROPERTIES TIMEOUT 60)
add_test(NAME fpga.incremental_grounding_snapshot COMMAND packing_test incremental_grounding_snapshot)
add_test(NAME fpga.incremental_grounding_packing COMMAND packing_test incremental_grounding_packing)
set_tests_properties(fpga.incremental_grounding_snapshot fpga.incremental_grounding_packing PROPERTIES TIMEOUT 30)
add_test(NAME fpga.incremental_constant_snapshot COMMAND packing_test incremental_constant_snapshot)
add_test(NAME fpga.incremental_constant_packing COMMAND packing_test incremental_constant_packing)
set_tests_properties(fpga.incremental_constant_snapshot fpga.incremental_constant_packing PROPERTIES TIMEOUT 30)

get_target_property(PACKING_PERF_SOURCES packing_test SOURCES)
list(REMOVE_ITEM PACKING_PERF_SOURCES src/tests/fpga/packing_test.cpp)
get_target_property(PACKING_PERF_INCLUDES packing_test INCLUDE_DIRECTORIES)
add_executable(packing_perf src/tests/fpga/packing_perf.cpp ${PACKING_PERF_SOURCES})
target_include_directories(packing_perf PRIVATE ${PACKING_PERF_INCLUDES})
target_link_libraries(packing_perf PRIVATE $<IF:$<PLATFORM_ID:Windows>,stdc++exp.lib,stdc++exp>)
add_test(NAME fpga.packing_perf COMMAND packing_perf)

add_executable(passthrough_test
    src/tests/fpga/passthrough_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(passthrough_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.passthrough COMMAND passthrough_test)
add_test(NAME fpga.passthrough_global_control COMMAND passthrough_test global_control)
set_tests_properties(fpga.passthrough_global_control PROPERTIES TIMEOUT 30)

add_executable(clock_routing_test
    src/tests/fpga/clock_routing_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(clock_routing_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.clock_routing COMMAND clock_routing_test)

add_executable(angle_priority
    src/tests/fpga/angle_priority.cpp
    src/common/debug.cpp
    src/fpga/Crossbar.cpp
)

target_include_directories(angle_priority
    PRIVATE
        src/fpga
        src/common
        src/utils
)

add_test(NAME fpga.angle_priority COMMAND angle_priority)

add_executable(source_order_collision_test
    src/tests/fpga/source_order_collision.cpp
    src/common/debug.cpp
    src/fpga/Crossbar.cpp
)
target_include_directories(source_order_collision_test PRIVATE
    src/fpga src/common src/utils src/db libs/jsoncpp/include)
foreach(case snapshot dst_first local_first directions)
    add_test(NAME fpga.source_order_${case}
        COMMAND source_order_collision_test ${case})
    set_tests_properties(fpga.source_order_${case} PROPERTIES TIMEOUT 10)
endforeach()

add_executable(cb_names_test
    src/tests/fpga/cb_names_test.cpp
    src/tests/fpga/conn_names_test.cpp
    src/common/debug.cpp
    src/fpga/Crossbar.cpp
)

target_include_directories(cb_names_test
    PRIVATE
        src/fpga
        src/common
        src/utils
        src/db
)

add_test(NAME fpga.cb_names COMMAND cb_names_test)
add_test(NAME fpga.conn_names COMMAND cb_names_test --shared-connections)
set_tests_properties(fpga.conn_names PROPERTIES TIMEOUT 10)

add_executable(cb_sharing_test
    src/tests/fpga/cb_sharing_test.cpp
    src/common/debug.cpp
    src/fpga/Crossbar.cpp
)
target_link_libraries(cb_sharing_test PRIVATE $<IF:$<PLATFORM_ID:Windows>,stdc++exp.lib,stdc++exp>)
add_test(NAME fpga.cb_sharing COMMAND cb_sharing_test)

add_executable(dst_by_src_test
    src/tests/fpga/dst_by_src_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(dst_by_src_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.dst_by_src COMMAND dst_by_src_test)

add_executable(subtype_test
    src/tests/fpga/subtype_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/pnr/route/Docking.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(subtype_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

target_compile_definitions(subtype_test PRIVATE SCALEPNR_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_test(NAME fpga.subtypes COMMAND subtype_test)
set_tests_properties(fpga.subtypes PROPERTIES TIMEOUT 600 LABELS database)

add_executable(docking_test
    src/tests/fpga/docking_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/pnr/route/Docking.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(docking_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.docking COMMAND docking_test)

add_executable(backwards_resolve_test
    src/tests/fpga/backwards_resolve.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/pnr/route/Docking.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(backwards_resolve_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.backwards_resolve COMMAND backwards_resolve_test)

add_executable(pnr_db_test
    src/tests/fpga/pnr_db_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
)

target_include_directories(pnr_db_test
    PRIVATE
        src/db
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.pnr_db COMMAND pnr_db_test)

add_executable(visualizer_test
    src/tests/fpga/visualizer_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
)

target_include_directories(visualizer_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_test(NAME fpga.visualizer COMMAND visualizer_test)
target_link_libraries(visualizer_test PUBLIC PNG::PNG)

add_executable(device_format_test
    src/tests/fpga/device_format_test.cpp
    src/common/debug.cpp
)

target_include_directories(device_format_test
    PRIVATE
        src/common
        src/db
        src/fpga
        src/utils
        libs/jsoncpp/include
)

add_test(NAME fpga.device_format COMMAND device_format_test)

add_executable(outline_test
    src/tests/fpga/outline_test.cpp
)

target_include_directories(outline_test
    PRIVATE
        src/pnr/outline
)

add_test(NAME fpga.outline COMMAND outline_test)

add_executable(pin_lookup_cache_test src/tests/fpga/pin_lookup_cache_test.cpp)
target_include_directories(pin_lookup_cache_test PRIVATE src/common src/fpga src/utils)
add_test(NAME fpga.pin_lookup_cache COMMAND pin_lookup_cache_test)

add_executable(arena_test
    src/tests/fpga/arena_test.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(arena_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.arena COMMAND arena_test)
add_test(NAME fpga.routing_fail_fast COMMAND arena_test fail_fast)
add_test(NAME fpga.target_deadend_commit COMMAND arena_test target_deadend)
add_test(NAME fpga.immediate_docking COMMAND arena_test immediate_docking)
add_test(NAME fpga.fanout_source COMMAND arena_test fanout_source)
add_test(NAME fpga.fanout_spread COMMAND arena_test fanout_spread)
add_test(NAME fpga.fanout_destination COMMAND arena_test fanout_destination)

target_link_libraries(arena_test PUBLIC PNG::PNG)

add_executable(routing_puzzle_test
    src/tests/puzzle/routing_puzzle.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(routing_puzzle_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

# The puzzle ladder adds one routing pressure at each level. Keep the
# Basic-only case first so trunk regressions remain separate from fanout bugs.
add_test(NAME puzzle.routing_basic
    COMMAND routing_puzzle_test 10000 50 50 50 0)
add_test(NAME puzzle.routing
    COMMAND routing_puzzle_test 10000 50 50 50 10)
add_test(NAME puzzle.routing_90
    COMMAND routing_puzzle_test 10000 50 50 90 10)
add_test(NAME puzzle.routing_50k
    COMMAND routing_puzzle_test 50000 100 100 50 4)
add_test(NAME puzzle.routing_50k_90
    COMMAND routing_puzzle_test 50000 100 100 90 4)
add_test(NAME puzzle.routing_fanout_forest
    COMMAND routing_puzzle_test 50000 100 100 50 80 12000)
set_tests_properties(puzzle.routing_basic PROPERTIES TIMEOUT 300)
set_tests_properties(puzzle.routing PROPERTIES TIMEOUT 300)
set_tests_properties(puzzle.routing_90 PROPERTIES TIMEOUT 300)
set_tests_properties(puzzle.routing_50k PROPERTIES TIMEOUT 300)
set_tests_properties(puzzle.routing_50k_90 PROPERTIES TIMEOUT 300)
set_tests_properties(puzzle.routing_fanout_forest PROPERTIES TIMEOUT 300)
set_tests_properties(
    puzzle.routing_basic puzzle.routing puzzle.routing_90
    puzzle.routing_50k puzzle.routing_50k_90 puzzle.routing_fanout_forest
    PROPERTIES LABELS "puzzle")
target_link_libraries(routing_puzzle_test PUBLIC PNG::PNG)

set(PLACING_PUZZLE_SOURCES
    src/tests/puzzle/placing_puzzle.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

function(configure_placing_puzzle_target target)
    add_executable(${target} ${PLACING_PUZZLE_SOURCES})
    target_include_directories(${target} PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
    )
    target_link_libraries(${target} PUBLIC PNG::PNG)
endfunction()

configure_placing_puzzle_target(placing_puzzle_test)
configure_placing_puzzle_target(placing_puzzle_100_test)

# The 100x100 puzzle exercises 160,000 cells. Keep only that stress-test
# executable optimized in a Debug build; the established smaller regressions
# retain their original compilation and deterministic placement trajectory.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(placing_puzzle_100_test PRIVATE -O2)
endif()

add_test(NAME puzzle.placing COMMAND placing_puzzle_test 12 50)
set_tests_properties(puzzle.placing PROPERTIES TIMEOUT 300 LABELS "puzzle")
add_test(NAME puzzle.placing_50x50 COMMAND placing_puzzle_test 50 50)
set_tests_properties(
    puzzle.placing_50x50
    PROPERTIES TIMEOUT 300 LABELS "puzzle")
add_test(NAME puzzle.placing_100x100 COMMAND placing_puzzle_100_test 100 50)
set_tests_properties(
    puzzle.placing_100x100
    PROPERTIES TIMEOUT 600 LABELS "puzzle")
add_test(NAME puzzle.placing_100x100_2clocks COMMAND placing_puzzle_100_test 100 50 2)
set_tests_properties(
    puzzle.placing_100x100_2clocks
    PROPERTIES TIMEOUT 600 LABELS "puzzle;multiclock")
# Fixed optimization deadlines make search quality sensitive to contention.
# Keep these stress tests isolated without relaxing their timing assertions.
set_tests_properties(
    puzzle.placing_100x100 puzzle.placing_100x100_2clocks
    PROPERTIES RUN_SERIAL TRUE)
# Fast coverage of clock-domain generation and retention through the same flow.
add_test(NAME puzzle.placing_2clocks COMMAND placing_puzzle_100_test 12 50 2)
set_tests_properties(
    puzzle.placing_2clocks
    PROPERTIES TIMEOUT 60 LABELS "puzzle;multiclock")

add_executable(repair_prefixes_test
    src/tests/fpga/repair_prefixes.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(repair_prefixes_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.repair_prefixes COMMAND repair_prefixes_test)
target_link_libraries(repair_prefixes_test PUBLIC PNG::PNG)

add_executable(place_timing_test
    src/tests/placing/place_timing.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(place_timing_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.timing COMMAND place_timing_test)
add_test(NAME placing.input_routing_pressure COMMAND place_timing_test routing_pressure)
set_tests_properties(placing.input_routing_pressure PROPERTIES TIMEOUT 60 LABELS "placing;regression")
target_link_libraries(place_timing_test PUBLIC PNG::PNG)

# Exercise the real Tcl clock commands and the same engines used by placement
# and routing. Keep clock regressions independent of large puzzle runtimes.
get_target_property(MULTI_CLOCK_SOURCES place_timing_test SOURCES)
list(REMOVE_ITEM MULTI_CLOCK_SOURCES src/tests/placing/place_timing.cpp)
add_executable(multi_clock_test src/tests/placing/multi_clock.cpp
    src/tcl/create_clock_cmd.cpp src/tcl/get_ports_cmd.cpp ${MULTI_CLOCK_SOURCES})
get_target_property(MULTI_CLOCK_INCLUDES scalepnr INCLUDE_DIRECTORIES)
target_include_directories(multi_clock_test PRIVATE ${MULTI_CLOCK_INCLUDES})
target_link_libraries(multi_clock_test PRIVATE PNG::PNG tcl_imported)
add_dependencies(multi_clock_test tcl)
if(WIN32)
    target_link_libraries(multi_clock_test PRIVATE stdc++exp.lib ws2_32.lib Netapi32.lib Userenv.lib)
else()
    target_link_libraries(multi_clock_test PRIVATE stdc++exp pthread dl)
endif()
foreach(mode tcl timing estimate placement routing routing_conflict incremental_routing flow packing ambiguous_clock sorting swapping data_routing)
    add_test(NAME multi_clock.${mode} COMMAND multi_clock_test ${mode})
    set_tests_properties(multi_clock.${mode} PROPERTIES TIMEOUT 60)
endforeach()

add_executable(swapping_placing_test
    src/tests/placing/swapping_placing.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(swapping_placing_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.swapping COMMAND swapping_placing_test)
target_link_libraries(swapping_placing_test PUBLIC PNG::PNG)

add_executable(sorting_placing_test
    src/tests/placing/sorting_placing.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(sorting_placing_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.sorting COMMAND sorting_placing_test)
target_link_libraries(sorting_placing_test PUBLIC PNG::PNG)

add_executable(estimate_design_test
    src/tests/placing/estimate_design.cpp
    src/common/debug.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/pnr/EstimateDesign.cpp
)

target_include_directories(estimate_design_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.estimate_design COMMAND estimate_design_test)
target_link_libraries(estimate_design_test PUBLIC PNG::PNG)

add_executable(outline_design_test
    src/tests/placing/outline_design.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(outline_design_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.outline_design COMMAND outline_design_test directional)
add_test(NAME placing.outline_design.memory COMMAND outline_design_test memory)
add_test(NAME placing.outline_design.100Kcells COMMAND outline_design_test 100Kcells)
target_link_libraries(outline_design_test PUBLIC PNG::PNG)

add_executable(outline_attractor_test
    src/tests/placing/outline_attractor.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/fpga/Visualizer.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/place/PlaceSorting.cpp
    src/pnr/place/PlaceSwapping.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(outline_attractor_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.outline_attractor COMMAND outline_attractor_test)
add_test(NAME placing.outline_attractor.path_span COMMAND outline_attractor_test span)
add_test(NAME placing.outline_attractor.comb_direction COMMAND outline_attractor_test comb)
target_link_libraries(outline_attractor_test PUBLIC PNG::PNG)

get_target_property(PNR_TEST_SOURCES moving_test SOURCES)
list(REMOVE_ITEM PNR_TEST_SOURCES src/tests/fpga/moving_test.cpp)
get_target_property(PNR_TEST_INCLUDES moving_test INCLUDE_DIRECTORIES)
add_executable(pnr_test src/tests/pnr/pnr.cpp src/tests/placing/place_routing.cpp src/pnr/pnr.cpp ${PNR_TEST_SOURCES})
target_include_directories(pnr_test PRIVATE ${PNR_TEST_INCLUDES})
target_link_libraries(pnr_test PUBLIC PNG::PNG)
add_test(NAME pnr.plan COMMAND pnr_test planner)
add_test(NAME placing.routing_pressure COMMAND pnr_test routing_pressure)
add_test(NAME pnr.routing_aware COMMAND pnr_test routing_aware)
add_test(NAME pnr.incremental COMMAND pnr_test incremental)
add_test(NAME pnr.failure COMMAND pnr_test failure)
add_test(NAME pnr.clock COMMAND pnr_test clock)
add_test(NAME pnr.clock_batch COMMAND pnr_test clock_batch)
add_test(NAME pnr.clock_cleanup COMMAND pnr_test clock_cleanup)
add_test(NAME pnr.clock_fail_fast COMMAND pnr_test clock_fail_fast)
set_tests_properties(pnr.clock_cleanup PROPERTIES TIMEOUT 60)
set_tests_properties(pnr.clock_batch PROPERTIES TIMEOUT 60)
add_test(NAME pnr.constants COMMAND pnr_test constants)
add_test(NAME pnr.mesh COMMAND pnr_test mesh)
add_test(NAME pnr.mesh_fanout COMMAND pnr_test mesh_fanout)
add_test(NAME pnr.moving_deferred COMMAND pnr_test moving_deferred)
add_test(NAME pnr.run_budget COMMAND pnr_test run_budget)
set_tests_properties(pnr.run_budget PROPERTIES TIMEOUT 60)
set_tests_properties(pnr.moving_deferred PROPERTIES TIMEOUT 60)
add_test(NAME pnr.tcl COMMAND scalepnr ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/pnr/pnr.tcl)
set_tests_properties(pnr.tcl PROPERTIES TIMEOUT 30
    ENVIRONMENT "TCL_LIBRARY=${CMAKE_CURRENT_SOURCE_DIR}/libs/tcl8.6.14/library")
set_tests_properties(pnr.plan pnr.incremental pnr.failure pnr.clock pnr.constants pnr.mesh pnr.mesh_fanout PROPERTIES TIMEOUT 60)

if(WIN32)
target_link_libraries(pnr_test PUBLIC stdc++exp.lib)
target_link_libraries(routing_test PUBLIC stdc++exp.lib)
target_link_libraries(grounding_preemption PUBLIC stdc++exp.lib)
target_link_libraries(moving_test PUBLIC stdc++exp.lib)
target_link_libraries(packing_test PUBLIC stdc++exp.lib)
target_link_libraries(passthrough_test PUBLIC stdc++exp.lib)
target_link_libraries(clock_routing_test PUBLIC stdc++exp.lib)
target_link_libraries(angle_priority PUBLIC stdc++exp.lib)
target_link_libraries(source_order_collision_test PUBLIC stdc++exp.lib)
target_link_libraries(cb_names_test PUBLIC stdc++exp.lib)
target_link_libraries(dst_by_src_test PUBLIC stdc++exp.lib)
target_link_libraries(subtype_test PUBLIC stdc++exp.lib)
target_link_libraries(docking_test PUBLIC stdc++exp.lib)
target_link_libraries(backwards_resolve_test PUBLIC stdc++exp.lib)
target_link_libraries(pnr_db_test PUBLIC stdc++exp.lib)
target_link_libraries(visualizer_test PUBLIC stdc++exp.lib)
target_link_libraries(arena_test PUBLIC stdc++exp.lib)
target_link_libraries(routing_puzzle_test PUBLIC stdc++exp.lib)
target_link_libraries(placing_puzzle_test PUBLIC stdc++exp.lib)
target_link_libraries(placing_puzzle_100_test PUBLIC stdc++exp.lib)
target_link_libraries(repair_prefixes_test PUBLIC stdc++exp.lib)
target_link_libraries(outline_test PUBLIC stdc++exp.lib)
target_link_libraries(pin_lookup_cache_test PUBLIC stdc++exp.lib)
target_link_libraries(place_timing_test PUBLIC stdc++exp.lib)
target_link_libraries(swapping_placing_test PUBLIC stdc++exp.lib)
target_link_libraries(sorting_placing_test PUBLIC stdc++exp.lib)
target_link_libraries(estimate_design_test PUBLIC stdc++exp.lib)
target_link_libraries(outline_design_test PUBLIC stdc++exp.lib)
target_link_libraries(outline_attractor_test PUBLIC stdc++exp.lib)
else()
target_link_libraries(pnr_test PUBLIC stdc++exp)
target_link_libraries(routing_test PUBLIC stdc++exp)
target_link_libraries(grounding_preemption PUBLIC stdc++exp)
target_link_libraries(moving_test PUBLIC stdc++exp)
target_link_libraries(packing_test PUBLIC stdc++exp)
target_link_libraries(passthrough_test PUBLIC stdc++exp)
target_link_libraries(clock_routing_test PUBLIC stdc++exp)
target_link_libraries(angle_priority PUBLIC stdc++exp)
target_link_libraries(source_order_collision_test PUBLIC stdc++exp)
target_link_libraries(cb_names_test PUBLIC stdc++exp)
target_link_libraries(dst_by_src_test PUBLIC stdc++exp)
target_link_libraries(subtype_test PUBLIC stdc++exp)
target_link_libraries(docking_test PUBLIC stdc++exp)
target_link_libraries(backwards_resolve_test PUBLIC stdc++exp)
target_link_libraries(pnr_db_test PUBLIC stdc++exp)
target_link_libraries(visualizer_test PUBLIC stdc++exp)
target_link_libraries(arena_test PUBLIC stdc++exp)
target_link_libraries(routing_puzzle_test PUBLIC stdc++exp)
target_link_libraries(placing_puzzle_test PUBLIC stdc++exp)
target_link_libraries(placing_puzzle_100_test PUBLIC stdc++exp)
target_link_libraries(repair_prefixes_test PUBLIC stdc++exp)
target_link_libraries(outline_test PUBLIC stdc++exp)
target_link_libraries(pin_lookup_cache_test PUBLIC stdc++exp)
target_link_libraries(place_timing_test PUBLIC stdc++exp)
target_link_libraries(swapping_placing_test PUBLIC stdc++exp)
target_link_libraries(sorting_placing_test PUBLIC stdc++exp)
target_link_libraries(estimate_design_test PUBLIC stdc++exp)
target_link_libraries(outline_design_test PUBLIC stdc++exp)
target_link_libraries(outline_attractor_test PUBLIC stdc++exp)
endif()
