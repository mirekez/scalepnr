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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
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

add_executable(grounding_preemption
    src/tests/fpga/grounding_preemption.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.grounding_preemption COMMAND grounding_preemption)

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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.moving COMMAND moving_test)

target_link_libraries(moving_test PUBLIC PNG::PNG)

add_executable(packing_test
    src/tests/fpga/packing_test.cpp
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

add_executable(cb_names_test
    src/tests/fpga/cb_names_test.cpp
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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME fpga.arena COMMAND arena_test)

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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
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

add_executable(placing_puzzle_test
    src/tests/puzzle/placing_puzzle.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
    src/common/debug.cpp
    src/fpga/Device.cpp
    src/fpga/Crossbar.cpp
    src/fpga/Tile.cpp
    src/fpga/Wire.cpp
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
    src/pnr/route/Docking.cpp
    src/pnr/route/RouteClocks.cpp
    src/pnr/route/RouteVCC.cpp
    src/pnr/route/RouteDesign.cpp
    ${SCALEPNR_RE2_ABSL_SOURCES}
)

target_include_directories(placing_puzzle_test
    PRIVATE
        src/clocks
        src/fpga
        src/rtl
        src/common
        src/db
        src/utils
        src/tech
        src/pnr
        src/pnr/estimate
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME puzzle.placing COMMAND placing_puzzle_test 12 50)
set_tests_properties(puzzle.placing PROPERTIES TIMEOUT 300 LABELS "puzzle")
add_test(NAME puzzle.placing_50x50 COMMAND placing_puzzle_test 50 50)
set_tests_properties(
    puzzle.placing_50x50
    PROPERTIES TIMEOUT 300 LABELS "puzzle")
target_link_libraries(placing_puzzle_test PUBLIC PNG::PNG)

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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.timing COMMAND place_timing_test)
target_link_libraries(place_timing_test PUBLIC PNG::PNG)

add_executable(estimate_design_test
    src/tests/placing/estimate_design.cpp
    src/common/debug.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/pnr/estimate/EstimateDesign.cpp
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
        src/pnr/estimate
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
    src/rtl/Design.cpp
    src/rtl/getInsts.cpp
    src/rtl/getConns.cpp
    src/rtl/Conn.cpp
    src/rtl/Inst.cpp
    src/rtl/PrintDesign.cpp
    src/clocks/Clocks.cpp
    src/clocks/Timings.cpp
    src/tech/Tech.cpp
    src/pnr/estimate/EstimateDesign.cpp
    src/pnr/outline/OutlineDesign.cpp
    src/pnr/place/PlaceDesign.cpp
    src/pnr/place/PlaceTiming.cpp
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
        src/pnr/estimate
        src/pnr/outline
        src/pnr/place
        src/pnr/route
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
        libs/re2
        libs/abseil-cpp
)

add_test(NAME placing.outline_design COMMAND outline_design_test directional)
add_test(NAME placing.outline_design.100Kcells COMMAND outline_design_test 100Kcells)
target_link_libraries(outline_design_test PUBLIC PNG::PNG)

if(WIN32)
target_link_libraries(routing_test PUBLIC stdc++exp.lib)
target_link_libraries(grounding_preemption PUBLIC stdc++exp.lib)
target_link_libraries(moving_test PUBLIC stdc++exp.lib)
target_link_libraries(packing_test PUBLIC stdc++exp.lib)
target_link_libraries(passthrough_test PUBLIC stdc++exp.lib)
target_link_libraries(clock_routing_test PUBLIC stdc++exp.lib)
target_link_libraries(angle_priority PUBLIC stdc++exp.lib)
target_link_libraries(cb_names_test PUBLIC stdc++exp.lib)
target_link_libraries(dst_by_src_test PUBLIC stdc++exp.lib)
target_link_libraries(subtype_test PUBLIC stdc++exp.lib)
target_link_libraries(docking_test PUBLIC stdc++exp.lib)
target_link_libraries(backwards_resolve_test PUBLIC stdc++exp.lib)
target_link_libraries(pnr_db_test PUBLIC stdc++exp.lib)
target_link_libraries(arena_test PUBLIC stdc++exp.lib)
target_link_libraries(routing_puzzle_test PUBLIC stdc++exp.lib)
target_link_libraries(placing_puzzle_test PUBLIC stdc++exp.lib)
target_link_libraries(repair_prefixes_test PUBLIC stdc++exp.lib)
target_link_libraries(outline_test PUBLIC stdc++exp.lib)
target_link_libraries(place_timing_test PUBLIC stdc++exp.lib)
target_link_libraries(estimate_design_test PUBLIC stdc++exp.lib)
target_link_libraries(outline_design_test PUBLIC stdc++exp.lib)
else()
target_link_libraries(routing_test PUBLIC stdc++exp)
target_link_libraries(grounding_preemption PUBLIC stdc++exp)
target_link_libraries(moving_test PUBLIC stdc++exp)
target_link_libraries(packing_test PUBLIC stdc++exp)
target_link_libraries(passthrough_test PUBLIC stdc++exp)
target_link_libraries(clock_routing_test PUBLIC stdc++exp)
target_link_libraries(angle_priority PUBLIC stdc++exp)
target_link_libraries(cb_names_test PUBLIC stdc++exp)
target_link_libraries(dst_by_src_test PUBLIC stdc++exp)
target_link_libraries(subtype_test PUBLIC stdc++exp)
target_link_libraries(docking_test PUBLIC stdc++exp)
target_link_libraries(backwards_resolve_test PUBLIC stdc++exp)
target_link_libraries(pnr_db_test PUBLIC stdc++exp)
target_link_libraries(arena_test PUBLIC stdc++exp)
target_link_libraries(routing_puzzle_test PUBLIC stdc++exp)
target_link_libraries(placing_puzzle_test PUBLIC stdc++exp)
target_link_libraries(repair_prefixes_test PUBLIC stdc++exp)
target_link_libraries(outline_test PUBLIC stdc++exp)
target_link_libraries(place_timing_test PUBLIC stdc++exp)
target_link_libraries(estimate_design_test PUBLIC stdc++exp)
target_link_libraries(outline_design_test PUBLIC stdc++exp)
endif()
