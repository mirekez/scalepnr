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
    src/pnr/route/RouteDesign.cpp
    src/tcl/tcl_pnr.cpp
    src/tcl/get_tiles_cmd.cpp
    src/tcl/get_ports_cmd.cpp
    src/tcl/create_clock_cmd.cpp
    src/tcl/check_timing_cmd.cpp
    src/tcl/load_design_cmd.cpp
    src/tcl/open_design_cmd.cpp
    src/tcl/place_design_cmd.cpp
    src/tcl/route_design_cmd.cpp
    src/tcl/print_design_cmd.cpp
    src/tcl/set_property_cmd.cpp
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
