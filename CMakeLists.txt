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
    src/gear/Device.cpp
    src/tcl/tcl_pnr.cpp
    src/tcl/get_tiles_cmd.cpp
)

add_executable(tilegrid_tool
    db/tilegrid_tool.cpp
    libs/jsoncpp/src/lib_json/json_reader.cpp
    libs/jsoncpp/src/lib_json/json_value.cpp
    libs/jsoncpp/src/lib_json/json_writer.cpp
)

include_directories(
    PRIVATE
        src/gear
        src/common
        src/io
        src/utils
        src/tcl
        libs/tcl8.6.14/generic
        libs/tcl8.6.14/win
        libs/jsoncpp/include
        libs/jsoncpp/src/lib_json
)

add_library(tcl_imported STATIC IMPORTED)
set_target_properties(tcl_imported PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libs/tcl8.6.14/libtcl.a)
target_link_libraries(scalepnr PRIVATE tcl_imported)
add_dependencies(scalepnr tcl)

target_link_libraries(scalepnr PUBLIC ws2_32.lib)
target_link_libraries(scalepnr PUBLIC Netapi32.lib)
target_link_libraries(scalepnr PUBLIC Userenv.lib)
target_link_libraries(scalepnr PUBLIC stdc++exp.lib)
target_link_options(scalepnr PRIVATE -static-libgcc -static-libstdc++)

target_link_libraries(tilegrid_tool PUBLIC ws2_32.lib)
target_link_libraries(tilegrid_tool PUBLIC Netapi32.lib)
target_link_libraries(tilegrid_tool PUBLIC Userenv.lib)
target_link_libraries(tilegrid_tool PUBLIC stdc++exp.lib)
target_link_options(tilegrid_tool PRIVATE -static-libgcc -static-libstdc++)
