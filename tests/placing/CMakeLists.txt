option(SCALEPNR_SYNTH_PLACING_TESTS "Enable vendor-independent synthesized placement tests" ON)
if(NOT SCALEPNR_SYNTH_PLACING_TESTS)
    return()
endif()
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_program(SCALEPNR_YOSYS NAMES yosys HINTS "${CMAKE_SOURCE_DIR}/.conda/bin")
if(NOT SCALEPNR_YOSYS)
    message(FATAL_ERROR "Yosys not found. Install yosys or set SCALEPNR_YOSYS=/path/to/yosys; use SCALEPNR_SYNTH_PLACING_TESTS=OFF to disable this suite.")
endif()
set(sources ${PLACING_PUZZLE_SOURCES})
list(REMOVE_ITEM sources src/tests/puzzle/placing_puzzle.cpp)
list(TRANSFORM sources PREPEND "${CMAKE_SOURCE_DIR}/")
add_executable(placing_synth_test placing_synth.cpp ${sources} "${CMAKE_SOURCE_DIR}/src/pnr/pnr.cpp")
# Source properties set in the root directory do not propagate to sources
# compiled by this subdirectory; honor the same configured placer settings.
set_property(SOURCE "${CMAKE_SOURCE_DIR}/src/pnr/place/PlaceDesign.cpp" APPEND PROPERTY
    COMPILE_DEFINITIONS
        SCALEPNR_PLACE_REDISTRIBUTION_STEP=${SCALEPNR_PLACE_REDISTRIBUTION_STEP}
        SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR=${SCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR})
target_link_libraries(placing_synth_test PRIVATE PNG::PNG)
if(WIN32)
    target_link_libraries(placing_synth_test PRIVATE stdc++exp.lib)
else()
    target_link_libraries(placing_synth_test PRIVATE stdc++exp)
endif()
add_test(NAME placing.synth.yosys COMMAND "${SCALEPNR_YOSYS}" -V)
set_tests_properties(placing.synth.yosys PROPERTIES FIXTURES_SETUP placing_yosys LABELS "placing;synthesis" TIMEOUT 30)
foreach(size 1k 5k 10k 30k 50k 80k 100k)
    file(GLOB designs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${size}/*.sv")
    foreach(design IN LISTS designs)
        get_filename_component(family "${design}" NAME_WE)
        add_test(NAME placing.synth.${size}.${family}
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/run.py"
                --yosys "${SCALEPNR_YOSYS}" --source "${design}"
                --placer $<TARGET_FILE:placing_synth_test>
                --work "${CMAKE_CURRENT_BINARY_DIR}/${size}/${family}")
        set_tests_properties(placing.synth.${size}.${family} PROPERTIES
            FIXTURES_REQUIRED placing_yosys LABELS "placing;synthesis;${size}"
            TIMEOUT 2400 PROCESSORS 2)
    endforeach()
endforeach()
