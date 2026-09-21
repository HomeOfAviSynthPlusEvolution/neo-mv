cmake_minimum_required(VERSION 3.24)
project(neo_mv VERSION 0.1.0 LANGUAGES CXX)

option(NEO_MV_BUILD_VAPOURSYNTH "Build the VapourSynth plugin" ON)
option(NEO_MV_ENABLE_HIGHWAY "Build Highway pixel kernels and runtime dispatch" ON)
# The core-only build uses the standalone view header without the DS2 runtime.
# Override FETCHCONTENT_SOURCE_DIR_DUALSYNTH2 to use an existing checkout.
include(FetchContent)
if(NEO_MV_BUILD_VAPOURSYNTH)
  set(DS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(DS_BUILD_ACCEPTANCE_PLUGIN OFF CACHE BOOL "" FORCE)
  set(DS_ENABLE_AVISYNTH OFF CACHE BOOL "" FORCE)
  set(DS_ENABLE_VAPOURSYNTH ON CACHE BOOL "" FORCE)
  set(_neo_mv_ds2_subdir .)
else()
  set(_neo_mv_ds2_subdir neo_mv_header_only)
endif()
FetchContent_Declare(dualsynth2
  GIT_REPOSITORY https://github.com/HomeOfAviSynthPlusEvolution/dualsynth2.git
  GIT_TAG 9ee756ab6213fa0ca8505a7438cb19ac5ec7d1d5
  SOURCE_SUBDIR "${_neo_mv_ds2_subdir}"
)
FetchContent_MakeAvailable(dualsynth2)
unset(_neo_mv_ds2_subdir)

add_library(neo_mv_core INTERFACE)
target_compile_features(neo_mv_core INTERFACE cxx_std_17)
target_include_directories(neo_mv_core INTERFACE src)
target_include_directories(neo_mv_core SYSTEM INTERFACE "${dualsynth2_SOURCE_DIR}/include")
if(MSVC)
  target_compile_options(neo_mv_core INTERFACE /fp:strict)
else()
  target_compile_options(neo_mv_core INTERFACE -fno-fast-math -ffp-contract=off)
endif()

FetchContent_Declare(pocketfft
  GIT_REPOSITORY https://github.com/mreineck/pocketfft.git
  GIT_TAG c90e55b3d529f8efa40ed01a20de22405f45fc65
  SOURCE_SUBDIR neo_mv_header_only)
FetchContent_MakeAvailable(pocketfft)
option(NEO_MV_HWY_SCALAR_ONLY "Build only Highway's portable scalar target" OFF)
foreach(profile scalar native)
  add_library(neo_mv_fft_${profile} OBJECT src/core/depan/estimate_fft_backend.cpp)
  target_link_libraries(neo_mv_fft_${profile} PRIVATE neo_mv_core)
  target_include_directories(neo_mv_fft_${profile} SYSTEM PRIVATE "${pocketfft_SOURCE_DIR}")
  set_target_properties(neo_mv_fft_${profile} PROPERTIES POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
endforeach()
target_compile_definitions(neo_mv_fft_scalar PRIVATE NEO_MV_FFT_NATIVE=0)
target_compile_definitions(neo_mv_fft_native PRIVATE NEO_MV_FFT_NATIVE=1)
if(NOT NEO_MV_ENABLE_HIGHWAY OR NEO_MV_HWY_SCALAR_ONLY)
  target_compile_definitions(neo_mv_fft_native PRIVATE POCKETFFT_NO_VECTORS)
endif()
add_library(neo_mv_fft STATIC src/core/depan/estimate_fft.cpp)
target_sources(neo_mv_fft PRIVATE $<TARGET_OBJECTS:neo_mv_fft_scalar> $<TARGET_OBJECTS:neo_mv_fft_native>)
target_link_libraries(neo_mv_fft PUBLIC neo_mv_core)
target_include_directories(neo_mv_fft SYSTEM PRIVATE "${pocketfft_SOURCE_DIR}")
set_target_properties(neo_mv_fft PROPERTIES POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)

if(NEO_MV_ENABLE_HIGHWAY)
  set(HWY_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
  set(HWY_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(HWY_ENABLE_CONTRIB OFF CACHE BOOL "" FORCE)
  set(HWY_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  set(HWY_FORCE_STATIC_LIBS ON CACHE BOOL "" FORCE)
  FetchContent_Declare(highway
    URL https://github.com/google/highway/archive/refs/tags/1.4.0.tar.gz
    URL_HASH SHA256=e72241ac9524bb653ae52ced768b508045d4438726a303f10181a38f764a453c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(highway)
  add_library(neo_mv_highway STATIC src/highway/rows.cpp src/highway/render_rows.cpp src/highway/mask_rows.cpp src/highway/interpolation_rows.cpp src/highway/depan_rows.cpp src/highway/depan_estimate.cpp)
  target_link_libraries(neo_mv_highway PUBLIC neo_mv_core hwy)
  target_compile_definitions(neo_mv_highway PUBLIC HWY_COMPILE_ALL_ATTAINABLE)
  set_target_properties(neo_mv_highway hwy PROPERTIES POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
  if(NEO_MV_HWY_SCALAR_ONLY)
    target_compile_definitions(neo_mv_highway PUBLIC HWY_COMPILE_ONLY_SCALAR)
  endif()
endif()

option(NEO_MV_BUILD_BENCHMARKS "Build manually invoked benchmarks (never registered with CTest)" OFF)
if(NEO_MV_BUILD_BENCHMARKS)
  if(NOT NEO_MV_ENABLE_HIGHWAY)
    message(FATAL_ERROR "Manual kernel benchmarks require NEO_MV_ENABLE_HIGHWAY")
  endif()
  add_executable(neo_mv_benchmark benchmarks/kernels.cpp)
  target_link_libraries(neo_mv_benchmark PRIVATE neo_mv_highway)
endif()

include(CTest)
if(NEO_MV_BUILD_VAPOURSYNTH)
  include(cmake/VapourSynth.cmake)
  add_library(neo_mv_vs MODULE src/plugin/vapoursynth/entry.cpp
    src/kernels/selection.cpp)
  target_link_libraries(neo_mv_vs PRIVATE neo_mv_core neo_mv_fft DualSynth::dualsynth)
  target_compile_definitions(neo_mv_vs PRIVATE NEO_MV_ENABLE_HIGHWAY=$<BOOL:${NEO_MV_ENABLE_HIGHWAY}>)
  if(NEO_MV_ENABLE_HIGHWAY)
    target_link_libraries(neo_mv_vs PRIVATE neo_mv_highway)
  endif()
  target_include_directories(neo_mv_vs SYSTEM PRIVATE "${NEO_MV_VS_INCLUDE_DIR}")
  if(MSVC)
    target_compile_options(neo_mv_vs PRIVATE /W4)
  else()
    target_compile_options(neo_mv_vs PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  if(MINGW)
    target_link_options(neo_mv_vs PRIVATE -static -static-libgcc -static-libstdc++)
  endif()
  set_target_properties(neo_mv_vs PROPERTIES PREFIX "" OUTPUT_NAME "neo-mv"
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
  if(BUILD_TESTING)
    option(NEO_MV_TEST_VAPOURSYNTH "Run tests using an installed vspipe of the matching architecture" ON)
    find_program(NEO_MV_VSPIPE_EXECUTABLE vspipe)
    if(NEO_MV_TEST_VAPOURSYNTH AND NOT NEO_MV_VSPIPE_EXECUTABLE)
      message(FATAL_ERROR "VS acceptance requested but vspipe is missing; set NEO_MV_VSPIPE_EXECUTABLE or explicitly disable NEO_MV_TEST_VAPOURSYNTH")
    endif()
    if(NEO_MV_TEST_VAPOURSYNTH AND NEO_MV_VSPIPE_EXECUTABLE)
      add_test(NAME vs.acceptance COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/acceptance.vpy" --)
      set_tests_properties(vs.acceptance PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE)
      set_tests_properties(vs.acceptance PROPERTIES ENVIRONMENT "NEO_MV_KERNEL=scalar")
      add_test(NAME vs.render COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/render.vpy" --)
      set_tests_properties(vs.render PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
        ENVIRONMENT "NEO_MV_KERNEL=scalar" FAIL_REGULAR_EXPRESSION "Core freed but")
      add_test(NAME vs.mask COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/mask.vpy" --)
      set_tests_properties(vs.mask PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
        ENVIRONMENT "NEO_MV_KERNEL=scalar" FAIL_REGULAR_EXPRESSION "Core freed but")
      add_test(NAME vs.flow COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/flow.vpy" --)
      set_tests_properties(vs.flow PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
        ENVIRONMENT "NEO_MV_KERNEL=scalar" FAIL_REGULAR_EXPRESSION "Core freed but")
      add_test(NAME vs.interpolation COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/interpolation.vpy" --)
      set_tests_properties(vs.interpolation PROPERTIES TIMEOUT 180 LABELS "spec" RUN_SERIAL TRUE
        ENVIRONMENT "NEO_MV_KERNEL=scalar" FAIL_REGULAR_EXPRESSION "Core freed but")
      add_test(NAME vs.depan COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/depan.vpy" --)
      set_tests_properties(vs.depan PROPERTIES TIMEOUT 180 LABELS "spec" RUN_SERIAL TRUE
        ENVIRONMENT "NEO_MV_KERNEL=scalar" FAIL_REGULAR_EXPRESSION "Core freed but")
      add_test(NAME vs.depan_estimate COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
        --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/depan_estimate.vpy" --)
      set_tests_properties(vs.depan_estimate PROPERTIES TIMEOUT 180 LABELS "spec" RUN_SERIAL TRUE
        ENVIRONMENT "NEO_MV_KERNEL=scalar" FAIL_REGULAR_EXPRESSION "Core freed but")
      if(NEO_MV_ENABLE_HIGHWAY)
        add_test(NAME vs.acceptance.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>" --arg "kernel=highway"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/acceptance.vpy" --)
        set_tests_properties(vs.acceptance.highway PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway")
        add_test(NAME vs.render.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/render.vpy" --)
        set_tests_properties(vs.render.highway PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway" FAIL_REGULAR_EXPRESSION "Core freed but")
        add_test(NAME vs.mask.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/mask.vpy" --)
        set_tests_properties(vs.mask.highway PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway" FAIL_REGULAR_EXPRESSION "Core freed but")
        add_test(NAME vs.interpolation.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/interpolation.vpy" --)
        set_tests_properties(vs.interpolation.highway PROPERTIES TIMEOUT 180 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway" FAIL_REGULAR_EXPRESSION "Core freed but")
        add_test(NAME vs.depan.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/depan.vpy" --)
        set_tests_properties(vs.depan.highway PROPERTIES TIMEOUT 180 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway" FAIL_REGULAR_EXPRESSION "Core freed but")
        add_test(NAME vs.depan_estimate.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/depan_estimate.vpy" --)
        set_tests_properties(vs.depan_estimate.highway PROPERTIES TIMEOUT 180 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway" FAIL_REGULAR_EXPRESSION "Core freed but")
        add_test(NAME vs.flow.highway COMMAND "${NEO_MV_VSPIPE_EXECUTABLE}"
          --arg "plugin=$<TARGET_FILE:neo_mv_vs>"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/vapoursynth/flow.vpy" --)
        set_tests_properties(vs.flow.highway PROPERTIES TIMEOUT 120 LABELS "spec" RUN_SERIAL TRUE
          ENVIRONMENT "NEO_MV_KERNEL=highway" FAIL_REGULAR_EXPRESSION "Core freed but")
      endif()
    endif()
  endif()
endif()
if(BUILD_TESTING)
  if(NEO_MV_ENABLE_HIGHWAY)
    add_executable(neo_mv_highway_tests tests/highway/differential.cpp)
    target_link_libraries(neo_mv_highway_tests PRIVATE neo_mv_highway)
    add_test(NAME core.highway COMMAND neo_mv_highway_tests)
    add_executable(neo_mv_highway_depan_estimate_tests tests/highway/depan_estimate.cpp)
    target_link_libraries(neo_mv_highway_depan_estimate_tests PRIVATE neo_mv_highway neo_mv_fft)
    add_test(NAME core.highway.depan_estimate COMMAND neo_mv_highway_depan_estimate_tests)
    set_tests_properties(core.highway.depan_estimate PROPERTIES LABELS "spec" TIMEOUT 60)
    set_tests_properties(core.highway PROPERTIES LABELS "spec" TIMEOUT 180 RUN_SERIAL TRUE)
    foreach(operator render render_ops interpolation depan)
      add_executable(neo_mv_highway_${operator}_tests tests/highway/${operator}.cpp)
      target_link_libraries(neo_mv_highway_${operator}_tests PRIVATE neo_mv_highway)
      add_test(NAME core.highway.${operator} COMMAND neo_mv_highway_${operator}_tests)
      set_tests_properties(core.highway.${operator} PROPERTIES LABELS "spec" TIMEOUT 180)
    endforeach()
    add_executable(neo_mv_highway_render_frame_tests tests/core/test_render_frame.cpp)
    target_link_libraries(neo_mv_highway_render_frame_tests PRIVATE neo_mv_highway)
    target_compile_definitions(neo_mv_highway_render_frame_tests PRIVATE NEO_MV_TEST_HIGHWAY=1)
    add_test(NAME core.highway.render_frame COMMAND neo_mv_highway_render_frame_tests)
    set_tests_properties(core.highway.render_frame PROPERTIES LABELS "spec" TIMEOUT 180)
    foreach(operator mask spec_mask_scores spec_grid_resampling flow)
      add_executable(neo_mv_highway_${operator}_tests tests/highway/${operator}.cpp)
      target_link_libraries(neo_mv_highway_${operator}_tests PRIVATE neo_mv_highway)
      add_test(NAME core.highway.${operator} COMMAND neo_mv_highway_${operator}_tests)
      set_tests_properties(core.highway.${operator} PROPERTIES LABELS "spec" TIMEOUT 180)
    endforeach()
    add_executable(neo_mv_highway_mask_frame_tests tests/core/test_mask_frame.cpp)
    target_link_libraries(neo_mv_highway_mask_frame_tests PRIVATE neo_mv_highway)
    target_compile_definitions(neo_mv_highway_mask_frame_tests PRIVATE NEO_MV_TEST_HIGHWAY=1)
    add_test(NAME core.highway.mask_frame COMMAND neo_mv_highway_mask_frame_tests)
    set_tests_properties(core.highway.mask_frame PROPERTIES LABELS "spec" TIMEOUT 180)
    foreach(operator flow_dense flow_frame interpolation_frame)
      add_executable(neo_mv_highway_${operator}_tests tests/core/test_${operator}.cpp)
      target_link_libraries(neo_mv_highway_${operator}_tests PRIVATE neo_mv_highway)
      target_compile_definitions(neo_mv_highway_${operator}_tests PRIVATE NEO_MV_TEST_HIGHWAY=1)
      add_test(NAME core.highway.${operator} COMMAND neo_mv_highway_${operator}_tests)
      set_tests_properties(core.highway.${operator} PROPERTIES LABELS "spec" TIMEOUT 180)
    endforeach()
  endif()
  foreach(operator border_extension pyramid subpixel block_metric block_sampling search prediction analysis_field scene_classification recalculate analyse super render_reference render_sampling overlap compensation degrain render_frame grid_resampling mask_input mask_scores mask_frame flow_dense flow_sampling flow_frame interpolation_input interpolation_dense interpolation_pixel interpolation_sampling interpolation_rate interpolation_blur interpolation_frame depan_transform depan_analysis depan_sampling depan_estimate_geometry depan_estimate_motion depan_estimate_image)
    add_executable(neo_mv_${operator}_tests tests/core/test_${operator}.cpp)
    target_link_libraries(neo_mv_${operator}_tests PRIVATE neo_mv_core)
    if(MSVC)
      target_compile_options(neo_mv_${operator}_tests PRIVATE /W4)
    else()
      target_compile_options(neo_mv_${operator}_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME core.${operator} COMMAND neo_mv_${operator}_tests)
    set_tests_properties(core.${operator} PROPERTIES LABELS "spec")
  endforeach()
  find_package(Threads REQUIRED)
  add_executable(neo_mv_depan_estimate_fft_tests tests/core/test_depan_estimate_fft.cpp)
  target_link_libraries(neo_mv_depan_estimate_fft_tests PRIVATE neo_mv_fft Threads::Threads)
  add_test(NAME core.depan_estimate_fft COMMAND neo_mv_depan_estimate_fft_tests)
  set_tests_properties(core.depan_estimate_fft PROPERTIES LABELS "spec" RUN_SERIAL TRUE TIMEOUT 60)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(NAME harness.protocol COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/blackbox/test_protocol.py")
    set_tests_properties(harness.protocol PROPERTIES LABELS "spec")
  endif()
  option(NEO_MV_TEST_BLACKBOX "Compare against installed VS R79 / MVUtensils 8 binaries" OFF)
  if(NEO_MV_TEST_BLACKBOX)
    if(NOT NEO_MV_BUILD_VAPOURSYNTH)
      message(FATAL_ERROR "Black-box tests require NEO_MV_BUILD_VAPOURSYNTH")
    endif()
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME blackbox.smoke COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/blackbox/run.py"
      --plugin "$<TARGET_FILE:neo_mv_vs>"
      --output-root "${CMAKE_CURRENT_BINARY_DIR}/blackbox")
    set_tests_properties(blackbox.smoke PROPERTIES LABELS "blackbox" RUN_SERIAL TRUE TIMEOUT 1200)
    if(NEO_MV_ENABLE_HIGHWAY AND NOT NEO_MV_HWY_SCALAR_ONLY)
      add_test(NAME blackbox.highway COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/blackbox/run.py" --neo-kernel highway
        --plugin "$<TARGET_FILE:neo_mv_vs>"
        --output-root "${CMAKE_CURRENT_BINARY_DIR}/blackbox")
      set_tests_properties(blackbox.highway PROPERTIES LABELS "blackbox" RUN_SERIAL TRUE TIMEOUT 1200)
    endif()
  endif()
endif()
