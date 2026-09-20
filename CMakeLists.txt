cmake_minimum_required(VERSION 3.20)
project(neo_mv LANGUAGES CXX)

# Only the standalone view header is needed; do not configure the DS2 runtime.
# Override FETCHCONTENT_SOURCE_DIR_DUALSYNTH2 to use an existing checkout.
include(FetchContent)
FetchContent_Declare(dualsynth2
  GIT_REPOSITORY https://github.com/HomeOfAviSynthPlusEvolution/dualsynth2.git
  GIT_TAG 9b8ca4b1fee43f2f38a2c9b751e9ba5a836acd36
  SOURCE_SUBDIR neo_mv_header_only
)
# SOURCE_SUBDIR deliberately has no CMakeLists.txt: populate without adding
# DS2's runtime, SDK discovery, acceptance plugins or tests to this build.
FetchContent_MakeAvailable(dualsynth2)

add_library(neo_mv_core INTERFACE)
target_compile_features(neo_mv_core INTERFACE cxx_std_17)
target_include_directories(neo_mv_core INTERFACE src)
target_include_directories(neo_mv_core SYSTEM INTERFACE "${dualsynth2_SOURCE_DIR}/include")
if(MSVC)
  target_compile_options(neo_mv_core INTERFACE /fp:strict)
else()
  target_compile_options(neo_mv_core INTERFACE -fno-fast-math -ffp-contract=off)
endif()

include(CTest)
if(BUILD_TESTING)
  foreach(operator border_extension pyramid subpixel block_metric block_sampling search)
    add_executable(neo_mv_${operator}_tests tests/core/test_${operator}.cpp)
    target_link_libraries(neo_mv_${operator}_tests PRIVATE neo_mv_core)
    if(MSVC)
      target_compile_options(neo_mv_${operator}_tests PRIVATE /W4)
    else()
      target_compile_options(neo_mv_${operator}_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME core.${operator} COMMAND neo_mv_${operator}_tests)
  endforeach()
endif()
