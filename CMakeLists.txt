cmake_minimum_required(VERSION 3.20)
project(neo_mv LANGUAGES CXX)

# Only the standalone view header is needed; do not configure the DS2 runtime.
# Override FETCHCONTENT_SOURCE_DIR_DUALSYNTH2 to use an existing checkout.
include(FetchContent)
FetchContent_Declare(dualsynth2
  GIT_REPOSITORY https://github.com/HomeOfAviSynthPlusEvolution/dualsynth2.git
  GIT_TAG c9ecec7e812728f6c677b5ef4b4b2e6274137196
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
  add_executable(neo_mv_core_tests tests/core/test_border_extension.cpp)
  target_link_libraries(neo_mv_core_tests PRIVATE neo_mv_core)
  if(MSVC)
    target_compile_options(neo_mv_core_tests PRIVATE /W4)
  else()
    target_compile_options(neo_mv_core_tests PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_test(NAME core.border_extension COMMAND neo_mv_core_tests)
endif()
