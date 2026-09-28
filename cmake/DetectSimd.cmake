# Use Highway's target/toolchain checks for every architecture, including RVV
# and LSX/LASX. A hardware baseline is required because software fallback
# targets are excluded from the SIMD build.
set(_neo_mv_disabled_targets "(HWY_SCALAR|HWY_EMU128)")
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # GCC crashes while optimizing the SVE2 Degrain kernels. This mask has no
  # effect on other architectures and is shared by the probe and all users.
  set(_neo_mv_disabled_targets "(HWY_SCALAR|HWY_EMU128|HWY_ALL_SVE)")
endif()

function(neo_mv_detect_simd highway_source result)
  include(CheckCXXSourceCompiles)
  include(CMakePushCheckState)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_INCLUDES "${highway_source}")
  set(CMAKE_REQUIRED_DEFINITIONS "-DHWY_DISABLED_TARGETS=${_neo_mv_disabled_targets}")
  # No executable or target CPU is needed, including when cross-compiling.
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(CMAKE_BUILD_TYPE AND NOT CMAKE_TRY_COMPILE_CONFIGURATION)
    set(CMAKE_TRY_COMPILE_CONFIGURATION "${CMAKE_BUILD_TYPE}")
  endif()
  # Re-evaluate after changes to target flags or the Highway configuration.
  unset(NEO_MV_HAS_HWY_BASELINE CACHE)
  check_cxx_source_compiles([=[
    #include "hwy/detect_targets.h"
    #if !(HWY_ENABLED_BASELINE & ~(HWY_SCALAR | HWY_EMU128))
    #error No supported hardware SIMD baseline
    #endif
    #if HWY_TARGETS & (HWY_SCALAR | HWY_EMU128)
    #error Software SIMD targets must not be generated
    #endif
    #include "hwy/highway.h"
    void probe(float* p) {
      const hwy::HWY_NAMESPACE::ScalableTag<float> d;
      hwy::HWY_NAMESPACE::StoreU(hwy::HWY_NAMESPACE::Zero(d), d, p);
    }
  ]=] NEO_MV_HAS_HWY_BASELINE)
  set(${result} ${NEO_MV_HAS_HWY_BASELINE} PARENT_SCOPE)
  cmake_pop_check_state()
endfunction()
