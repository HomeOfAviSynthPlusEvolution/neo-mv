if(WIN32 AND NOT MSVC)
  message(FATAL_ERROR "The AviSynth C++ interface on Windows requires an MSVC-compatible compiler (MSVC or clang-cl). Use -DNEO_MV_BUILD_AVISYNTH=OFF for a VapourSynth-only build with MinGW.")
endif()

set(NEO_MV_AVS_SDK "" CACHE PATH "Optional AviSynth+ SDK directory")
find_path(NEO_MV_AVS_INCLUDE_DIR NAMES avisynth.h
  HINTS "${NEO_MV_AVS_SDK}" "${NEO_MV_AVS_SDK}/include"
    "${NEO_MV_AVS_SDK}/avs_core/include" "$ENV{AVISYNTH_SDK}"
    "$ENV{AVISYNTH_SDK}/include" "${DS_AVISYNTH_INCLUDE_DIR}")
if(NOT NEO_MV_AVS_INCLUDE_DIR)
  FetchContent_Declare(neo_mv_avisynth_sdk
    GIT_REPOSITORY https://github.com/AviSynth/AviSynthPlus.git
    GIT_TAG 5c82777b374bdef16e13007a11e77d735ac1e4eb
    SOURCE_SUBDIR neo_mv_headers_only)
  FetchContent_MakeAvailable(neo_mv_avisynth_sdk)
  set(NEO_MV_AVS_INCLUDE_DIR "${neo_mv_avisynth_sdk_SOURCE_DIR}/avs_core/include")
endif()
