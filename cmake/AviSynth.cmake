set(NEO_MV_AVS_SDK "" CACHE PATH "Optional AviSynth+ SDK directory")
find_path(NEO_MV_AVS_INCLUDE_DIR NAMES avisynth_c.h
  HINTS "${NEO_MV_AVS_SDK}" "${NEO_MV_AVS_SDK}/include"
    "${NEO_MV_AVS_SDK}/avs_core/include" "$ENV{AVISYNTH_SDK}"
    "$ENV{AVISYNTH_SDK}/include" "${DS_AVISYNTH_INCLUDE_DIR}")
if(NOT NEO_MV_AVS_INCLUDE_DIR)
  FetchContent_Declare(neo_mv_avisynth_sdk
    GIT_REPOSITORY https://github.com/AviSynth/AviSynthPlus.git
    GIT_TAG 001cfd68b3ef80479e9a0aa45235fee13b9dd1e8
    SOURCE_SUBDIR neo_mv_headers_only)
  FetchContent_MakeAvailable(neo_mv_avisynth_sdk)
  set(NEO_MV_AVS_INCLUDE_DIR "${neo_mv_avisynth_sdk_SOURCE_DIR}/avs_core/include")
endif()
