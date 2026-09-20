# Accept installed SDKs with either flat or namespaced include directories.
set(NEO_MV_VS_SDK "" CACHE PATH "Optional VapourSynth SDK directory")
find_path(NEO_MV_VS_SDK_INCLUDE_DIR NAMES vapoursynth/VapourSynth4.h VapourSynth4.h
  HINTS "${NEO_MV_VS_SDK}" "${NEO_MV_VS_SDK}/include"
    "$ENV{VAPOURSYNTH_SDK}" "$ENV{VAPOURSYNTH_SDK}/include"
    "${DS_VAPOURSYNTH_INCLUDE_DIR}")
if(NOT NEO_MV_VS_SDK_INCLUDE_DIR)
  FetchContent_Declare(neo_mv_vapoursynth_sdk
    URL https://github.com/vapoursynth/vapoursynth/archive/refs/tags/R73.zip
    URL_HASH SHA256=7c6b1eb2ec4aeae078675a29cf5e77c6ca1ab8b1dd322677c66e1ca6b76c511d
    SOURCE_SUBDIR neo_mv_headers_only)
  FetchContent_MakeAvailable(neo_mv_vapoursynth_sdk)
  set(NEO_MV_VS_SDK_INCLUDE_DIR "${neo_mv_vapoursynth_sdk_SOURCE_DIR}/include")
endif()
if(EXISTS "${NEO_MV_VS_SDK_INCLUDE_DIR}/vapoursynth/VapourSynth4.h")
  set(NEO_MV_VS_INCLUDE_DIR "${NEO_MV_VS_SDK_INCLUDE_DIR}")
else()
  set(NEO_MV_VS_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
  configure_file("${NEO_MV_VS_SDK_INCLUDE_DIR}/VapourSynth4.h"
    "${NEO_MV_VS_INCLUDE_DIR}/vapoursynth/VapourSynth4.h" COPYONLY)
endif()
