#pragma once
#include "filters/phase1.hpp"
#include "filters/phase2.hpp"

namespace neo_mv::ds2 {
inline ds::ParamSpec parameter(const char* name, ds::ParamType type, bool required = false, bool array = false) {
  return {name, type, {}, required, array, true, false};
}
inline ds::FilterDescriptor descriptor(Operation op) {
  using P = ds::ParamType;
  ds::FilterDescriptor d;
  auto add = [&](const char* n, P t = P::Integer, bool required = false, bool array = false) {
    d.params.push_back(parameter(n, t, required, array));
  };
  if (op == Operation::Super) {
    d.name = "Super";
    add("clip", P::Clip, true);
    add("blksize", P::Integer, true, true);
    add("overlap", P::Integer, true, true);
    add("pad", P::Integer, false, true);
    add("onelevel", P::Boolean);
    add("sharp");
    add("rfilter");
    add("pel");
    add("pelclip", P::Clip);
  } else if (op == Operation::Analyse) {
    d.name = "Analyse";
    add("super", P::Clip, true);
    add("blksize", P::Integer, false, true);
    for (auto n : {"levels", "search", "searchparam", "pelsearch", "mvlambda"})
      add(n);
    add("chroma", P::Boolean);
    add("delta");
    add("lsad");
    add("plevel");
    add("globalmv", P::Boolean);
    add("pnew");
    add("pzero");
    add("pglobal");
    add("overlap", P::Integer, false, true);
    add("badsad");
    add("badrange");
    add("meander", P::Boolean);
    add("trymany");
    add("fields", P::Boolean);
    add("tff", P::Boolean);
    add("satd", P::Boolean);
  } else if (op == Operation::Recalculate) {
    d.name = "Recalculate";
    add("super", P::Clip, true);
    add("vectors", P::Clip, true);
    add("thsad");
    add("smooth", P::Boolean);
    add("blksize", P::Integer, false, true);
    add("search");
    add("searchparam");
    add("mvlambda");
    add("chroma", P::Boolean);
    add("pnew");
    add("overlap", P::Integer, false, true);
    add("meander", P::Boolean);
    add("fields", P::Boolean);
    add("tff", P::Boolean);
    add("satd", P::Boolean);
  } else {
    d.name = "SCDetection";
    add("clip", P::Clip, true);
    add("vectors", P::Clip, true);
    add("thscd1");
    add("thscd2", P::Float);
  }
  add("prefix", P::String);
  return d;
}
inline constexpr char super_signature[] =
    "clip:vnode;blksize:int[]:empty;overlap:int[]:empty;pad:int[]:opt:empty;onelevel:int:opt;sharp:int:opt;rfilter:int:"
    "opt;pel:int:opt;pelclip:vnode:opt;prefix:data:opt;";
#define NEO_MV_ANALYSE_PARAMETERS                                                                                      \
  "super:vnode;blksize:int[]:opt:empty;levels:int:opt;search:int:opt;searchparam:int:opt;pelsearch:int:opt;mvlambda:"  \
  "int:opt;"                                                                                                           \
  "chroma:int:opt;delta:int:opt;lsad:int:opt;plevel:int:opt;globalmv:int:opt;pnew:int:opt;pzero:int:opt;pglobal:int:"  \
  "opt;"                                                                                                               \
  "overlap:int[]:opt:empty;badsad:int:opt;badrange:int:opt;meander:int:opt;trymany:int:opt;fields:int:opt;tff:int:"    \
  "opt;satd:int:opt;"
inline constexpr char analyse_signature[] = NEO_MV_ANALYSE_PARAMETERS "prefix:data:opt;";
inline constexpr char many_signature[] = NEO_MV_ANALYSE_PARAMETERS "radius:int:opt;prefix:data:opt;";
#undef NEO_MV_ANALYSE_PARAMETERS
inline constexpr char recalculate_signature[] =
    "super:vnode;vectors:vnode[];thsad:int:opt;smooth:int:opt;blksize:int[]:opt:empty;search:int:opt;searchparam:int:"
    "opt;mvlambda:int:opt;chroma:int:opt;pnew:int:opt;overlap:int[]:opt:empty;meander:int:opt;fields:int:opt;tff:int:"
    "opt;satd:int:opt;prefix:data:opt;";
inline constexpr char scene_signature[] = "clip:vnode;vectors:vnode;thscd1:int:opt;thscd2:float:opt;prefix:data:opt;";
template <Operation Op>
struct Bridge {
  using Core = Filter<Op>;
  static constexpr const char* vs_name = Core::name;
  static constexpr const char* vs_signature = Op == Operation::Super         ? super_signature
                                              : Op == Operation::Analyse     ? analyse_signature
                                              : Op == Operation::Recalculate ? recalculate_signature
                                                                             : scene_signature;
  // Complete DS2's bridge concept, but no AVS entry point is built or registered.
  static constexpr const char* avs_name = "";
  static constexpr const char* avs_signature = "";
  static constexpr const char* missing_input_error = "neo-mv: missing required video node";
  static constexpr const char* vs_format_error = "neo-mv: fixed DS2-compatible video format required";
  static constexpr const char* avs_format_error = vs_format_error;
  static constexpr std::size_t parity_source_index = 0;
  static constexpr bool forward_audio = false;
  static ds::FilterDescriptor descriptor() { return ds2::descriptor(Op); }
  static bool accepts_video_format(const ds::VideoFormat& format) {
    auto supported = ds::is_supported_video_format(format);
    return supported.has_value() && supported.value();
  }
};

inline constexpr char compensate_signature[] =
    "clip:vnode;super:vnode;vectors:vnode;thsad:int:opt;fields:int:opt;time:float:opt;thscd1:int:opt;"
    "thscd2:float:opt;tff:int:opt;prefix:data:opt;";
inline constexpr char degrain_signature[] =
    "clip:vnode;super:vnode;vectors:vnode[];thsad:int[]:opt:empty;thsad2:int[]:opt:empty;"
    "planes:int[]:opt:empty;limit:float[]:opt:empty;thscd1:int:opt;thscd2:float:opt;weights:int[]:opt:empty;prefix:"
    "data:opt;";
template <bool Degrain>
struct RenderBridge : Bridge<Operation::Super> {
  using Core = RenderFilter<Degrain>;
  static constexpr const char* vs_name = Core::name;
  static constexpr const char* vs_signature = Degrain ? degrain_signature : compensate_signature;
  static ds::FilterDescriptor descriptor() {
    using P = ds::ParamType;
    ds::FilterDescriptor d;
    d.name = Core::name;
    auto add = [&](const char* n, P t, bool required = false, bool array = false) {
      d.params.push_back(parameter(n, t, required, array));
    };
    add("clip", P::Clip, true);
    add("super", P::Clip, true);
    add("vectors", P::Clip, true, Degrain);
    add("thsad", P::Integer, false, Degrain);
    if constexpr (Degrain) {
      add("thsad2", P::Integer, false, true);
      add("planes", P::Integer, false, true);
      add("limit", P::Float, false, true);
    } else {
      add("fields", P::Boolean);
      add("time", P::Float);
    }
    add("thscd1", P::Integer);
    add("thscd2", P::Float);
    if constexpr (Degrain)
      add("weights", P::Integer, false, true);
    else
      add("tff", P::Boolean);
    add("prefix", P::String);
    return d;
  }
};
} // namespace neo_mv::ds2
