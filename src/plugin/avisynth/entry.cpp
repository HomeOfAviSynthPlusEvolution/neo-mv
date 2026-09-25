#include "plugin/bridges.hpp"
#include <avisynth.h>
#include <dualsynth/avisynth/video_bridge.hpp>

const AVS_Linkage* AVS_linkage = nullptr;

namespace neo_mv::ds2::avs {
namespace av = ds::avisynth;

// Native AviSynth arrays keep the same parameter positions as VapourSynth.
// The common descriptors remain unchanged for existing VS callers.
template <class Base>
struct Adapter : Base {
  static constexpr bool forward_audio = true;
  static constexpr av::MtMode avs_mt_mode = av::MtMode::NiceFilter;
  static ds::FilterDescriptor descriptor() {
    auto d = Base::descriptor();
    for (auto& p : d.params) {
      p.avs_enabled = true;
      if (p.is_array)
        p.avs_array_binding = ds::AvisynthArrayBinding::Native;
    }
    return d;
  }
};

std::size_t index(const ds::FilterDescriptor& d, const char* name) {
  for (std::size_t i = 0; i < d.params.size(); ++i)
    if (d.params[i].name == name)
      return i;
  throw std::logic_error("unknown AviSynth parameter");
}

// Normalize scalar shorthand to native arrays before passing arguments to DS2.
struct Call {
  std::vector<AVSValue> values;
  Call(const AVSValue& args, const ds::FilterDescriptor& d) : values(d.params.size()) {
    for (std::size_t i = 0; i < values.size(); ++i) {
      values[i] = args.IsArray() ? (i < std::size_t(args.ArraySize()) ? args[int(i)] : AVSValue())
                                 : (i == 0 ? args : AVSValue());
      if (d.params[i].is_array && values[i].Defined() && !values[i].IsArray()) {
        const AVSValue scalar = values[i];
        values[i] = AVSValue(&scalar, 1);
      }
    }
  }
  AVSValue arguments() const { return AVSValue(values.data(), static_cast<int>(values.size())); }
};

template <class Function>
AVSValue guarded(IScriptEnvironment* env, Function&& function) {
  try {
    env->CheckVersion(11);
    return function();
  } catch (const AvisynthError&) {
    throw;
  } catch (const std::exception& e) {
    env->ThrowError("neo-mv: %s", e.what());
  } catch (...) {
    env->ThrowError("neo-mv: AviSynth creation failed");
  }
  return {};
}

template <class Base>
AVSValue __cdecl create(AVSValue args, void* user_data, IScriptEnvironment* env) {
  return guarded(env, [&] {
    const auto d = Adapter<Base>::descriptor();
    Call call(args, d);
    if constexpr (std::is_same_v<Base, RenderBridge<true>>) {
      const auto radius = reinterpret_cast<std::intptr_t>(user_data);
      const auto vectors = call.values[index(d, "vectors")];
      require(radius == 0 || (vectors.IsArray() && vectors.ArraySize() == 2 * radius),
              "named Degrain requires exactly 2R vector members");
    }
    return av::create_video_filter_bridge<Adapter<Base>>(call.arguments(), env);
  });
}

ds::FilterDescriptor many_descriptor() {
  auto d = Adapter<Bridge<Operation::Analyse>>::descriptor();
  d.params.insert(d.params.begin() + index(d, "prefix"), {"radius", ds::ParamType::Integer});
  return d;
}

AVSValue __cdecl many(AVSValue args, void*, IScriptEnvironment* env) {
  return guarded(env, [&] {
    const auto d = many_descriptor();
    Call call(args, d);
    const auto parsed = unwrap(av::read_params(call.arguments(), d));
    const Params params{parsed};
    const int radius = params.integer("radius", 1), step = params.integer("delta", 1);
    require(radius > 0 && step > 0 && radius <= SHRT_MAX / 2 && std::int64_t(radius) * step <= INT32_MAX,
            "invalid AnalyseMany radius or delta product");
    call.values.erase(call.values.begin() + index(d, "radius"));
    std::vector<std::vector<AVSValue>> members;
    std::vector<AVSValue> calls;
    members.reserve(std::size_t(radius) * 2);
    calls.reserve(std::size_t(radius) * 2);
    for (int r = 1; r <= radius; ++r)
      for (int sign : {1, -1}) {
        members.push_back(call.values);
        members.back()[index(d, "delta")] = AVSValue(r * step * sign);
        calls.push_back(AVSValue(members.back().data(), static_cast<int>(members.back().size())));
      }
    return av::create_video_filter_bundle<Adapter<Bridge<Operation::Analyse>>>({calls.data(), calls.size()}, env);
  });
}

ds::FilterDescriptor recalculate_descriptor() {
  auto d = Adapter<Bridge<Operation::Recalculate>>::descriptor();
  d.params[index(d, "vectors")].is_array = true;
  d.params[index(d, "vectors")].avs_array_binding = ds::AvisynthArrayBinding::Native;
  return d;
}

AVSValue __cdecl recalculate(AVSValue args, void*, IScriptEnvironment* env) {
  return guarded(env, [&] {
    const auto d = recalculate_descriptor();
    Call call(args, d);
    const auto slot = index(d, "vectors");
    const auto vectors = call.values[slot];
    require(vectors.IsArray() && vectors.ArraySize() > 0, "Recalculate requires nonempty vectors");
    std::vector<std::vector<AVSValue>> members;
    std::vector<AVSValue> calls;
    members.reserve(vectors.ArraySize());
    calls.reserve(vectors.ArraySize());
    for (int i = 0; i < vectors.ArraySize(); ++i) {
      members.push_back(call.values);
      members.back()[slot] = vectors[i];
      calls.push_back(AVSValue(members.back().data(), static_cast<int>(members.back().size())));
    }
    return av::create_video_filter_bundle<Adapter<Bridge<Operation::Recalculate>>>({calls.data(), calls.size()}, env);
  });
}

template <class Base>
AVSValue __cdecl create_depan(AVSValue args, void*, IScriptEnvironment* env) {
  return guarded(env, [&] {
    const auto d = Adapter<Base>::descriptor();
    Call call(args, d);
    const auto info = call.values[index(d, "info")];
    const bool show = info.Defined() && info.IsBool() && info.AsBool();
    const auto base = av::create_video_filter_bridge<Adapter<Base>>(call.arguments(), env);
    if (!show)
      return base;
    AVSValue values[] = {base, AVSValue(Base::diagnostic_property)};
    const char* names[] = {nullptr, "props"};
    return env->Invoke("propShow", AVSValue(values, 2), names);
  });
}

AVSValue __cdecl kernel_info(AVSValue, void*, IScriptEnvironment* env) {
  return guarded(env, [&] {
    const auto fft = estimate_fft_profile();
    AVSValue fields[] = {AVSValue(env->SaveString(selected_backend_name())),
                         AVSValue(env->SaveString(selected_target_name())),
                         AVSValue(env->SaveString(depan::estimate::fft_profile_name(fft))),
                         AVSValue(depan::estimate::fft_lanes(fft))};
    return AVSValue(fields, 4);
  });
}

using ApplyFunc = AVSValue (__cdecl *)(AVSValue, void*, IScriptEnvironment*);
void add(IScriptEnvironment* env, const std::string& name, const ds::FilterDescriptor& d, ApplyFunc callback,
         void* data = nullptr) {
  const auto signature = unwrap(ds::make_avisynth_signature(d));
  env->AddFunction(env->SaveString(name.c_str()), env->SaveString(signature.c_str()), callback, data);
}
template <class Base>
void add(IScriptEnvironment* env) {
  add(env, std::string("neo_mv_") + Base::Core::name, Adapter<Base>::descriptor(), create<Base>);
}
template <class Base>
void add_depan(IScriptEnvironment* env) {
  add(env, std::string("neo_mv_") + Base::Core::name, Adapter<Base>::descriptor(), create_depan<Base>);
}
} // namespace neo_mv::ds2::avs

#if defined(_WIN32)
#define NEO_MV_AVS_EXPORT extern "C" __declspec(dllexport)
#else
#define NEO_MV_AVS_EXPORT extern "C" __attribute__((visibility("default")))
#endif

NEO_MV_AVS_EXPORT const char* __stdcall AvisynthPluginInit3(IScriptEnvironment* env, const AVS_Linkage* linkage) {
  AVS_linkage = linkage;
  using namespace neo_mv::ds2;
  using namespace neo_mv::ds2::avs;
  try {
    env->CheckVersion(11);
    add<Bridge<Operation::Super>>(env);
    add<Bridge<Operation::Analyse>>(env);
    add(env, "neo_mv_AnalyseMany", many_descriptor(), many);
    add(env, "neo_mv_Recalculate", recalculate_descriptor(), recalculate);
    add<Bridge<Operation::SCDetection>>(env);
    add<RenderBridge<false>>(env);
    add<RenderBridge<true>>(env);
    for (std::intptr_t radius = 1; radius <= 25; ++radius)
      add(env, "neo_mv_Degrain" + std::to_string(radius), Adapter<RenderBridge<true>>::descriptor(),
          create<RenderBridge<true>>, reinterpret_cast<void*>(radius));
    add<FlowBridge>(env);
    add<TemporalBridge<TemporalKind::Inter>>(env);
    add<TemporalBridge<TemporalKind::FPS>>(env);
    add<TemporalBridge<TemporalKind::Blur>>(env);
    add<MaskBridge<neo_mv::MaskKind::VectorLength>>(env);
    add<MaskBridge<neo_mv::MaskKind::SAD>>(env);
    add<MaskBridge<neo_mv::MaskKind::Occlusion>>(env);
    add_depan<DepanBridge<true>>(env);
    add_depan<DepanBridge<false>>(env);
    add_depan<DepanEstimateBridge>(env);
    add_depan<DepanStabiliseBridge>(env);
    add(env, "neo_mv_KernelInfo", {"KernelInfo", {}}, kernel_info);
    return "neo-mv AviSynth interface";
  } catch (const AvisynthError&) {
    throw;
  } catch (const std::exception& e) {
    env->ThrowError("neo-mv: %s", e.what());
  } catch (...) {
    env->ThrowError("neo-mv: AviSynth registration failed");
  }
  return nullptr;
}
