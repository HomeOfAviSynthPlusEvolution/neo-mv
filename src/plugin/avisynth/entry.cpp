#include "plugin/bridges.hpp"
// Declare the C API before DS2 enables AVSC_NO_DECLSPEC; DS2 uses decltype
// on these declarations while resolving every entry point dynamically.
#include <avisynth_c.h>
#include <dualsynth/avisynth/c/video_bridge.hpp>

namespace neo_mv::ds2::avs {
namespace ac = ds::avisynth::c;

// Native AviSynth arrays keep the same parameter positions as VapourSynth.
// The common descriptors remain unchanged for existing VS callers.
template <class Base>
struct Adapter : Base {
  static constexpr bool forward_audio = true;
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

// All AVS_Value objects here borrow from the live callback arguments. Separate
// scalar storage keeps singleton-array pointers valid until bridge creation ends.
struct Call {
  std::vector<AVS_Value> scalars, values;
  Call(AVS_Value args, const ds::FilterDescriptor& d) : scalars(d.params.size(), avs_void), values(scalars) {
    for (std::size_t i = 0; i < values.size(); ++i) {
      scalars[i] = avs_is_array(args) ? (i < std::size_t(avs_array_size(args)) ? args.d.array[i] : avs_void)
                                      : (i == 0 ? args : avs_void);
      values[i] = scalars[i];
      if (d.params[i].is_array && avs_defined(values[i]) && !avs_is_array(values[i]))
        values[i] = avs_new_value_array(&scalars[i], 1);
    }
  }
  AVS_Value arguments() { return avs_new_value_array(values.data(), static_cast<int>(values.size())); }
};

template <class Function>
AVS_Value guarded(AVS_ScriptEnvironment* env, Function&& function) {
  try {
    auto& api = ac::CApi::instance();
    require(api.check_version && !api.check_version(env, 11), "neo-mv requires AviSynth interface 11 or later");
    return function();
  } catch (const std::exception& e) {
    return ac::avs_new_value_error(ac::save_error(env, e.what()));
  } catch (...) {
    return ac::avs_new_value_error("neo-mv: AviSynth creation failed");
  }
}

template <class Base>
AVS_Value AVSC_CC create(AVS_ScriptEnvironment* env, AVS_Value args, void* user_data) {
  return guarded(env, [&] {
    const auto d = Adapter<Base>::descriptor();
    Call call(args, d);
    if constexpr (std::is_same_v<Base, RenderBridge<true>>) {
      const auto radius = reinterpret_cast<std::intptr_t>(user_data);
      const auto vectors = call.values[index(d, "vectors")];
      require(radius == 0 || (avs_is_array(vectors) && avs_array_size(vectors) == 2 * radius),
              "named Degrain requires exactly 2R vector members");
    }
    return ac::create_video_filter_bridge<Adapter<Base>>(env, call.arguments(), nullptr);
  });
}

ds::FilterDescriptor many_descriptor() {
  auto d = Adapter<Bridge<Operation::Analyse>>::descriptor();
  d.params.insert(d.params.end() - 1, {"radius", ds::ParamType::Integer});
  return d;
}

AVS_Value AVSC_CC many(AVS_ScriptEnvironment* env, AVS_Value args, void*) {
  return guarded(env, [&] {
    const auto d = many_descriptor();
    Call call(args, d);
    const auto parsed = unwrap(ac::read_params(call.arguments(), d));
    const Params params{parsed};
    const int radius = params.integer("radius", 1), step = params.integer("delta", 1);
    require(radius > 0 && step > 0 && radius <= SHRT_MAX / 2 && std::int64_t(radius) * step <= INT32_MAX,
            "invalid AnalyseMany radius or delta product");
    call.values.erase(call.values.begin() + index(d, "radius"));
    std::vector<std::vector<AVS_Value>> members;
    std::vector<AVS_Value> calls;
    members.reserve(std::size_t(radius) * 2);
    calls.reserve(std::size_t(radius) * 2);
    for (int r = 1; r <= radius; ++r)
      for (int sign : {1, -1}) {
        members.push_back(call.values);
        members.back()[index(d, "delta")] = avs_new_value_int(r * step * sign);
        calls.push_back(avs_new_value_array(members.back().data(), static_cast<int>(members.back().size())));
      }
    return ac::create_video_filter_bundle<Adapter<Bridge<Operation::Analyse>>>({calls.data(), calls.size()}, env);
  });
}

ds::FilterDescriptor recalculate_descriptor() {
  auto d = Adapter<Bridge<Operation::Recalculate>>::descriptor();
  d.params[index(d, "vectors")].is_array = true;
  d.params[index(d, "vectors")].avs_array_binding = ds::AvisynthArrayBinding::Native;
  return d;
}

AVS_Value AVSC_CC recalculate(AVS_ScriptEnvironment* env, AVS_Value args, void*) {
  return guarded(env, [&] {
    const auto d = recalculate_descriptor();
    Call call(args, d);
    const auto slot = index(d, "vectors");
    const auto vectors = call.values[slot];
    require(avs_is_array(vectors) && avs_array_size(vectors) > 0, "Recalculate requires nonempty vectors");
    std::vector<std::vector<AVS_Value>> members;
    std::vector<AVS_Value> calls;
    members.reserve(avs_array_size(vectors));
    calls.reserve(avs_array_size(vectors));
    for (int i = 0; i < avs_array_size(vectors); ++i) {
      members.push_back(call.values);
      members.back()[slot] = vectors.d.array[i];
      calls.push_back(avs_new_value_array(members.back().data(), static_cast<int>(members.back().size())));
    }
    return ac::create_video_filter_bundle<Adapter<Bridge<Operation::Recalculate>>>({calls.data(), calls.size()}, env);
  });
}

using Invoke = decltype(&avs_invoke);
Invoke host_invoke() {
  static const auto value = [] {
#if defined(_WIN32)
    return reinterpret_cast<Invoke>(
        reinterpret_cast<void*>(GetProcAddress(GetModuleHandleA("avisynth.dll"), "avs_invoke")));
#else
    return reinterpret_cast<Invoke>(dlsym(RTLD_DEFAULT, "avs_invoke"));
#endif
  }();
  return value;
}

template <class Base>
AVS_Value AVSC_CC create_depan(AVS_ScriptEnvironment* env, AVS_Value args, void*) {
  return guarded(env, [&] {
    const auto d = Adapter<Base>::descriptor();
    Call call(args, d);
    const auto info = call.values[index(d, "info")];
    const bool show = avs_defined(info) && avs_is_bool(info) && avs_as_bool(info);
    auto base = ac::create_video_filter_bridge<Adapter<Base>>(env, call.arguments(), nullptr);
    if (avs_is_error(base) || !show)
      return base;
    struct Owner {
      AVS_Value value;
      ~Owner() { ac::CApi::instance().release_value(value); }
    } owner{base};
    const auto invoke = host_invoke();
    require(invoke != nullptr, "Depan info requires avs_invoke and propShow");
    AVS_Value values[] = {base, avs_new_value_string(Base::diagnostic_property)};
    const char* names[] = {nullptr, "props"};
    return invoke(env, "propShow", avs_new_value_array(values, 2), names);
  });
}

AVS_Value AVSC_CC kernel_info(AVS_ScriptEnvironment* env, AVS_Value, void*) {
  return guarded(env, [&] {
    auto& api = ac::CApi::instance();
    require(api.copy_value && api.release_value && api.save_string, "KernelInfo requires value ownership APIs");
    const auto fft = estimate_fft_profile();
    AVS_Value fields[] = {avs_new_value_string(api.save_string(env, selected_backend_name(), -1)),
                          avs_new_value_string(api.save_string(env, selected_target_name(), -1)),
                          avs_new_value_string(api.save_string(env, depan::estimate::fft_profile_name(fft), -1)),
                          avs_new_value_int(depan::estimate::fft_lanes(fft))};
    AVS_Value result = avs_void;
    api.copy_value(&result, avs_new_value_array(fields, 4));
    return result;
  });
}

void add(AVS_ScriptEnvironment* env, const std::string& name, const ds::FilterDescriptor& d, AVS_ApplyFunc callback,
         void* data = nullptr) {
  auto& api = ac::CApi::instance();
  const auto signature = unwrap(ds::make_avisynth_signature(d));
  require(api.add_function(env, api.save_string(env, name.c_str(), -1), api.save_string(env, signature.c_str(), -1),
                           callback, data) == 0,
          "cannot register neo-mv AviSynth function");
}
template <class Base>
void add(AVS_ScriptEnvironment* env) {
  add(env, std::string("neo_mv_") + Base::Core::name, Adapter<Base>::descriptor(), create<Base>);
}
template <class Base>
void add_depan(AVS_ScriptEnvironment* env) {
  add(env, std::string("neo_mv_") + Base::Core::name, Adapter<Base>::descriptor(), create_depan<Base>);
}
} // namespace neo_mv::ds2::avs

#if defined(_WIN32)
#define NEO_MV_AVS_EXPORT extern "C" __declspec(dllexport)
#else
#define NEO_MV_AVS_EXPORT extern "C" __attribute__((visibility("default")))
#endif

NEO_MV_AVS_EXPORT const char* AVSC_CC avisynth_c_plugin_init2(AVS_ScriptEnvironment* env) {
  using namespace neo_mv::ds2;
  using namespace neo_mv::ds2::avs;
  try {
    auto& api = ac::CApi::instance();
    require(api.add_function && api.save_string, "neo-mv: missing AviSynth registration API");
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
  } catch (const std::exception& e) {
    return ac::save_error(env, e.what());
  } catch (...) {
    return "neo-mv: AviSynth registration failed";
  }
}
