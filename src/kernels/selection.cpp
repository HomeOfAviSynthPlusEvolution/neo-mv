#include "kernels/selection.hpp"

#include <cstdlib>
#include <cstring>
#include <stdexcept>
#if NEO_MV_ENABLE_HIGHWAY
#include "highway/rows.hpp"
#endif

namespace neo_mv {
KernelBackend selected_backend() {
  static const KernelBackend backend = [] {
    const char *value = std::getenv("NEO_MV_KERNEL");
    if (value && std::strcmp(value, "scalar") == 0)
      return KernelBackend::scalar;
    if (value && std::strcmp(value, "highway") != 0)
      throw std::invalid_argument("NEO_MV_KERNEL must be scalar or highway");
#if NEO_MV_ENABLE_HIGHWAY
    return KernelBackend::highway;
#else
    if (value)
      throw std::invalid_argument("Highway kernels are not built into this plugin");
    return KernelBackend::scalar;
#endif
  }();
  return backend;
}
const char *selected_backend_name() {
  return selected_backend() == KernelBackend::scalar ? "scalar" : "highway";
}
const char *selected_target_name() {
#if NEO_MV_ENABLE_HIGHWAY
  if (selected_backend() == KernelBackend::highway)
    return simd::detail::target_name();
#endif
  return "scalar";
}
} // namespace neo_mv
