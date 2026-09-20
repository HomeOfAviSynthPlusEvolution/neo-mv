#pragma once

namespace neo_mv {
enum class KernelBackend { scalar, highway };

// NEO_MV_KERNEL is read once per loaded plugin, before creating any filter.
// The resulting choice is immutable for the lifetime of its frame graphs.
KernelBackend selected_backend();
const char *selected_backend_name();
const char *selected_target_name();
} // namespace neo_mv
