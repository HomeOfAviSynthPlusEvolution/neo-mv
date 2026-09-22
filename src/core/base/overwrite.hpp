#pragma once
#include <memory>
#include <type_traits>
#include <vector>
#include <utility>

namespace neo_mv {
// Only for numeric storage whose complete live range is written before reading.
// Explicit value construction (including copies) retains normal vector semantics.
template <class T>
struct OverwriteAllocator : std::allocator<T> {
  template <class U>
  struct rebind {
    using other = OverwriteAllocator<U>;
  };
  OverwriteAllocator() = default;
  template <class U>
  OverwriteAllocator(const OverwriteAllocator<U>&) {}
  template <class U>
  void construct(U* p) {
    static_assert(std::is_arithmetic_v<U>);
    ::new (static_cast<void*>(p)) U;
  }
  template <class U, class... Args>
  void construct(U* p, Args&&... args) {
    ::new (static_cast<void*>(p)) U(std::forward<Args>(args)...);
  }
};
template <class T>
using OverwriteVector = std::vector<T, OverwriteAllocator<T>>;
struct OverwriteTag {};
inline constexpr OverwriteTag overwrite{};
} // namespace neo_mv
