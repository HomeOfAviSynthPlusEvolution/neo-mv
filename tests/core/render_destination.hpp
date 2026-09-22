#pragma once
#include "core/render/frame.hpp"
#include <cstring>

namespace neo_mv::test {
// Compare independent and host-style output, including padded rows and distinct
// plane contents. Repeated use catches stale scratch or accidental ownership.
template <class T, class Render>
void verify_destination(Render render) {
  const auto expected = render(static_cast<const RenderDestination<T>*>(nullptr));
  RenderDestination<T> destination{};
  std::array<std::vector<T>, 3> storage;
  for (std::size_t k = 0; k < expected.size(); ++k) {
    const auto v = expected[k].view();
    const int stride = v.width() + 7;
    storage[k].assign(std::size_t(stride) * v.height() + 2, T(93));
    destination[k] = checked_plane(storage[k].data() + 1, v.width(), v.height(), std::ptrdiff_t(stride) * sizeof(T),
                                   (storage[k].size() - 1) * sizeof(T));
  }
  for (int repeat = 0; repeat < 2; ++repeat) {
    const T guard = T(93 + repeat);
    for (auto& buffer : storage)
      std::fill(buffer.begin(), buffer.end(), guard);
    const auto result = render(&destination);
    if (result.size() != expected.size())
      throw std::runtime_error("destination plane count");
    for (std::size_t k = 0; k < expected.size(); ++k) {
      const auto a = expected[k].view(), b = result[k].view();
      if (b.row(0).data() != destination[k].row(0).data())
        throw std::runtime_error("destination was not used directly");
      for (int y = 0; y < a.height(); ++y) {
        if (std::memcmp(a.row(y).data(), b.row(y).data(), std::size_t(a.width()) * sizeof(T)))
          throw std::runtime_error("destination pixel mismatch");
        for (int gap = 0; gap < 7; ++gap)
          if (storage[k][1 + std::size_t(y) * (a.width() + 7) + a.width() + gap] != guard)
            throw std::runtime_error("destination row guard changed");
      }
      if (storage[k].front() != guard || storage[k].back() != guard)
        throw std::runtime_error("destination guard changed");
    }
  }
}
} // namespace neo_mv::test
