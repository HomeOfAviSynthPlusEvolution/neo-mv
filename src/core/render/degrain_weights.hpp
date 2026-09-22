#pragma once

#include "core/render/reference.hpp"

namespace neo_mv {

struct DegrainWeights {
  int centre;
  std::vector<int> reference;
};
struct ReferenceReliability {
  bool available;
  std::int64_t sad;
};

inline int degrain_reliability(std::int64_t sad, int threshold) {
  if (sad < 0 || threshold < 0 || threshold >= INT32_MAX)
    throw std::invalid_argument("invalid Degrain SAD or threshold");
  if (sad >= threshold)
    return 0;
  const double r = double(sad) / double(threshold);
  const double z = r * r;
  const double numerator = 256.0 * (1.0 - z);
  const double denominator = 1.0 + z;
  return static_cast<int>(numerator / denominator);
}

// Validates deltas in supplied pair order, without sorting or negating int32.
inline void validate_degrain_pairs(const std::vector<AnalysisMetadata>& members) {
  if (members.size() < 2 || members.size() > 50 || members.size() % 2 != 0)
    throw std::invalid_argument("Degrain requires 1 to 25 vector pairs");
  std::int64_t previous = 0;
  for (std::size_t i = 0; i < members.size(); i += 2) {
    const auto& a = members[i];
    const auto& b = members[i + 1];
    if (!valid_analysis_metadata(a) || !valid_analysis_metadata(b) || !same_render_analysis(members[0], a, false) ||
        !same_render_analysis(members[0], b, false))
      throw std::invalid_argument("Degrain member descriptors differ");
    const auto d = std::int64_t(a.delta);
    const auto distance = d < 0 ? -d : d;
    if (distance <= previous || std::int64_t(b.delta) != -d)
      throw std::invalid_argument("Degrain deltas must be opposite with increasing pair distance");
    previous = distance;
  }
}

class DegrainWeightPlan {
  int pairs_;
  std::array<std::vector<int>, 2> thresholds_;
  std::vector<std::int64_t> user_; // vector-input order; centre stored separately
  std::int64_t centre_;

public:
  DegrainWeightPlan(const AnalysisMetadata& m, int pairs, std::array<std::int64_t, 2> near,
                    std::array<std::int64_t, 2> far, const std::vector<std::int64_t>& user)
      : pairs_(pairs) {
    if (pairs < 1 || pairs > 25 || user.size() != std::size_t(2 * pairs + 1))
      throw std::invalid_argument("invalid Degrain pair or user coefficient count");
    const auto maximum = 2147483646LL / (256 * (2 * pairs + 1));
    std::vector<std::int64_t> effective;
    for (auto u : user) {
      const auto clamped = std::clamp(u, std::int64_t(INT32_MIN), std::int64_t(INT32_MAX));
      if (clamped < 0 || clamped > maximum)
        throw std::invalid_argument("Degrain user coefficient exceeds allowed range");
      effective.push_back(clamped);
    }
    centre_ = effective[pairs];
    for (int j = 1; j <= pairs; ++j) {
      user_.push_back(effective[pairs - j]);
      user_.push_back(effective[pairs + j]);
    }
    const RenderThresholdScale scale(m);
    constexpr double pi = 0x1.921fb54442d18p+1;
    for (int plane = 0; plane < 2; ++plane) {
      const auto n = scale(near[plane]), f = scale(far[plane]);
      if (n >= INT32_MAX || f >= INT32_MAX)
        throw std::invalid_argument("scaled Degrain threshold must be below INT32_MAX");
      auto& values = thresholds_[plane];
      values.push_back(static_cast<int>(n));
      for (int j = 2; j <= pairs; ++j) {
        const double numerator = double(j - 1) * pi;
        const double theta = numerator / double(pairs - 1);
        const double a = (1.0 - std::cos(theta)) / 2.0;
        const double adjustment = a * double(f - n);
        const double interpolated = double(n) + adjustment;
        values.push_back(static_cast<int>(std::floor(interpolated + 0.5)));
      }
    }
  }
  int threshold(int pair, int plane) const {
    if (pair < 0 || pair >= pairs_ || plane < 0 || plane > 2)
      throw std::invalid_argument("Degrain threshold index out of range");
    return thresholds_[plane == 0 ? 0 : 1][pair];
  }
  // Internal block path: caller supplies 2*pairs+1 slots, centre first.
  // The public vector-returning API below retains its original admission.
  void compute(const ReferenceReliability* references, std::size_t count, int plane, int* output) const {
    if (!references || !output || count != std::size_t(2 * pairs_) || plane < 0 || plane > 2)
      throw std::invalid_argument("invalid Degrain reliability output");
    std::array<std::int64_t, 50> products{};
    std::int64_t total = 256 * centre_ + 1;
    const auto& thresholds = thresholds_[plane == 0 ? 0 : 1];
    for (std::size_t i = 0; i < count; ++i) {
      const auto& r = references[i];
      products[i] = (r.available ? degrain_reliability(r.sad, thresholds[i / 2]) : 0) * user_[i];
      total += products[i];
    }
    const double g = 256.0 / double(total);
    output[0] = 256;
    for (std::size_t i = 0; i < count; ++i) {
      const int value = static_cast<int>(double(products[i]) * g);
      output[i + 1] = value;
      output[0] -= value;
    }
    if (output[0] < 0)
      throw std::overflow_error("negative normalized Degrain centre weight");
  }
  DegrainWeights operator()(const std::vector<ReferenceReliability>& references, int plane) const {
    if (references.size() != std::size_t(2 * pairs_))
      throw std::invalid_argument("Degrain reliability count mismatch");
    DegrainWeights result{0, std::vector<int>(references.size())};
    std::array<int, 51> values{};
    compute(references.data(), references.size(), plane, values.data());
    result.centre = values[0];
    std::copy(values.begin() + 1, values.begin() + 1 + references.size(), result.reference.begin());
    return result;
  }
};

} // namespace neo_mv
