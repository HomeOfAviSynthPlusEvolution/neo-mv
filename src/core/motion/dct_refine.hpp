#pragma once
#include <cstdint>
namespace neo_mv::dct_detail {
// Validated unsigned16 integer pixels and an AC frequency. The estimate only
// seeds the candidate; integer enclosures determine the returned rounding.
int refine_ac(const std::uint16_t* samples, int width, int height, int u, int v, double estimate);
// Internal entry points also exercised directly by the numerical tests.
bool fixed_round(const std::uint16_t* samples, int width, int height, int u, int v, int candidate);
int interval_round(const std::uint16_t* samples, int width, int height, int u, int v);
void transform_block(int width, int height, const double* input, double* rows, double* output, bool simd);
} // namespace neo_mv::dct_detail
