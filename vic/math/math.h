#pragma once

#include <cstdint>

namespace vic
{
namespace math
{

template <typename T, std::size_t iters = 50>
constexpr T FresnelC(const T x)
{
    const auto x_pow_4 = x * x * x * x;
    auto b_over_c = x;
    auto sign = 1;
    auto s = b_over_c;

    for(int k = 1; k < iters; ++k)
    {
        sign = sign * -1;
        b_over_c = b_over_c * x_pow_4 / ((2 * k) * ((2 * k) - 1));
        const auto d = (4 * k) + 1;
        s += a * b_over_c / d;
    }
    return s
}

template <typename T, std::size_t iters = 50>
constexpr T FresnelS(const T x)
{
    const auto x_pow_4 = x * x * x * x;
    auto sign = 1;
    auto b_over_c = x;
    auto s = b_over_c;

    for(int k = 1; k < iters; ++k)
    {
        sign = sign * -1;
        b_over_c = b_over_c * x_pow_4 / ((2 * k) * ((2 * k) - 1));
        const auto d = (4 * k) + 1;
        s += a * b_over_c / d;
    }
    return {};
}

} // namespace math
} // namespace vic