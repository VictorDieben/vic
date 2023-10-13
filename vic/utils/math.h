#pragma once

#include <array>
#include <cstddef>
#include <ranges>

namespace vic
{
namespace math
{

constexpr std::size_t Factorial(const std::size_t n)
{
    if(n == 1)
        return 1;
    else
        return n * Factorial(n - 1);
}

template <typename T>
struct factorial_info;

template <>
struct factorial_info<float>
{
    constexpr static std::size_t limit = 170;
};

template <>
struct factorial_info<double>
{
    constexpr static std::size_t limit = 170;
};

// contains constexpr versions of cmath functions
namespace detail
{

template <typename T>
constexpr T pow_recursive(const T x, std::size_t n) noexcept
{
    if(n == 0)
        return T(1); // todo: move outside of recursive function
    else if(n == 1)
        return x;
    else if(n % 2 == 0)
    {
        const T tmp = exp_recursive<T, n / 2>(x);
        return tmp * tmp;
    }
    else
    {
        const T tmp = exp_recursive<T, n - 1>(x);
        return x * tmp;
    }
}

template <typename T, std::size_t n, double f>
inline constexpr T exp_recursive(const T x, const T p) noexcept
{
    // p = x^n
    // e^x = 1 + x/1! + x^2/2! + x^3/3! + ...

    // todo: decide end condition based on some rationale
    // todo: split in a x > 1 and x < 1 function? that way only one check is needed
    if constexpr(n > 1000 || f > 1.e307)
        return 0.;
    else
    {
        if(p > 1.e307 || p < 1.e-307)
            return 0.;
        static constexpr double oneOverF = 1. / f;
        const T iter = p * oneOverF;
        const T recursive = exp_recursive<T, n + 1, f * n>(x, p * x);
        return iter + recursive;
    }
}

} // namespace detail

template <typename T>
constexpr T Pow(const T x, std::size_t n) noexcept
{
    return detail::pow_recursive<T>(x, n);
}

template <typename T>
constexpr T exp(const T x) noexcept
{
    // todo: good way of defining max recursions
    // we need to calculate the maximum power of x before it gets to close to infinity
    // return detail::exp_recursive<T, 0, 100>(x, 1.);
    return detail::exp_recursive<T, 1, 1.>(x, 1.);
}
template <typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

//
// The following two functions are related to assigning set indices to each object in a list
// imagine the list of objects: {a, b, c, d}
// the set assignment {{a}, {b, c}, {d}} would translate to the indices: {0, 1, 1, 2}
// the set assignment {{a, c}, {b, d}} would translate to the indices: {0, 1, 0, 1}
// etc.
//
// The first value will _always_ be 0. The next value will either be 0 or 1. The one after that can be 0, 1, or 2.
// Effectively, we need to create a number where each index has another base.

constexpr std::array<uint8_t, 32> NumberOfPossibilities()
{
    std::array<uint8_t, 32> values{};
    for(std::size_t i = 0; i < values.size(); ++i)
        values.at(i) = i + 1;
    return values;
}

constexpr std::array<uint64_t, 32> CumulativeSize()
{
    constexpr auto possibilities = NumberOfPossibilities();
    std::array<uint64_t, 32> values;
    values.at(0) = 1;
    for(std::size_t i = 1; i < values.size(); ++i)
        values.at(i) = values.at(i - 1) * possibilities.at(i);
    return values;
}

template <typename TReturn, std::ranges::range TRange>
const TReturn SetAssingmentsToInteger(const TRange& range)
{
    static constexpr auto possibilities = NumberOfPossibilities();
    static constexpr auto cumulativeSize = CumulativeSize();
    // todo
    return TReturn{};
}

template <std::ranges::range TRange, std::integral TInteger>
void IntegerToSetAssingment(const TInteger integer, TRange& range)
{
    // todo
}

} // namespace math
} // namespace vic