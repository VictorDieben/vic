#pragma once

#include <cstddef>

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
    if constexpr(n == 0)
        return T(1); // todo: move outside of recursive function
    else if constexpr(n == 1)
        return x;
    else if constexpr(n % 2 == 0)
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

template <typename T, std::size_t n>
constexpr T exp_recursive(const T x, const T p, const T f) noexcept
{
    // todo: make f a value template 
    if constexpr(n > 500)
        return 0.;
    else
    {
        if(p > 1e300 || p < 1e-300 || f > 1e300)
            return 0.;
        const T iter = p / f; 
        const T recursive = exp_recursive<T, n + 1>(x, p * x, f * n);
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
    return detail::exp_recursive<T, 1>(x, 1., 1.);
}

} // namespace math
} // namespace vic