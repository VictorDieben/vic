#pragma once

namespace vic
{
namespace templates
{

template <typename T1, typename T2>
struct multiplication
{
    using type = decltype(std::declval<T1&>() * std::declval<T2&>());
};

template <typename T1, typename T2>
using multiplication_t = multiplication<T1, T2>::type;

//

template <typename T1, typename T2>
struct division
{
    using type = decltype(std::declval<T1&>() / std::declval<T2&>());
};

template <typename T1, typename T2>
using division_t = division<T1, T2>::type;

//

template <typename T1, typename T2>
struct addition
{
    using type = decltype(std::declval<T1&>() + std::declval<T2&>());
};

template <typename T1, typename T2>
using addition_t = addition<T1, T2>::type;

//

template <typename T1, typename T2>
struct subtraction
{
    using type = decltype(std::declval<T1&>() - std::declval<T2&>());
};

template <typename T1, typename T2>
using subtraction_t = subtraction<T1, T2>::type;

//

template <typename T1, typename T2>
struct remainder
{
    using type = decltype(std::declval<T1&>() % std::declval<T2&>());
};

template <typename T1, typename T2>
using remainder_t = remainder<T1, T2>::type;

} // namespace templates
} // namespace vic