#pragma once

#include "concepts.h"

#include <cstdint>

namespace vic
{

using Numerator = uint64_t;
using Denominator = uint64_t;

template <typename T>
concept ConceptRational = requires(T& rational) {
    typename T::DataType; //
    {
        decltype(T::Num)(T::Num)
    } -> std::integral;
    {
        decltype(T::Denom)(T::Denom)
    } -> std::integral;
    {
        decltype(rational.val)(rational.val)
    } -> Numeric;
};

// forward declare rational
template <typename T, Numerator num, Denominator denom>
struct Rational;

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr auto Add(const T1 first, const T2 second)
{
    if constexpr(ConceptRational<T1> && ConceptRational<T2>)
    {
        if constexpr(std::is_same_v<T1, T2>)
            return T1(first.val + second.val);
        else
            return Rational<int, 1, 1>{};
    }
    else if constexpr(ConceptRational<T1> && Numeric<T2>)
    {
        return Rational<int, 1, 1>{};
    }
    else if(Numeric<T1> && ConceptRational<T2>)
    {
        return Rational<int, 1, 1>{};
    }
    else
    {
        return 0; // static assert?
    }
}

template <typename T, Numerator num, Denominator denom>
struct Rational
{
    using DataType = T;

    static constexpr Numerator Num = num;
    static constexpr Denominator Denom = denom;

    Rational() = default;

    template <typename TInput>
        requires std::integral<TInput>
    Rational(const TInput _val)
        : val(_val)
    { }

    Rational& operator=(Rational other)
    {
        val = other.val;
        return *this;
    }

    template <typename TInput>
        requires vic::Numeric<TInput>
    Rational& operator=(const TInput other)
    {
        val = other;
        return *this;
    }

    template <typename TOther>
        requires vic::Numeric<TOther> || ConceptRational<TOther>
    auto operator+(const TOther other)
    {
        return ::vic::Add(*this, other);
    }

    explicit operator T() const { return val; }
    explicit operator bool() const { return val; }

    T val{};
};

template <typename T>
    requires ConceptRational<T>
struct rational_simplify
{
    using data_type = typename T::DataType;
    static constexpr auto gcd = std::gcd(T::Num, T::Denom);
    using type = Rational<data_type, T::Num / gcd, T::Denom / gcd>;
};

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
struct rational_addition
{
    using data_type = decltype(std::declval<typename T1::DataType>() + std::declval<typename T2::DataType>());
    using type = Rational<data_type, 1, 1>;
};

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
struct rational_multiplication
{
    using data_type = decltype(std::declval<typename T1::DataType>() * std::declval<typename T2::DataType>());
    using type = Rational<data_type, 1, 1>;
};

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_addition_t = rational_addition<T1, T2>::type;

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_multiplication_t = rational_multiplication<T1, T2>::type;

template <typename T>
    requires ConceptRational<T>
using rational_simplify_t = rational_simplify<T>::type;

//
//
//

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
constexpr auto Multiply(const T1 first, const T2 second)
{
    using TReturn = rational_multiplication_t<T1, T2>;
    return TReturn{};
}

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
constexpr bool Equality(const T1 first, const T2 second)
{
    // using TReturn = rational_multiplication_t<T1, T2>;
    return false;
}

//
//
//

template <typename TResult, typename TInput>
    requires ConceptRational<TResult> && ConceptRational<TInput>
TResult To(const TInput input)
{
    return TResult{};
}

//template <typename TRational>
//    requires ConceptRational<TRational>
//TRational Add(const TRational first, const TRational second)
//{
//    return TRational{first.val + second.val};
//}

} // namespace vic