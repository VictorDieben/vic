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
constexpr auto Add(const T1 first, const T2 second);

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
constexpr bool Equality(const T1 first, const T2 second)
{
    // x * (a/b) == y * (c/d)
    // x * (ad/bd) == y * (cb/bd)
    // x * ad == y * cb

    // todo: this might lead to overflow problems, but there is no good way to solve it otherwise.
    return first.val * (T1::Num * T2::Denom) == second.val * (T2::Num * T1::Denom);
}

template <typename T>
    requires ConceptRational<T>
struct rational_simplify
{
    using data_type = typename T::DataType;
    static constexpr auto gcd = std::gcd(T::Num, T::Denom);
    using type = Rational<data_type, T::Num / gcd, T::Denom / gcd>;
};

template <typename T>
    requires ConceptRational<T>
using rational_simplify_t = rational_simplify<T>::type;

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
struct rational_addition
{
    using data_type = decltype(std::declval<typename T1::DataType>() + std::declval<typename T2::DataType>());

    // x * (a/b) + y * (c/d)  == x * (a*d)/(b*d) + y * (c*b)/(b*d)

    //static constexpr Numerator num1 = T1::Num * T2::Denom;
    //static constexpr Numerator num2 = T2::Num * T1::Denom;

    // make sure we are working with simplified representations
    using S1 = rational_simplify_t<T1>;
    using S2 = rational_simplify_t<T2>;

    static constexpr Denominator denom = S1::Denom * S2::Denom;

    using type = rational_simplify_t<Rational<data_type, 1, denom>>;
};

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
struct rational_multiplication
{
    // x*(a/b) * y*(c/d)  == x * y * (ac/bd)

    using data_type = decltype(std::declval<typename T1::DataType>() * std::declval<typename T2::DataType>());

    using S1 = rational_simplify_t<T1>;
    using S2 = rational_simplify_t<T2>;

    using type = rational_simplify_t<Rational<data_type, S1::Num * S2::Num, S1::Denom * S2::Denom>>;
};

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_addition_t = rational_addition<T1, T2>::type;

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_multiplication_t = rational_multiplication<T1, T2>::type;

//
//
//

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr auto Multiply(const T1 first, const T2 second)
{
    if constexpr(ConceptRational<T1> && ConceptRational<T2>)
    {
        return rational_multiplication_t<T1, T2>{first.val * second.val};
    }
    else if constexpr(ConceptRational<T1> && Numeric<T2>)
    {
        return Multiply(first, Rational<T2, 1, 1>{second});
    }
    else if constexpr(Numeric<T1> && ConceptRational<T2>)
    {
        return Multiply(second, Rational<T1, 1, 1>{first});
    }
    else
    {
        return 0; // static assert? Should not be reachable
    }
}

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
        {
            // x * (a/b) + y * (c/d)
            using AdditionType = rational_addition_t<T1, T2>;
            const AdditionType t1 = To<AdditionType>(first);
            const AdditionType t2 = To<AdditionType>(second);
            return AdditionType{t1.val + t2.val};
        }
    }
    else if constexpr(ConceptRational<T1> && Numeric<T2>)
    {
        return Add(first, Rational<T2, 1, 1>{second});
    }
    else if constexpr(Numeric<T1> && ConceptRational<T2>)
    {
        return Add(second, Rational<T1, 1, 1>{first});
    }
    else
    {
        return 0; // static assert? Should not be reachable
    }
}

//
//
//

template <typename TResult, typename TInput>
    requires ConceptRational<TResult> && ConceptRational<TInput>
TResult To(const TInput input)
{
    // x*(a/b) => y*(c/d)
    // x* (ad/bc) => y

    // todo: this will create roundoff errors, think about how to properly do this
    return TResult{(input.val * TInput::Num * TResult::Denom) / (TInput::Denom * TResult::Num)};
}

//template <typename TRational>
//    requires ConceptRational<TRational>
//TRational Add(const TRational first, const TRational second)
//{
//    return TRational{first.val + second.val};
//}

template <typename T, Numerator num, Denominator denom>
//    requires(num != 0 && denom != 0) // TODO
struct Rational
{
    using ThisType = Rational<T, num, denom>;
    using DataType = T;

    static constexpr Numerator Num = num;
    static constexpr Denominator Denom = denom;

    constexpr Rational() = default;

    template <typename TInput>
        requires std::integral<TInput>
    constexpr Rational(const TInput _val)
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

    template <typename TOther>
        requires ConceptRational<TOther>
    bool operator==(const TOther other) const
    {
        return Equality(*this, other);
    }

    T val{};
};

} // namespace vic