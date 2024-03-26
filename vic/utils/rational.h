#pragma once

#include "concepts.h"

#include <compare>
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
struct rational_division
{
    // x*(a/b) / y*(c/d)  == (x ad) / (y bc) == (x/y) * (ad/bc)

    using data_type = decltype(std::declval<typename T1::DataType>() / std::declval<typename T2::DataType>());

    using S1 = rational_simplify_t<T1>;
    using S2 = rational_simplify_t<T2>;

    using type = rational_simplify_t<Rational<data_type, S1::Num * S2::Denom, S1::Denom * S2::Num>>;
};

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_addition_t = rational_addition<T1, T2>::type;

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_subtraction = rational_addition<T1, T2>; // todo: different type? i don't think it is needed

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_subtraction_t = rational_subtraction<T1, T2>::type;

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_multiplication_t = rational_multiplication<T1, T2>::type;

template <typename T1, typename T2>
    requires ConceptRational<T1> && ConceptRational<T2>
using rational_division_t = rational_division<T1, T2>::type;

//
//
//

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr auto Multiply(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
        return Multiply(Rational<T1, 1, 1>{first}, second);
    else if constexpr(Numeric<T2>)
        return Multiply(first, Rational<T2, 1, 1>{second});
    else
    {
        return rational_multiplication_t<T1, T2>{first.val * second.val};
    }
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr auto Add(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
        return Add(Rational<T1, 1, 1>{first}, second);
    else if constexpr(Numeric<T2>)
        return Add(first, Rational<T2, 1, 1>{second});
    else
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
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr auto Subtract(const T1 first, const T2 second) // first - second
{
    if constexpr(Numeric<T1>)
        return Subtract(Rational<T1, 1, 1>{first}, second);
    else if constexpr(Numeric<T2>)
        return Subtract(first, Rational<T2, 1, 1>{second});
    else
    {
        if constexpr(std::is_same_v<T1, T2>)
            return T1(first.val - second.val);
        else
        {
            // x * (a/b) - y * (c/d)
            using SubtractType = rational_subtraction_t<T1, T2>;
            const SubtractType t1 = To<SubtractType>(first);
            const SubtractType t2 = To<SubtractType>(second);
            return SubtractType{t1.val - t2.val};
        }
    }
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr auto Devide(const T1 first, const T2 second) // first / second
{
    if constexpr(Numeric<T1>)
        return Devide(Rational<T1, 1, 1>{first}, second);
    else if constexpr(Numeric<T2>)
        return Devide(first, Rational<T2, 1, 1>{second});
    else
    {
        // x*(a/b) / y*(c/d)  == (x ad) / (y bc) == (x/y) * (ad/bc)

        using DivisionType = rational_division_t<T1, T2>;
        const DivisionType t1 = To<DivisionType>(first);
        const DivisionType t2 = To<DivisionType>(second);

        //const auto ad = T1::Num * T2::Denom;
        //const auto bc = T1::Denom * T2::Num;
        // const auto newval = (first.val * ad) / (second.val * bc);
        //return DivisionType{(newval * bc) / ad};

        const auto newval = t1.val / t2.val;
        return DivisionType{newval};
    }
}

//
// Comparisson operators
//

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr std::strong_ordering Spaceship(const T1 first, const T2 second) // first <=> second
{
    if constexpr(Numeric<T1>)
        return Spaceship(Rational<T1, 1, 1>{first}, second);
    else if constexpr(Numeric<T2>)
        return Spaceship(first, Rational<T2, 1, 1>{second});
    else
    {
        // x * (a/b) <=> y * (c/d)
        // x * ad <=> y * bc

        static constexpr Numerator ad = (T1::Num * T2::Denom);
        static constexpr Numerator bc = (T1::Denom * T2::Num);
        return first.val * ad <=> second.val * bc;
    }
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator<=>(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs);
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator==(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs) == std::strong_ordering::equal;
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator<(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs) == std::strong_ordering::less;
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator<=(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs) != std::strong_ordering::greater;
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator>(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs) == std::strong_ordering::greater;
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator>=(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs) != std::strong_ordering::less;
}

template <typename T1, typename T2>
    requires(ConceptRational<T1> && ConceptRational<T2>) || //
            (ConceptRational<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptRational<T2>)
constexpr bool operator!=(const T1 lhs, const T2 rhs)
{
    return Spaceship(lhs, rhs) != std::strong_ordering::equal;
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

    // the fraction ad/bc can be computed at compile time,
    // but we need to do some extra work to avoid overflow as much as possible
    static constexpr auto ad = TInput::Num * TResult::Denom;
    static constexpr auto bc = TInput::Denom * TResult::Num;
    static constexpr auto gcd = std::gcd(ad, bc);
    static constexpr auto _ad = ad / gcd;
    static constexpr auto _bc = bc / gcd;

    // todo: this will create roundoff errors, think about how to properly do this
    return TResult{(input.val * _ad) / _bc};
}

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
        : val(static_cast<T>(_val))
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
    template <typename TOther>
        requires vic::Numeric<TOther> || ConceptRational<TOther>
    auto operator-(const TOther other)
    {
        return ::vic::Subtract(*this, other);
    }
    template <typename TOther>
        requires vic::Numeric<TOther> || ConceptRational<TOther>
    auto operator*(const TOther other)
    {
        return ::vic::Multiply(*this, other);
    }
    template <typename TOther>
        requires vic::Numeric<TOther> || ConceptRational<TOther>
    auto operator/(const TOther other)
    {
        return ::vic::Devide(*this, other);
    }

    explicit operator T() const { return val; }
    explicit operator bool() const { return val; }

    T val{};
};

//
//
//

template <typename T>
using Giga = Rational<T, 1'000'000'000, 1>;

template <typename T>
using Mega = Rational<T, 1'000'000, 1>;

template <typename T>
using Kilo = Rational<T, 1'000, 1>;

template <typename T>
using Unit = Rational<T, 1, 1>; // todo: SI has no name for this ratio

template <typename T>
using Deci = Rational<T, 1, 10>;

template <typename T>
using Centi = Rational<T, 1, 100>;

template <typename T>
using Milli = Rational<T, 1, 1000>;

template <typename T>
using Micro = Rational<T, 1, 1'000'000>;

// todo: definitions for quarter etc.

//
//
//

template <typename T>
concept ConceptDecimal = ConceptRational<T> && requires(T& rational) {
    typename T::DataType; //
    {
        decltype(T::Num)(T::Num)
    } -> std::integral;
    {
        decltype(T::Denom)(T::Denom)
    } -> std::integral;
    // todo: check that numerator and denominator are powers of 10
};

} // namespace vic