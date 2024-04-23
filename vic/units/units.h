#pragma once

#include "vic/utils/concepts.h"
#include <cmath>
#include <format>
#include <utility>

namespace vic
{
namespace units
{

// This file will contain templates that offer type checking at several levels of abstractions:
// 1: Buckingham pi; avoid adding length to time or mass etc. No units included.
// 2: Units: difference between m/cm/mm/um/etc.
// 3: todo: Ok Jork, je hebt me omgepraat, Differential Geometry types (vector / covector etc.)

// todo: compiler flags to turn BPI off, e.g. make Length<double> just a synonym for double

template <typename T>
concept ConceptBPI = requires(T bpi) {
    T::Mass; //
    T::Length; //
    T::Time; //
    typename T::template SameType<double>; // Get the BPI with a different representation
};

// wrapper for buckingham pi
template <typename T, int mass, int length, int time>
// requires(Numeric<T>) NOTE: no clue why this does not work, temporarily solved with static assert
struct BPI
{
public:
    static_assert(Numeric<T> && "Incompatible data type"); // todo: should be removed if we allow vic::Rational, or something like complex numbers
    static_assert(!ConceptBPI<T> && "BPI should not be nested!");

    constexpr static int Mass = mass;
    constexpr static int Length = length;
    constexpr static int Time = time;

    using DataType = T;

    // using ThisType = BPI<T, mass, length, time>; // bit silly, but avoids clutter

    template <typename T2>
    using SameType = BPI<T2, mass, length, time>; // same type, different representation

    explicit constexpr BPI() = default;
    constexpr BPI(const T val)
        : mValue(val)
    { }

    constexpr ~BPI() = default;

    //
    //
    //

    constexpr BPI(const BPI& other) noexcept { mValue = other.mValue; }
    constexpr BPI(const BPI&& other) noexcept { mValue = other.mValue; }

    constexpr BPI& operator=(const BPI& other) noexcept
    {
        mValue = other.mValue;
        return *this;
    }

    constexpr BPI& operator=(const BPI&& other) noexcept
    {
        mValue = other.mValue;
        return *this;
    }

    //

    //constexpr BPI<T, mass, length, time>(const T& other) noexcept { mValue = other; }
    //constexpr BPI<T, mass, length, time>(const T&& other) noexcept { mValue = other; }
    //constexpr BPI<T, mass, length, time>& operator=(const T& other) noexcept
    //{
    //    mValue = other;
    //    return *this;
    //}
    //constexpr BPI<T, mass, length, time>& operator=(const T&& other) noexcept
    //{
    //    mValue = other;
    //    return *this;
    //}

    //
    //
    //

    constexpr static auto TypeName()
    {
        return std::format("BPI<{}, {}, {}>", Mass, Length, Time); //
    }

    constexpr T Get() const { return mValue; }
    // explicit constexpr operator T() const { return mValue; }

protected:
    T mValue{};
};

template <typename T1, typename T2>
static constexpr bool BPICompatible = (T1::Mass == T2::Mass) && (T1::Length == T2::Length) && (T1::Time == T2::Time);

//

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto Add(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
    {
        using TNumeric = T2::template SameType<T1>;
        return Add(TNumeric{first}, second);
    }
    else if constexpr(Numeric<T2>)
    {
        using TNumeric = T1::template SameType<T2>;
        return Add(first, TNumeric{second});
    }
    else
    {
        static_assert(BPICompatible<T1, T2>); // cannot add e.g. distance to volume
        using TRet = decltype(typename T1::DataType{} + typename T2::DataType{});
        return T1::template SameType<TRet>(first.Get() + second.Get());
    }
}

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto operator+(const T1 lhs, const T2 rhs)
{
    return Add(lhs, rhs);
}

template <typename T>
    requires ConceptBPI<T>
constexpr auto operator+(const T& item)
{
    return item;
}

//
// -
//

template <typename T>
    requires ConceptBPI<T>
constexpr auto Negative(const T& item)
{
    return T{-item.Get()};
}

template <typename T>
    requires ConceptBPI<T>
constexpr auto operator-(const T& item)
{
    return Negative(item);
}

//
//
//

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto Subtract(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
    {
        using TNumeric = T2::template SameType<T1>;
        return Subtract(TNumeric{first}, second);
    }
    else if constexpr(Numeric<T2>)
    {
        using TNumeric = T1::template SameType<T2>;
        return Subtract(first, TNumeric{second});
    }
    else
    {
        static_assert(BPICompatible<T1, T2>);
        using TRet = decltype(typename T1::DataType{} + typename T2::DataType{});
        return T1::template SameType<TRet>(first.Get() - second.Get());
    }
}

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto operator-(const T1 lhs, const T2 rhs)
{
    return Subtract(lhs, rhs);
}

//
//  /
//

//
template <typename T>
using Unitless = BPI<T, 0, 0, 0>;

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto Division(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
        return Division(Unitless{first}, second);
    else if constexpr(Numeric<T2>)
        return Division(first, Unitless{second});
    else
    {
        using TRet = decltype(typename T1::DataType{} / typename T2::DataType{});
        constexpr const int ResultMass = T1::Mass - T2::Mass;
        constexpr const int ResultLength = T1::Length - T2::Length;
        constexpr const int ResultTime = T1::Time - T2::Time;
        return BPI<TRet, ResultMass, ResultLength, ResultTime>(first.Get() / second.Get());
    }
}
template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto operator/(const T1 lhs, const T2 rhs)
{
    return Division(lhs, rhs);
}

//
// *
//

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto Multiplication(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
        return Multiplication(Unitless{first}, second); // T2{second.Get() * first};
    else if constexpr(Numeric<T2>)
        return Multiplication(first, Unitless{second});
    else
    {
        using TRet = decltype(typename T1::DataType{} * typename T2::DataType{});
        //using TRet = decltype(std::declvar<T1::DataType>() * std::declvar<T2::DataType>());
        constexpr const int ResultMass = T1::Mass + T2::Mass;
        constexpr const int ResultLength = T1::Length + T2::Length;
        constexpr const int ResultTime = T1::Time + T2::Time;
        return BPI<TRet, ResultMass, ResultLength, ResultTime>(first.Get() * second.Get());
    }
}
template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto operator*(const T1 lhs, const T2 rhs)
{
    return Multiplication(lhs, rhs);
}

//
// %
//

template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto Remainder(const T1 first, const T2 second)
{
    if constexpr(Numeric<T1>)
        return Remainder(Unitless{first}, second); // T2{second.Get() * first};
    else if constexpr(Numeric<T2>)
        return Remainder(first, Unitless{second});
    else
    {
        using TRet = decltype((typename T1::DataType{}) % (typename T2::DataType{}));
        constexpr const int ResultMass = T1::Mass;
        constexpr const int ResultLength = T1::Length;
        constexpr const int ResultTime = T1::Time;
        return BPI<TRet, ResultMass, ResultLength, ResultTime>(first.Get() % second.Get());
    }
}
template <typename T1, typename T2>
    requires(ConceptBPI<T1> && ConceptBPI<T2>) || //
            (ConceptBPI<T1> && Numeric<T2>) || //
            (Numeric<T1> && ConceptBPI<T2>)
constexpr auto operator%(const T1 lhs, const T2 rhs)
{
    return Remainder(lhs, rhs);
}

// geometric

template <typename T>
using Length = BPI<T, 0, 1, 0>;

template <typename T>
using Area = BPI<T, 0, 2, 0>;

template <typename T>
using Volume = BPI<T, 0, 3, 0>;

// kinematic

template <typename T>
using Time = BPI<T, 0, 0, 1>;

template <typename T>
using AngularVelocity = BPI<T, 0, 0, -1>;

template <typename T>
using Frequency = BPI<T, 0, 0, -1>;

template <typename T>
using Velocity = BPI<T, 0, 1, -1>;

template <typename T>
using KinematicViscosity = BPI<T, 0, 3, -1>;

template <typename T>
using AngularAcceleration = BPI<T, 0, 1, -2>;

template <typename T>
using Acceleration = BPI<T, 0, 1, -2>;

// dynamic

template <typename T>
using Density = BPI<T, 1, -3, 0>;

template <typename T>
using Mass = BPI<T, 1, 0, 0>;

template <typename T>
using MoI = BPI<T, 1, 2, 0>;

template <typename T>
using DynamicViscosity = BPI<T, 1, -1, -1>;

template <typename T>
using MassFlow = BPI<T, 1, 0, -1>;

template <typename T>
using Momentum = BPI<T, 1, 1, -1>;

// todo: Impulse

template <typename T>
using AngularMomentum = BPI<T, 1, 2, -1>;

template <typename T>
using SpecificWeight = BPI<T, 1, -2, -2>;

template <typename T>
using Pressure = BPI<T, 1, -2, 0>;

template <typename T>
using UnitShearStress = BPI<T, 1, -1, -2>;

//template <typename T>
//using ModulusOfElasticity = BPI<T, 1, -1, -2>;

template <typename T>
using SurfaceTension = BPI<T, 1, 0, -2>;

template <typename T>
using Force = BPI<T, 1, 1, -2>;

// template <typename T>
// using Energy = BPI<T, 1, 1, -2>;

template <typename T>
using Work = BPI<T, 1, 2, -2>;

//template <typename T>
//using Torque = BPI<T, 1, 1, -2>;

template <typename T>
using Power = BPI<T, 1, 2, -3>;

// todo:
// BPI uses 3 values,
// a different system uses 7: SoME2.
// think about changing these classes
// example:
// https://www.youtube.com/watch?v=bI-FS7aZJpY

// todo: Implement this:
// https://www.win.tue.nl/~lflorack/Extensions/2WAH0CourseNotes.pdf

} // namespace units
} // namespace vic

//
// Some operators from std need to be specialized for BPI units
//

namespace std
{

// overload for sqrt
template <typename T>
    requires ConceptBPI<T>
constexpr auto sqrt(const T bpi) noexcept
{
    // make sure that taking the square root makes sense.
    // e.g. you cannot take the square root of a volume
    static_assert((T::Mass % 2 == 0) && (T::Length % 2 == 0) && (T::Time % 2 == 0));

    using TRet = typename T::DataType;
    constexpr const int mass = T::Mass / 2;
    constexpr const int length = T::Length / 2;
    constexpr const int time = T::Time / 2;
    return vic::units::BPI<TRet, mass, length, time>(::std::sqrt(bpi.Get()));
}

// overload for cbrt (cubic root)
template <typename T>
    requires ConceptBPI<T>
constexpr auto cbrt(const T bpi) noexcept
{
    // make sure that taking the cubic root makes sense.
    // e.g. you cannot take the cubic root of an area
    static_assert((T::Mass % 3 == 0) && (T::Length % 3 == 0) && (T::Time % 3 == 0));

    using TRet = typename T::DataType;
    constexpr const int mass = T::Mass / 3;
    constexpr const int length = T::Length / 3;
    constexpr const int time = T::Time / 3;
    return vic::units::BPI<TRet, mass, length, time>(::std::cbrt(bpi.Get()));
}

} // namespace std
