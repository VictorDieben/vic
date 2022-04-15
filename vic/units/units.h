#pragma once

#include <format>

namespace vic
{
namespace units
{

// This file will contain templates that offer type checking at several levels of abstractions:
// 1: Buckingham pi; avoid adding length to time or mass etc. No units included.
// 2: Units: difference between m/cm/mm/um/etc.
// 3: todo: Ok Jork, je hebt me omgepraat, Differential Geometry types (vector / covector etc.)

// wrapper for buckingham pi
template <typename T, int mass, int length, int time>
struct BPI
{
public:
    constexpr static int Mass = mass;
    constexpr static int Length = length;
    constexpr static int Time = time;

    using ThisType = BPI<T, mass, length, time>; // bit silly, but avoids clutter

    template <typename T2>
    using SameType = BPI<T2, mass, length, time>; // same type, different representation

    constexpr BPI() = default;
    constexpr BPI(T val)
        : mValue(val)
    { }

    constexpr static auto TypeName()
    {
        return std::format("BPI<{}, {}, {}>", Mass, Length, Time); //
    }

    // operator T() const { return mValue; } // todo: decide if we want this
    constexpr T Get() const { return mValue; }

    template <typename T2>
    constexpr auto operator+(SameType<T2> other)
    {
        using TRet = decltype(T{} + T2{});
        return BPI<TRet, mass, length, time>{mValue + other.Get()};
    }
    template <typename T2>
    constexpr auto operator-(SameType<T2> other)
    {
        using TRet = decltype(T{} - T2{});
        return BPI<TRet, mass, length, time>{mValue - other.Get()};
    }
    constexpr auto operator-() { return BPI<T, mass, length, time>{-mValue}; }

    template <typename T2, int massOther, int lengthOther, int timeOther>
    constexpr auto operator*(BPI<T2, massOther, lengthOther, timeOther> other)
    {
        using TRet = decltype(T{} * T2{});
        return BPI<TRet, mass + massOther, length + lengthOther, time + timeOther>{mValue * other.Get()};
    }

    template <typename T2, int massOther, int lengthOther, int timeOther>
    constexpr auto operator/(BPI<T2, massOther, lengthOther, timeOther> other)
    {
        using TRet = decltype(T{} / T2{});
        return BPI<TRet, mass - massOther, length - lengthOther, time - timeOther>{mValue / other.Get()};
    }

    // TODO: square/square root needs to be specialized for BPI. make members?

protected:
    T mValue{};
};

//

template <typename T>
using Unitless = BPI<T, 0, 0, 0>;

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

//template <typename T>
//using SpecificVolume = BPI<T, -1, 4, -2>; // seems wrong

//

// TODO:
// This struct is only needed to determine the types of variables.
// I'm not really sure how this should be used in practice,
// as you do not know the types of the variables in equations until compilation
template <typename T, typename... Ts>
struct BPIFundamentalPhysics
{
public:
private:
};

//
//
//

// todo: Implement this:
// https://www.win.tue.nl/~lflorack/Extensions/2WAH0CourseNotes.pdf

} // namespace units
} // namespace vic