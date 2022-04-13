#pragma once

namespace vic
{
namespace units
{

// This file will contain templates that offer type checking at several levels of abstractions:
// 1: Buckingham pi; avoid adding length to time or mass etc. No units included.
// 2: Units: difference between m/cm/mm/um/etc.
// 3: todo: Ok Jork, je hebt me omgepraat, Differential Geometry types (vector / covector etc.)

// wrapper for buckingham pi
template <typename T, int length, int time, int mass>
struct BPI
{
public:
    constexpr static int Length = length;
    constexpr static int Time = time;
    constexpr static int Mass = mass;

    constexpr BPI(T val)
        : mValue(val)
    { }

    // operator T() const { return mValue; } // todo: decide if we want this
    constexpr T Get() const { return mValue; }

    template <typename T2>
    constexpr auto operator+(BPI<T2, length, time, mass> other)
    {
        using TRet = decltype(T{} + T2{});
        return BPI<TRet, length, time, mass>{mValue + other.Get()};
    }

    template <typename T2, int lengthOther, int timeOther, int massOther>
    constexpr auto operator*(BPI<T2, lengthOther, timeOther, massOther> other)
    {
        using TRet = decltype(T{} * T2{});
        return BPI<TRet, length + lengthOther, time + timeOther, mass + massOther>{mValue * other.Get()};
    }

    template <typename T2, int lengthOther, int timeOther, int massOther>
    constexpr auto operator/(BPI<T2, lengthOther, timeOther, massOther> other)
    {
        using TRet = decltype(T{} / T2{});
        return BPI<TRet, length - lengthOther, time - timeOther, mass - massOther>{mValue / other.Get()};
    }

protected:
    T mValue{};
};

//

template <typename T>
using Unitless = BPI<T, 0, 0, 0>;

//

template <typename T>
using Length = BPI<T, 1, 0, 0>;

template <typename T>
using Area = BPI<T, 2, 0, 0>;

template <typename T>
using Volume = BPI<T, 3, 0, 0>;

//

template <typename T>
using Time = BPI<T, 0, 1, 0>;

template <typename T>
using Frequency = BPI<T, 0, -1, 0>;

//

template <typename T>
using Mass = BPI<T, 0, 0, 1>;

} // namespace units
} // namespace vic