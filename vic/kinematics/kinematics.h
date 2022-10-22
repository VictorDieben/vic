#pragma once

#include "vic/linalg/matrices.h"
#include "vic/linalg/tools.h"

#include <array>

namespace vic
{
namespace kinematics
{

using namespace vic::linalg;

// todo: this type is mostly for convenience.
// We will likely never use this library with anything other than a double.
static constexpr Vector3<double> xAxis{{1, 0, 0}};
static constexpr Vector3<double> yAxis{{0, 1, 0}};
static constexpr Vector3<double> zAxis{{0, 0, 1}};

constexpr double pi = 3.14159265358979323846;

template <typename T>
struct Screw
{
public:
    using DataType = T;

    constexpr Screw() = default;
    constexpr Screw(const std::array<DataType, 6>& vec)
        : mVector(vec)
    { }
    constexpr Screw(const Vector6<DataType>& vec)
        : mVector(vec)
    { }
    Screw(const std::array<DataType, 3>& angular, const std::array<DataType, 3>& linear)
    {
        Assign<0, 0>(mVector, Vector3<DataType>{angular});
        Assign<3, 0>(mVector, Vector3<DataType>{linear});
    }
    Screw(const Vector3<DataType>& angular, const Vector3<DataType>& linear)
    {
        Assign<0, 0>(mVector, angular);
        Assign<3, 0>(mVector, linear);
    }

    Vector3<DataType> GetAngular() const { return Extract<Vector3<DataType>, 0, 0>(mVector); }
    Vector3<DataType> GetLinear() const { return Extract<Vector3<DataType>, 3, 0>(mVector); }

private:
    Vector6<DataType> mVector{};
};

// todo: separate types? they will be exactly the same

template <typename T>
using Twist = Screw<T>;

template <typename T>
using Wrench = Screw<T>;

} // namespace kinematics
} // namespace vic