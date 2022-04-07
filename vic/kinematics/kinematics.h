#pragma once

#include "vic/linalg/linalg.h"
#include "vic/linalg/tools.h"

namespace vic
{
namespace kinematics
{

using namespace vic::linalg;

// todo: this type is mostly for convenience.
// We will likely never use this library with anything other than a double.
using DataType = double;

constexpr Vector3<DataType> xAxis{{1, 0, 0}};
constexpr Vector3<DataType> yAxis{{0, 1, 0}};
constexpr Vector3<DataType> zAxis{{0, 0, 1}};

constexpr double pi = 3.14159265358979323846;

struct Screw
{
public:
    constexpr Screw() = default;
    constexpr Screw(const Vector6<DataType>& vec)
        : mVector(vec)
    { }
    constexpr Screw(const Vector3<DataType>& angular, const Vector3<DataType>& linear)
    {
        Assign<0, 0>(mVector, angular);
        Assign<3, 0>(mVector, linear);
    }

    constexpr Vector3<DataType> GetAngular() const { return Extract<Vector3<DataType>, 0, 0>(mVector); }
    constexpr Vector3<DataType> GetLinear() const { return Extract<Vector3<DataType>, 3, 0>(mVector); }

private:
    Vector6<DataType> mVector{};
};

// todo: separate types? they will be exactly the same
using Twist = Screw;
using Wrench = Screw;

} // namespace kinematics
} // namespace vic