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

    Vector3<DataType> GetAngular() const
    {
        return {}; // todo
    }
    Vector3<DataType> GetLinear() const
    {
        return {}; // todo
    }

private:
    Vector6<DataType> mVector{};
};

// todo: separate types? they will be exactly the same
using Twist = Screw;
using Wrench = Screw;

Matrix<DataType, 3, 3> ExponentialRotationHelper(const Matrix3<DataType>& b, //
                                                 const Matrix3<DataType>& bSquared, //
                                                 const DataType theta)
{
    return Add(Identity<DataType, 3>{}, //
               Matmul(std::sin(theta), b), //
               Matmul(1 - std::cos(theta), bSquared));
}

Matrix<DataType, 3, 3> ExponentialRotation(const Vector3<DataType>& axis, //
                                           const DataType theta)
{
    const auto b = Matrix3<DataType>{Bracket3(axis)};
    const auto bSquared = Matmul(b, b);
    // todo: bracket * bracket is itself a bracket (i think)
    return ExponentialRotationHelper(b, bSquared, theta);
}

Matrix<DataType, 4, 4> ExponentialTransform(const Screw& screw, //
                                            const DataType theta)
{
    const auto angular = screw.GetAngular();
    const auto linear = screw.GetLinear();
    const auto b = Matrix3<DataType>{Bracket3(angular)}; // todo: Bracket3 overload for matmul
    const auto bSquared = Matmul(b, b);

    const auto tmp = Add(Identity<DataType, 3>{}, //
                         Matmul(1. - std::cos(theta), b), //
                         Matmul(theta - std::sin(theta), bSquared));

    const auto rot = ExponentialRotationHelper(b, bSquared, theta);

    Matrix<DataType, 4, 4> res{};
    // todo: assign rot to [0:3, 0:3]
    // todo: assign tmp*lin to [0:3, 3]
    // todo: assing 1 to [3, 3]
    return res;
}

} // namespace kinematics
} // namespace vic