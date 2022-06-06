#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/kinematics/translation.h"

#include "vic/linalg/add.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

#include <cmath>

namespace vic
{
namespace kinematics
{

// inspired by boost, put any implementation details in detail namespace
namespace detail
{
Matrix<DataType, 3, 3> ExponentialRotationHelper(const Matrix3<DataType>& b, //
                                                 const Matrix3<DataType>& bSquared, //
                                                 const DataType theta)
{
    return Add(Identity<DataType, 3>{}, //
               Matmul(std::sin(theta), b),
               Matmul(1. - std::cos(theta), bSquared));
}
} // namespace detail

Matrix<DataType, 3, 3> ExponentialRotation(const Vector3<DataType>& axis, //
                                           const DataType theta)
{
    const auto b = Matrix3<DataType>{Bracket3(axis)};
    const auto bSquared = Matmul(b, b);
    return detail::ExponentialRotationHelper(b, bSquared, theta);
}

Matrix<DataType, 4, 4> ExponentialTransform(const Screw& screw, //
                                            const DataType theta)
{
    const auto angular = screw.GetAngular();
    const auto linear = screw.GetLinear();
    const auto b = Matrix3<DataType>{Bracket3(angular)}; // todo: Bracket3 overload for matmul
    const auto bSquared = Matmul(b, b);

    const auto tmp = Add(Matmul(Identity<DataType, 3>{}, theta), //
                         Matmul(1. - std::cos(theta), b), //
                         Matmul(theta - std::sin(theta), bSquared));

    const Matrix3<DataType> rot = detail::ExponentialRotationHelper(b, bSquared, theta);

    Matrix<DataType, 4, 4> res{};
    Assign<0, 0>(res, rot); // assign rot to [0:3, 0:3]
    Assign<0, 3>(res, Matmul(tmp, linear)); // assign tmp*lin to [0:3, 3]
    res.At(3, 3) = 1.; // assing 1 to [3, 3]
    return res;
}

} // namespace kinematics
} // namespace vic