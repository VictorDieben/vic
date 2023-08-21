#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/kinematics/translation.h"

#include "vic/linalg/linalg.h"
#include "vic/utils.h"

#include <cmath>

namespace vic
{
namespace kinematics
{

// inspired by boost, put any implementation details in detail namespace
namespace detail
{

template <typename T>
Matrix3<T> ExponentialRotationHelper(const Matrix3<T>& b, //
                                     const Matrix3<T>& bSquared, //
                                     const T theta)
{
    return Add(Identity3<T>{}, //
               Matmul(std::sin(theta), b),
               Matmul(1. - std::cos(theta), bSquared));
}
} // namespace detail

template <typename T>
Matrix3<T> ExponentialRotation(const Vector3<T>& axis, //
                               const T theta)
{
    const auto b = Matrix3<T>{Bracket3(axis)};
    const auto bSquared = Matmul(b, b);
    return detail::ExponentialRotationHelper(b, bSquared, theta);
}

template <typename T>
Matrix4<T> ExponentialTransform(const Screw<T>& screw, //
                                const T theta)
{
    const auto angular = screw.GetAngular();
    const auto linear = screw.GetLinear();
    const auto b = To<Matrix3<T>>(Bracket3(angular)); // todo: Bracket3 overload for matmul
    const auto bSquared = Matmul(b, b);

    const auto tmp = Add(Matmul(Identity3<T>{}, theta), //
                         Matmul(1. - std::cos(theta), b), //
                         Matmul(theta - std::sin(theta), bSquared));

    const Matrix3<T> rot = detail::ExponentialRotationHelper(b, bSquared, theta);

    Matrix4<T> res{};
    Assign<0, 0>(res, rot); // assign rot to [0:3, 0:3]
    Assign<0, 3>(res, Matmul(tmp, linear)); // assign tmp*lin to [0:3, 3]
    res.At(3, 3) = 1.; // assing 1 to [3, 3]
    return res;
}

} // namespace kinematics
} // namespace vic