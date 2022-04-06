#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/kinematics/translation.h"

#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

namespace vic
{
namespace kinematics
{

template <typename T>
Matrix<T, 4, 4> TwistExponential(const Twist& twist, const T theta)
{

    const Bracket3<T> b{vec};
    const Matrix3<T> bSquared = Matmul(b, b);

    return {};
}

} // namespace kinematics
} // namespace vic