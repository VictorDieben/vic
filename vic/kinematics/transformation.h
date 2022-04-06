#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/translation.h"

#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

#include <algorithm>
#include <assert.h>

namespace vic
{
namespace kinematics
{

Transformation TransformationExponent(const Transformation& transform, const DataType theta)
{
    return {}; // todo
}

struct Transformation
{
public:
    constexpr Transformation() = default;
    constexpr Transformation(const Matrix<DataType, 3, 3>& rotation, //
                             const Vector3<DataType> translation)
    { }

    constexpr Transformation Inverse() const
    {
        return {}; // todo
    }

    friend Transformation operator*(const Transformation& r1, const Transformation& r2)
    {
        return Transformation{}; // todo
    }

    Rotation GetRotation() const
    {
        return Rotation{}; // todo
    }

    Translation GetTranslation() const
    {
        return Translation{}; // todo
    }

    Matrix<DataType, 4, 4> ToMatrix() const { return mMatrix; }

private:
    Matrix<DataType, 4, 4> mMatrix{};
};

} // namespace kinematics
} // namespace vic