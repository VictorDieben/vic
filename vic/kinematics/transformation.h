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

struct Transformation
{
public:
    constexpr Transformation() = default;
    constexpr Transformation(const Rotation& rotation, //
                             const Translation& translation)
        : Transformation(rotation.ToMatrix(), translation.ToMatrix())
    {
        // todo
    }
    constexpr Transformation(const Matrix<DataType, 3, 3>& rotation, //
                             const Vector3<DataType> translation)
    {
        // todo
    }
    constexpr Transformation(const Matrix<DataType, 4, 4>& mat)
        : mMatrix(mat)
    { }

    friend Transformation operator*(const Transformation& r1, const Transformation& r2)
    {
        return Transformation{}; // todo
    }

    constexpr Rotation GetRotation() const
    {
        return Rotation{}; // todo
    }

    constexpr Translation GetTranslation() const
    {
        return Translation{}; // todo
    }

    constexpr Matrix<DataType, 4, 4> ToMatrix() const
    {
        return mMatrix; // todo: construct from rot+trans
    }

    constexpr Transformation Inverse() const
    {
        Rotation rot = GetRotation();
        auto inv = rot.Inverse();
        Translation translation = GetTranslation();
        auto res = Matmul(-1., inv.ToMatrix(), translation.ToMatrix());
        return Transformation{inv, Translation{res}};
    }

private:
    Matrix<DataType, 4, 4> mMatrix{}; // todo: store rotation and translation
};

Transformation TransformationExponent(const Transformation& transform, const DataType theta)
{
    return {}; // todo
}

} // namespace kinematics
} // namespace vic