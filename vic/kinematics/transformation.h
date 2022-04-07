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
        : mRotation(rotation)
        , mTranslation(translation)
    { }
    constexpr Transformation(const Matrix<DataType, 3, 3>& rotation, //
                             const Vector3<DataType> translation)
        : mRotation(rotation)
        , mTranslation(translation)
    { }

    friend Transformation operator*(const Transformation& r1, const Transformation& r2)
    {
        return Transformation{}; // todo
    }

    constexpr Rotation GetRotation() const { return mRotation; }

    constexpr Translation GetTranslation() const { return mTranslation; }

    constexpr Matrix<DataType, 4, 4> ToMatrix() const
    {
        return {}; // todo: construct from rot+trans
    }

    constexpr Transformation Inverse() const
    {
        const auto inv = mRotation.Inverse();
        const auto res = Matmul(-1., inv.ToMatrix(), mTranslation.ToMatrix());
        return Transformation{inv, Translation{res}};
    }

private:
    Translation mTranslation{};
    Rotation mRotation{Matrix<DataType, 3, 3>{Identity<DataType, 3>{}}};
};

} // namespace kinematics
} // namespace vic