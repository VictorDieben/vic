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
    constexpr Transformation(const Matrix<DataType, 4, 4>& matrix)
        : mRotation(Extract<Matrix<DataType, 3, 3>, 0, 0>(matrix))
        , mTranslation(Extract<Vector3<DataType>, 0, 3>(matrix))
    { }

    friend Transformation operator*(const Transformation& r1, const Transformation& r2)
    {
        return Transformation{Matmul(r1.ToMatrix(), r2.ToMatrix())}; //
    }

    // not const ref, other types of transformations might not have them in memory
    constexpr Rotation GetRotation() const { return mRotation; }
    constexpr Translation GetTranslation() const { return mTranslation; }
    constexpr Matrix<DataType, 4, 4> ToMatrix() const
    {
        Matrix<DataType, 4, 4> result{};
        Assign<0, 0>(result, mRotation.ToMatrix()); // assign R
        Assign<0, 3>(result, mTranslation.ToMatrix()); // assign p
        result.At(3, 3) = 1.;
        return result;
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