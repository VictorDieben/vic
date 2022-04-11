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
    {
        Assign<0, 0>(mMatrix, rotation.ToMatrix());
        Assign<0, 3>(mMatrix, translation.ToMatrix());
    }
    constexpr Transformation(const Matrix<DataType, 3, 3>& rotation, //
                             const Vector3<DataType> translation)
    {
        Assign<0, 0>(mMatrix, rotation);
        Assign<0, 3>(mMatrix, translation);
    }
    constexpr Transformation(const Matrix<DataType, 4, 4>& matrix)
        : mMatrix(matrix)
    { }

    friend Transformation operator*(const Transformation& r1, const Transformation& r2)
    {
        // TODO: specialized transformation multiplication
        return Transformation{Matmul(r1.ToMatrix(), r2.ToMatrix())};
    }

    // not const ref, other types of transformations might not have them in memory
    constexpr Rotation GetRotation() const { return Rotation{Extract<Matrix<DataType, 3, 3>, 0, 0>(mMatrix)}; }
    constexpr Translation GetTranslation() const { return Translation{Extract<Vector3<DataType>, 0, 3>(mMatrix)}; }
    constexpr Matrix<DataType, 4, 4> ToMatrix() const { return mMatrix; }

    constexpr Transformation Inverse() const
    {
        const auto inv = Transpose(Extract<Matrix<DataType, 3, 3>, 0, 0>(mMatrix));
        const auto trans = Matmul(-1., inv, Extract<Vector3<DataType>, 0, 3>(mMatrix));
        return Transformation{inv, trans};
    }

private:
    Matrix<DataType, 4, 4> mMatrix{Identity<DataType, 4>{}};
};

} // namespace kinematics
} // namespace vic