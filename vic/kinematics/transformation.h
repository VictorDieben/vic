#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/translation.h"

#include "vic/linalg/linalg.h"
#include "vic/utils.h"

namespace vic
{
namespace kinematics
{
template <typename T>
struct Transformation
{
public:
    using DataType = T;

    constexpr Transformation() = default;
    Transformation(const Rotation<T>& rotation, //
                   const Translation<T>& translation)
    {
        Assign<0, 0>(mMatrix, rotation.ToMatrix());
        Assign<0, 3>(mMatrix, translation.ToMatrix());
        // 3,3 already set to 1 by default constructor
    }
    explicit Transformation(const Matrix3<T>& rotation, //
                            const Vector3<T>& translation)
    {
        Assign<0, 0>(mMatrix, rotation);
        Assign<0, 3>(mMatrix, translation);
        // 3,3 already set to 1 by default constructor
    }
    constexpr Transformation(const Matrix4<T>& matrix)
        : mMatrix(matrix)
    { }

    friend Transformation<T> operator*(const Transformation<T>& r1, const Transformation<T>& r2)
    {
        // TODO: specialized transformation multiplication
        return Transformation<T>{Matmul(r1.ToMatrix(), r2.ToMatrix())};
    }

    // not const ref, other types of transformations might not have them in memory
    Rotation<T> GetRotation() const { return Rotation{Extract<Matrix3<T>, 0, 0>(mMatrix)}; }
    Translation<T> GetTranslation() const { return Translation{Extract<Vector3<T>, 0, 3>(mMatrix)}; }
    constexpr Matrix4<T> ToMatrix() const { return mMatrix; }

    Transformation<T> Inverse() const
    {
        const auto inv = Transpose(Extract<Matrix3<T>, 0, 0>(mMatrix));
        const auto trans = Matmul(-1., inv, Extract<Vector3<T>, 0, 3>(mMatrix));
        return Transformation<T>{inv, trans};
    }

private:
    Matrix4<T> mMatrix{Identity4<T>{}};
};

} // namespace kinematics
} // namespace vic