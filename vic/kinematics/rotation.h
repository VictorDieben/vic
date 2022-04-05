#pragma once

#include "vic/kinematics/kinematics.h"

#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

#include <algorithm>
#include <assert.h>

namespace vic
{
namespace kinematics
{

template <typename T, std::size_t rows, std::size_t cols>
constexpr T Sum(const Matrix<T, rows, cols>& mat)
{
    T sum = 0;
    for(std::size_t i = 0; i < rows; ++i)
        for(std::size_t j = 0; j < cols; ++j)
            sum += mat.Get(i, j);
    return sum;
}

template <typename T, std::size_t rows>
constexpr Matrix<T, rows, 1> Dot(const Matrix<T, rows, 1>& v1, const Matrix<T, rows, 1>& v2)
{
    Matrix<T, rows, 1> res;
    for(std::size_t i = 0; i < rows; ++i)
        res.At(i, 0) = v1.Get(i, 0) * v2.Get(i, 0);
    return res;
}

template <typename T, std::size_t rows>
constexpr T Norm(const Matrix<T, rows, 1>& vec)
{
    return Sum(Dot(vec, vec));
}

template <typename T>
constexpr Matrix<T, 3, 3> Rotate(const Vector3<T>& vec, const T angle)
{
    assert(Abs(Norm(vec) - 1.) < 1e-10); // vec should have length 1
    constexpr Identity<T, 3> identity{};
    const Bracket3<T> b{vec};
    const Matrix3<T> bSquared = Matmul(b, b);
    const Matrix3<T> tmp1 = Matmul(std::sin(angle), b);
    const Matrix3<T> tmp2 = Matmul(1. - std::cos(angle), bSquared);
    return Add(identity, tmp1, tmp2);
}

template <typename T>
constexpr Matrix<T, 3, 3> EulerAngles(const T alpha, const T beta, const T gamma)
{
    // TODO(vicdie): order of matrix multiplications
    return Matmul(Rotate(xAxis, alpha), Rotate(yAxis, beta), Rotate(zAxis, gamma));
}

// wrapper around rotation matrix, so that we can later also use quaternions etc.
// also allows us to use * operator
// TODO(vicdie): make T a template?
struct Rotation
{
public:
    constexpr Rotation() = default;
    constexpr Rotation(const Matrix<DataType, 3, 3>& rotation)
        : mMatrix(rotation)
    { }

    constexpr Rotation Inverse() const { return Rotation{Transpose(mMatrix)}; }

    friend Rotation operator*(const Rotation& r1, const Rotation& r2)
    {
        return Rotation(Matmul(r1.mMatrix, r2.mMatrix)); //
    }

    Matrix<DataType, 3, 3> ToMatrix() const { return mMatrix; }

private:
    Matrix<DataType, 3, 3> mMatrix{Identity<DataType, 3>{}};
};

} // namespace kinematics
} // namespace vic