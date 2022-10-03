#pragma once

#include "vic/kinematics/kinematics.h"

#include "vic/linalg/add.h"
#include "vic/linalg/matmul.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

#include <cassert>

namespace vic
{
namespace kinematics
{

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
    return Matmul(Rotate(xAxis, alpha), Rotate(yAxis, beta), Rotate(zAxis, gamma)); // intrinsic x->y'->z''
}

template <typename T>
constexpr Matrix<T, 3, 3> Quaternion(const T w, const T x, const T y, const T z)
{
    Matrix<T, 3, 3> R;
    R.At(0, 0) = 2 * (w * w + x * x) - 1;
    R.At(0, 1) = 2 * (x * y - w * z);
    R.At(0, 2) = 2 * (x * z + w * y);

    R.At(1, 0) = 2 * (x * y + w * z);
    R.At(1, 1) = 2 * (w * w + y * y) - 1;
    R.At(1, 2) = 2 * (y * z - w * x);

    R.At(2, 0) = 2 * (x * z - w * y);
    R.At(2, 1) = 2 * (y * z + w * x);
    R.At(2, 2) = 2 * (w * w + z * z) - 1;

    return R;
}

template <typename T>
constexpr Matrix<T, 3, 3> Quaternion(const std::vector<T>& wxyz)
{
    return Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
}

template <typename T>
constexpr Vector4<T> RotToQuaternion(const Matrix<T, 3, 3>& R)
{
    //std::vector<T> wxyz;
    Vector4<T> wxyz;
    const auto trace = Trace(R);
    if(trace > 0.)
    {
        T s = 0.5 / sqrt(trace + 1.0);
        wxyz.At(0, 0) = 0.25 / s;
        wxyz.At(1, 0) = (R.Get(2, 1) - R.Get(1, 2)) * s;
        wxyz.At(2, 0) = (R.Get(0, 2) - R.Get(2, 0)) * s;
        wxyz.At(3, 0) = (R.Get(1, 0) - R.Get(0, 1)) * s;
    }
    else
    {
        if(R.Get(0, 0) > R.Get(1, 1) && R.Get(0, 0) > R.Get(2, 2))
        {
            T s = 2.0 * sqrt(1.0 + R.Get(0, 0) - R.Get(1, 1) - R.Get(2, 2));
            wxyz.At(0, 0) = (R.Get(2, 1) - R.Get(1, 2)) / s;
            wxyz.At(1, 0) = 0.25 * s;
            wxyz.At(2, 0) = (R.Get(0, 1) + R.Get(1, 0)) / s;
            wxyz.At(3, 0) = (R.Get(0, 2) + R.Get(2, 0)) / s;
        }
        else if(R.Get(1, 1) > R.Get(2, 2))
        {
            T s = 2.0 * sqrt(1.0 + R.Get(1, 1) - R.Get(0, 0) - R.Get(2, 2));
            wxyz.At(0, 0) = (R.Get(0, 2) - R.Get(2, 0)) / s;
            wxyz.At(1, 0) = (R.Get(0, 1) + R.Get(1, 0)) / s;
            wxyz.At(2, 0) = 0.25 * s;
            wxyz.At(3, 0) = (R.Get(1, 2) + R.Get(2, 1)) / s;
        }
        else
        {
            T s = 2.0 * sqrt(1.0 + R.Get(2, 2) - R.Get(0, 0) - R.Get(1, 1));
            wxyz.At(0, 0) = (R.Get(1, 0) - R.Get(0, 1)) / s;
            wxyz.At(1, 0) = (R.Get(0, 2) + R.Get(2, 0)) / s;
            wxyz.At(2, 0) = (R.Get(1, 2) + R.Get(2, 1)) / s;
            wxyz.At(3, 0) = 0.25 * s;
        }
    }

    return wxyz;
}

// Transforms an arbirtary 6D vector to a valid rotation matrix.
//     This represenation avoids discontinuities found in Euler or quaternons,
//     which facilitates network regression in machine learning applications.
//     See also: https://doi.org/10.48550/arXiv.1812.07035
// Rep: 6D representation of a rotation.
// returns: A valid rotation matrix
template <typename T>
constexpr Matrix<T, 3, 3> Vec6ToRot(const Vector6<T>& Rep)
{
    Matrix<T, 3, 3> X; // [b1 | b2 | b3]

    const Vector3<DataType> a1 = Extract<Vector3<T>, 0, 0>(Rep);
    const Vector3<DataType> a2 = Extract<Vector3<T>, 3, 0>(Rep);

    const Vector3<DataType> b1 = Normalize(a1);

    const Vector3<DataType> b2 = Normalize(Subtract(a2, Matmul(Dot(b1, a2), b1)));
    const Vector3<DataType> b3 = Cross(b1, b2);

    Assign<0, 0>(X, b1);
    Assign<0, 1>(X, b2);
    Assign<0, 2>(X, b3);

    return X;
}

// Inverse mapping of Vec6ToRot(): rotation matrix to an arbirtary 6D representation vector.
//         See also: https://doi.org/10.48550/arXiv.1812.07035
// X: a valid rotation matrix
// returns a 6D representation of X.
//         Essentially, the last column is dropped.
template <typename T>
constexpr Vector6<T> RotToVec6(const Matrix<T, 3, 3>& X)
{
    Vector6<T> Rep;
    //     [0 1 2
    // X =  3 4 5
    //      6 7 8]
    Rep.At(0, 0) = X.Get(0, 0);
    Rep.At(1, 0) = X.Get(1, 0);
    Rep.At(2, 0) = X.Get(2, 0);

    Rep.At(3, 0) = X.Get(0, 1);
    Rep.At(4, 0) = X.Get(1, 1);
    Rep.At(5, 0) = X.Get(2, 1);
    return Rep;
}

// wrapper around rotation matrix, so that we can later also use quaternions etc.
// also allows us to use * operator
// TODO: make T a template?
struct Rotation
{
public:
    constexpr Rotation() = default;
    explicit constexpr Rotation(const Matrix<DataType, 3, 3>& rotation)
        : mMatrix(rotation)
    { }

    Rotation Inverse() const { return Rotation{Transpose(mMatrix)}; }

    friend Rotation operator*(const Rotation& r1, const Rotation& r2)
    {
        return Rotation(Matmul(r1.mMatrix, r2.mMatrix)); //
    }

    constexpr Matrix<DataType, 3, 3> ToMatrix() const { return mMatrix; }

private:
    Matrix<DataType, 3, 3> mMatrix{Identity<DataType, 3>{}};
};

Rotation RotationExponent(const Rotation& transform, const DataType theta)
{
    return {}; //
}

} // namespace kinematics
} // namespace vic