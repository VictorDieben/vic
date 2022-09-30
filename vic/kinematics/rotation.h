#pragma once

#include "vic/kinematics/kinematics.h"

#include "vic/linalg/matmul.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"
#include "vic/utils.h"

#include <cassert>

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
    return std::sqrt(Sum(Dot(vec, vec)));
}

template <typename T, std::size_t rows>
constexpr Matrix<T, rows, 1> Normalize(const Matrix<T, rows, 1>& vec)
{
    const auto norm = Norm(vec);
    return Matmul(vec, T{1} / norm);
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
    return Matmul(Rotate(xAxis, alpha), Rotate(yAxis, beta), Rotate(zAxis, gamma)); // intrinsic x->y'->z''
}


template <typename T>
constexpr Matrix<T, 3, 3> Quaternion( T w,  T x, const T y, const T z)
{
    Matrix<T, 3, 3> R;
    R.At(0,0) = 2*(w*w + x*x)-1;
    R.At(0,1) = 2*(x*y - w*z);
    R.At(0,2) = 2*(x*z + w*y);
    
    R.At(1,0) = 2*(x*y + w*z);
    R.At(1,1) = 2*(w*w + y*y)-1;
    R.At(1,2) = 2*(y*z - w*x);

    R.At(2,0) = 2*(x*z - w*y);
    R.At(2,1) = 2*(y*z + w*x);
    R.At(2,2) = 2*(w*w + z*z)-1;
    
    return R;
}

template <typename T>
constexpr Matrix<T, 3, 3> Quaternion(std::vector<T> wxyz)
{
    return Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
}



template <typename T>
constexpr std::vector<T> R2Q(Matrix<T, 3, 3> R){
    std::vector<T> wxyz;
    auto trace = Trace(R);
    if (trace > 0){
        T s = 0.5 / sqrt(trace + 1.0);
        wxyz[0] = 0.25 / s;
        wxyz[1] = (R.Get(2,1) - R.Get(1,2)) * s;
        wxyz[2] = (R.Get(0,2) - R.Get(2,0)) * s;
        wxyz[3] = (R.Get(1,0) - R.Get(0,1)) * s;
    } else{
        if ( R.Get(0,0) > R.Get(1,1) && R.Get(0,0) > R.Get(2,2) ) {
            T s = 2.0 * sqrt( 1.0 + R.Get(0,0) - R.Get(1,1) - R.Get(2,2));
            wxyz[0] = (R.Get(2,1) - R.Get(1,2) ) / s;
            wxyz[1] = 0.25 * s;
            wxyz[2] = (R.Get(0,1) + R.Get(1,0) ) / s;
            wxyz[3] = (R.Get(0,2) + R.Get(2,0) ) / s;
        } else if (R.Get(1,1) > R.Get(2,2)) {
            T s = 2.0 * sqrt( 1.0 + R.Get(1,1) - R.Get(0,0) - R.Get(2,2));
            wxyz[0]= (R.Get(0,2) - R.Get(2,0) ) / s;
            wxyz[1] = (R.Get(0,1) + R.Get(1,0) ) / s;
            wxyz[2] = 0.25 * s;
            wxyz[3] = (R.Get(1,2) + R.Get(2,1) ) / s;
        } else {
            T s = 2.0 * sqrt( 1.0 + R.Get(2,2) - R.Get(0,0) - R.Get(1,1) );
            wxyz[0] = (R.Get(1,0) - R.Get(0,1) ) / s;
            wxyz[1] = (R.Get(0,2) + R.Get(2,0) ) / s;
            wxyz[2] = (R.Get(1,2) + R.Get(2,1) ) / s;
            wxyz[3] = 0.25 * s;
        }
    }

    return wxyz;
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