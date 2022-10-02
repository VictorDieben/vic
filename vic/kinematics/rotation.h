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
constexpr Matrix<T, rows, 1> (const Matrix<T, rows, 1>& v1, const Matrix<T, rows, 1>& v2)
{
    Matrix<T, rows, 1> res;
    for(std::size_t i = 0; i < rows; ++i)
        res.At(i, 0) = v1.Get(i, 0) * v2.Get(i, 0);
    return res;
}

template <typename T, std::size_t rows>
constexpr T Norm(const Matrix<T, rows, 1>& vec)
{
    return std::sqrt(Sum((vec, vec)));
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
constexpr std::vector<T> RotToQuaternion(Matrix<T, 3, 3> R){
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

/*!
 *    @brief  Transforms an arbirtary 6D vector to a valid rotation matrix.
 *            This represenation avoids discontinuities found in Euler or quaternons,  
 *            which facilitates network regression in machine learning applications.
              See also: https://doi.org/10.48550/arXiv.1812.07035
 *    @param  Rep 6D representation of a rotation. 
 *    @return A valid rotation matrix
 */
template <typename T>
constexpr Matrix<T, 3, 3> Vec6ToRot(Vector6 Rep)
{
    Matrix<T, 3, 3> X; // [b1 | b2 | b3]

    Vector3<DataType> a1 = Extract<Vector3<DataType>, 0, 0>(Rep);
    Vector3<DataType> a2 = Extract<Vector3<DataType>, 3, 0>(Rep);
    
    Vector3<DataType> b1 = Norm(a1);
    Vector3<DataType> b2 = Norm(a2 - ((b1,a2)*b1));
    Vector3<DataType> b3 = Cross(b1,b2);

    Assign<0,0>(X, b1);
    Assign<0,1>(X, b2);
    Assign<0,2>(X, b3);

    return X;
}

/*!
 *    @brief  Inverse mapping of Vec6ToRot(): rotation matrix to an arbirtary 6D representation vector.
              See also: https://doi.org/10.48550/arXiv.1812.07035
 *    @param X a valid rotation matrix
 *    @return 6D representation of X. 
 *                Essentially, the last column is dropped.
 */
template <typename T>
constexpr  Vec6 RotToVec6(Matrix<T, 3, 3> X)
{
    Vector6 Rep;
    //     [0 1 2
    // X =  3 4 5
    //      6 7 8]
    Rep[0] = X[0]; // X.Get(0,0)
    Rep[1] = X[3]; // X.Get(1,0)
    Rep[2] = X[6]; // X.Get(2,0)

    Rep[3] = X[1]; // X.Get(0,1)
    Rep[4] = X[4]; // X.Get(1,1)
    Rep[5] = X[7]; // X.Get(2,1)
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