#pragma once

#include "vic/linalg/matrices/matrix.h"
#include "vic/linalg/tools.h"

namespace vic
{

// 3b1b: https://www[2]outube.com/watch?v=d4EgbgTm0Bg

// todo: decide if this should be a linalg::Vector4<T>
//template <typename T>
//struct Quaternion
//{
//    T w;
//    T x;
//    T y;
//    T z;
//};

//template <typename T>
//using Quaternion = vic::linalg::Vector4<T>;

template <typename T>
struct Quaternion : public vic::linalg::Vector4<T>
{
    using Base = vic::linalg::Vector4<T>;
    using Base::Base;
    T w() const { return Base::Get(0); }
    T x() const { return Base::Get(1); }
    T y() const { return Base::Get(2); }
    T z() const { return Base::Get(3); }
    //
    T& w() { return Base::At(0); }
    T& x() { return Base::At(1); }
    T& y() { return Base::At(2); }
    T& z() { return Base::At(3); }
};

constexpr Quaternion<double> IdentityQuaternion{1., 0., 0., 0.};
constexpr Quaternion<double> zeroQuaternion{0., 0., 0., 0.};

constexpr Quaternion<double> rotate_x_180{0., 1., 0., 0.};
constexpr Quaternion<double> rotate_y_180{0., 0., 1., 0.};
constexpr Quaternion<double> rotate_z_180{0., 0., 0., 1.};

template <typename T>
constexpr Quaternion<T> Add(const Quaternion<T>& a, const Quaternion<T>& b)
{
    // https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    return Quaternion<T>{a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
}

template <typename T>
constexpr Quaternion<T> Multiply(const Quaternion<T>& q, const T& s)
{
    // https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    return Quaternion<T>{q[0] * s, q[1] * s, q[2] * s, q[3] * s};
}

template <typename T>
constexpr Quaternion<T> Multiply(const Quaternion<T>& a, const Quaternion<T>& b)
{
    // https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    return Quaternion<T>{(a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]), //
                         (a[0] * b[1]) + (a[1] * b[0]) - (a[2] * b[3]) + (a[3] * b[2]), //
                         (a[0] * b[2]) + (a[1] * b[3]) + (a[2] * b[0]) - (a[3] * b[1]), //
                         (a[0] * b[3]) - (a[1] * b[2]) + (a[2] * b[1]) + (a[3] * b[0])};
}

template <typename T>
constexpr T Norm(const Quaternion<T>& quat)
{
    return std::sqrt((quat[0] * quat[0]) + //
                     (quat[1] * quat[1]) + //
                     (quat[2] * quat[2]) + //
                     (quat[3] * quat[3]));
}

template <typename T>
constexpr Quaternion<T> Normalize(const Quaternion<T>& quat)
{
    const T inv = T{1.} / Norm(quat);
    return Quaternion<T>{quat[0] * inv, //
                         quat[1] * inv,
                         quat[2] * inv,
                         quat[3] * inv};
}

template <typename T>
constexpr Quaternion<T> Inverse(const Quaternion<T>& quat)
{
    // https://www.mathworks.com/help/aeroblks/quaternioninverse.html
    const T inv = T{1.} / Norm(quat);
    return Quaternion<T>{quat[0] * inv, //
                         -quat[1] * inv,
                         -quat[2] * inv,
                         -quat[3] * inv};
}

template <typename T>
constexpr Quaternion<T> Conjugate(const Quaternion<T>& quat)
{
    return Quaternion<T>{quat[0], -quat[1], -quat[2], -quat[3]};
}

template <typename T>
constexpr linalg::Matrix3<T> ToRotationMatrix(const Quaternion<T>& quat)
{
    const auto [q0, q1, q2, q3] = Unpack(quat);

    //  First row
    const auto r00 = 2 * ((q0 * q0) + (q1 * q1)) - 1;
    const auto r01 = 2 * ((q1 * q2) - (q0 * q3));
    const auto r02 = 2 * ((q1 * q3) + (q0 * q2));

    //  Second row
    const auto r10 = 2 * ((q1 * q2) + (q0 * q3));
    const auto r11 = 2 * ((q0 * q0) + (q2 * q2)) - 1;
    const auto r12 = 2 * ((q2 * q3) - (q0 * q1));

    // Third row
    const auto r20 = 2 * ((q1 * q3) - (q0 * q2));
    const auto r21 = 2 * ((q2 * q3) + (q0 * q1));
    const auto r22 = 2 * ((q0 * q0) + (q3 * q3)) - 1;

    return linalg::Matrix3<T>{r00, r01, r02, r10, r11, r12, r20, r21, r22};
}

template <typename T>
constexpr Quaternion<T> ToQuaternion(const linalg::Matrix3<T>& mat)
{
    // assert(IsSpecialOrthogonal(mat));

    const auto trace = Trace(mat);

    const auto [m00, m01, m02, m10, m11, m12, m20, m21, m22] = Unpack(mat);

    if(trace > 0.)
    {
        const auto s = 0.5 / std::sqrt(trace + 1.);
        return Quaternion<T>{0.25 / s, //
                             (m21 - m12) * s,
                             (m02 - m20) * s,
                             (m10 - m01) * s};
    }
    else
    {
        if(m00 > m11 && m00 > m22)
        {
            const auto s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
            return Quaternion<T>{(m21 - m12) / s, //
                                 0.25 * s,
                                 (m01 + m10) / s,
                                 (m02 + m20) / s};
        }
        else if(m11 > m22)
        {
            const auto s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
            return Quaternion<T>{(m02 - m20) / s, //
                                 (m01 + m10) / s,
                                 0.25 * s,
                                 (m12 + m21) / s};
        }
        else
        {
            const auto s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
            return Quaternion<T>{(m10 - m01) / s, //
                                 (m02 + m20) / s,
                                 (m12 + m21) / s,
                                 0.25 * s};
        }
    }
}

//template <typename T>
//constexpr bool IsEqual(const Quaternion<T>& q1, //
//                       const Quaternion<T>& q2,
//                       const T eps = 1e-10)
//{
//    return (std::abs(q1[0] - q2[0]) < eps) && //
//           (std::abs(q1[1] - q2[1]) < eps) && //
//           (std::abs(q1[2] - q2[2]) < eps) && //
//           (std::abs(q1[3] - q2[3]) < eps);
//}

} // namespace vic