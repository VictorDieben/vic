#pragma once

#include "vic/linalg/matrices/matrix.h"
#include "vic/linalg/tools.h"

namespace vic
{

// 3b1b: https://www.youtube.com/watch?v=d4EgbgTm0Bg

// todo: decide if this should be a linalg::Vector4<T>
template <typename T>
struct Quaternion
{
    T w;
    T x;
    T y;
    T z;
};

template <typename T>
constexpr bool IsEqual(const Quaternion<T>& q1, //
                       const Quaternion<T>& q2,
                       const T eps = 1e-10)
{
    return (std::abs(q1.w - q2.w) < eps) && //
           (std::abs(q1.x - q2.x) < eps) && //
           (std::abs(q1.y - q2.y) < eps) && //
           (std::abs(q1.z - q2.z) < eps);
}

template <typename T>
constexpr Quaternion<T> Normalize(const Quaternion<T>& quat)
{
    const T inv = T{1.} / std::sqrt((quat.w * quat.w) + //
                                    (quat.x * quat.x) + //
                                    (quat.y * quat.y) + //
                                    (quat.z * quat.z));
    return Quaternion<T>{quat.w * inv, //
                         quat.x * inv,
                         quat.y * inv,
                         quat.z * inv};
}

template <typename T>
constexpr Quaternion<T> Inverse(const Quaternion<T>& quat)
{
    // https://www.mathworks.com/help/aeroblks/quaternioninverse.html
    const T inv = T{1.} / std::sqrt((quat.w * quat.w) + //
                                    (quat.x * quat.x) + //
                                    (quat.y * quat.y) + //
                                    (quat.z * quat.z));
    return Quaternion<T>{quat.w * inv, //
                         -quat.x * inv,
                         -quat.y * inv,
                         -quat.z * inv};
}

template <typename T>
constexpr Quaternion<T> Multiply(const Quaternion<T>& a, const Quaternion<T>& b)
{
    // https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    return Quaternion<T>{(a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z), //
                         (a.w * b.x) + (a.x * b.w) - (a.y * b.z) + (a.z * b.y), //
                         (a.w * b.y) + (a.x * b.z) + (a.y * b.w) - (a.z * b.x), //
                         (a.w * b.z) - (a.x * b.y) + (a.y * b.x) + (a.z * b.w)};
}

template <typename T>
constexpr linalg::Matrix3<T> ToRotationMatrix(const Quaternion<T>& quat)
{
    auto& [q0, q1, q2, q3] = quat;

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
    assert(IsSpecialOrthogonal(mat));

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

} // namespace vic