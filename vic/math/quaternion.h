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
    const T& w() const { return Base::Get(0); }
    const T& x() const { return Base::Get(1); }
    const T& y() const { return Base::Get(2); }
    const T& z() const { return Base::Get(3); }
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
constexpr Quaternion<T> Negation(const Quaternion<T>& q)
{
    return Quaternion<T>{-q[0], -q[1], -q[2], -q[3]};
}

template <typename T>
constexpr Quaternion<T> Add(const Quaternion<T>& a, const Quaternion<T>& b)
{
    // https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    return Quaternion<T>{a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
}

template <typename T>
constexpr Quaternion<T> Subtract(const Quaternion<T>& a, const Quaternion<T>& b)
{
    // a - b
    return Add(a, Negation(b));
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
constexpr Quaternion<T> Vector(const Quaternion<T>& quat)
{
    return Quaternion<T>{0., quat[1], quat[2], quat[3]};
}

template <typename T>
constexpr Quaternion<T> Scalar(const Quaternion<T>& quat)
{
    return Quaternion<T>{quat.w(), 0., 0., 0.};
}

template <typename T>
constexpr Quaternion<T> Rotation(const T theta, const linalg::Vector3<T>& axis)
{
    const auto n = vic::linalg::Normalize(axis);
    const T c = std::cos(theta);
    const T s = std::sin(theta);
    return Quaternion<T>{c, s * n[0], s * n[1], s * n[2]};
}

template <typename T>
constexpr Quaternion<T> ToQuaternion(const linalg::Vector3<T>& vec)
{
    return Quaternion<T>{0., vec[0], vec[1], vec[2]};
}

template <typename T>
constexpr linalg::Vector3<T> Apply(const linalg::Vector3<T>& vec, const Quaternion<T>& quat)
{
    // apply a quaternion rotation to a vector
    const Quaternion<T> quatVec = ToQuaternion(vec);
    const auto res = Multiply(Multiply(quat, quatVec), Conjugate(quat));
    return linalg::Vector3<T>{res.x(), res.y(), res.z()};
}

template <typename T>
constexpr Quaternion<T> Exponent(const Quaternion<T>& quat)
{
    const auto [w, x, y, z] = Unpack(quat);
    const T a = std::sqrt((x * x) + (y * y) + (z * z));
    const T sinAOverA = std::sin(a) / a;
    return Quaternion<T>{std::cos(a), //
                         x * sinAOverA,
                         y * sinAOverA,
                         z * sinAOverA};
}

template <typename T>
constexpr Quaternion<T> ExponentApprox(const Quaternion<T>& quat)
{
    const auto [w, x, y, z] = Unpack(quat);
    const T alpha = std::sqrt((x * x) + (y * y) + (z * z));
    //const T a2 = a * a;
    //const T a4 = a2 * a2;
    //const T a6 = a4 * a2;
    //const T cosApprox = 1. //
    //                    - (a2 / 2.) //
    //                    + (a4 / (4 * 3 * 2)) //
    //                    - (a6 / (6 * 5 * 4 * 3 * 2));
    //const T sinApprox = 1. //
    //                    - (a2 / (3 * 2)) //
    //                    + (a4 / (5 * 4 * 3 * 2)) //
    //                    - (a6 / (7 * 6 * 5 * 4 * 3 * 2));

    //return Quaternion<T>{cosApprox, //
    //                     x * sinApprox,
    //                     y * sinApprox,
    //                     z * sinApprox};

    const T ea = std::exp(w);
    const T a2 = alpha * alpha;
    const T a4 = a2 * a2;
    const T a6 = a4 * a2;
    const T cosApprox = 1. //
                        - (a2 / 2.) //
                        + (a4 / (4. * 3 * 2)) //
                        - (a6 / (6. * 5 * 4 * 3 * 2));
    const T sinApprox = 1. //
                        - (a2 / (3. * 2)) //
                        + (a4 / (5. * 4 * 3 * 2)) //
                        - (a6 / (7. * 6 * 5 * 4 * 3 * 2));

    return Quaternion<T>{ea * cosApprox, //
                         ea * x * sinApprox,
                         ea * y * sinApprox,
                         ea * z * sinApprox};
}

template <typename T>
constexpr linalg::Matrix3<T> ToRotationMatrix(const Quaternion<T>& quat)
{
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm

    const auto [w, x, y, z] = Unpack(quat);

    const T x2 = x * x;
    const T y2 = y * y;
    const T z2 = z * z;
    const T xy = x * y;
    const T xz = x * z;
    const T yz = y * z;
    const T wx = w * x;
    const T wy = w * y;
    const T wz = w * z;

    const T r00 = 1.0 - 2.0 * (y2 + z2);
    const T r01 = 2.0 * (xy - wz);
    const T r02 = 2.0 * (xz + wy);

    const T r10 = 2.0 * (xy + wz);
    const T r11 = 1.0 - 2.0 * (x2 + z2);
    const T r12 = 2.0 * (yz + wx);

    const T r20 = 2.0f * (xz + wy);
    const T r21 = 2.0f * (yz - wx);
    const T r22 = 1.0f - 2.0f * (x2 + y2);

    return linalg::Matrix3<T>{r00, r01, r02, r10, r11, r12, r20, r21, r22};
}

template <typename T>
constexpr Quaternion<T> ToQuaternion(const linalg::Matrix3<T>& mat)
{
    //// assert(IsSpecialOrthogonal(mat));

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
            const auto invS = 1. / s;
            return Quaternion<T>{(m21 - m12) * invS, //
                                 0.25 * s,
                                 (m01 + m10) * invS,
                                 (m02 + m20) * invS};
        }
        else if(m11 > m22)
        {
            const auto s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
            const auto invS = 1. / s;
            return Quaternion<T>{(m02 - m20) * invS, //
                                 (m01 + m10) * invS,
                                 0.25 * s,
                                 (m12 + m21) * invS};
        }
        else
        {
            const auto s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
            const auto invS = 1. / s;
            return Quaternion<T>{(m10 - m01) * invS, //
                                 (m02 + m20) * invS,
                                 (m12 + m21) * invS,
                                 0.25 * s};
        }
    }

    // https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf

    //const auto [m00, m01, m02, m10, m11, m12, m20, m21, m22] = Unpack(mat);

    //if(m22 < 0.)
    //{
    //    if(m00 > m11)
    //    {
    //        const auto t = 1. + m00 - m11 - m22;
    //        const auto f = .5 / std::sqrt(t);
    //        return Quaternion<T>{t * f, //
    //                             (m01 + m10) * f,
    //                             (m20 + m02) * f,
    //                             (m12 - m21) * f};
    //    }
    //    else
    //    {
    //        const auto t = 1. - m00 + m11 - m22;
    //        const auto f = .5 / std::sqrt(t);
    //        return Quaternion<T>{(m01 + m10) * f, //
    //                             t * f,
    //                             (m12 + m21) * f,
    //                             (m20 - m02) * f};
    //    }
    //}
    //else
    //{
    //    if(m00 < -m11)
    //    {
    //        const auto t = 1. - m00 - m11 + m22;
    //        const auto f = .5 / std::sqrt(t);
    //        return Quaternion<T>{(m20 + m02) * f, //
    //                             (m12 + m21) * f,
    //                             t * f,
    //                             (m01 - m10) * f};
    //    }
    //    else
    //    {
    //        const auto t = 1. + m00 + m11 + m22;
    //        const auto f = .5 / std::sqrt(t);
    //        return Quaternion<T>{(m12 - m21) * f, //
    //                             (m20 - m02) * f,
    //                             (m01 - m10) * f,
    //                             t * f};
    //    }
    //}
}

} // namespace vic