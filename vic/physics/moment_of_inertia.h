#pragma once

#include <tuple>

#include "vic/linalg/linalg.h"

namespace vic
{
//template <typename T>
//using Mass = T;

template <typename T>
using CG = ::vic::linalg::Vector2<T>;

template <typename T>
using MoIDiag = ::vic::linalg::Diagonal3<T>;

template <typename T>
using MoI = ::vic::linalg::Matrix3<T>;

template <typename T>
concept ConceptMoI = ::vic::linalg::ConceptConstexprMatrix<T> && requires(T mat) {
    requires(T::GetRows() == 3 && T::GetColumns() == 3); //
};

template <typename T>
MoIDiag<T> MoICuboid(const T h, const T w, const T l, const T mass)
{
    const T oneTwelvth = 1. / 12.;
    const T h2 = h * h;
    const T w2 = w * w;
    const T l2 = l * l;
    return MoIDiag<T>{oneTwelvth * mass * (w2 + l2), //
                      oneTwelvth * mass * (l2 + h2),
                      oneTwelvth * mass * (h2 + w2)}; //
}

template <typename T>
MoIDiag<T> MoISphere(const T radius, const T mass)
{
    const T i = (2. / 5.) * mass * radius * radius;
    return MoIDiag<T>{i, i, i};
}

template <typename T>
MoIDiag<T> MoISphereThinWalled(const T radius, const T mass)
{
    const T i = (2. / 3.) * mass * radius * radius;
    return MoIDiag<T>{i, i, i};
}

template <typename T>
ConceptMoI auto MoISphereThickWalled(const T r1, //
                                     const T r2,
                                     const T mass)
{
    // note: manually computing all powers are probably not needed, the compiler can do this perfectly fine,
    const T r1_2 = r1 * r1;
    const T r1_3 = r1_2 * r1;
    const T r1_5 = r1_2 * r1_3;

    const T r2_2 = r2 * r2;
    const T r2_3 = r2_2 * r2;
    const T r2_5 = r2_2 * r2_3;

    const T I = (2. / 5.) * mass * (r2_5 - r1_5) / (r2_3 - r1_3);

    return MoIDiag<T>{I, I, I};
}

template <typename T>
ConceptMoI auto MoICylinder(const T radius, //
                            const T height,
                            const T mass)
{
    const T r2 = radius * radius;
    const T Iz = .5 * mass * r2;
    const T Ix_y = (1. / 12.) * mass * ((3 * r2) + (height * height));
    return MoIDiag<T>{Ix_y, Ix_y, Iz}; //
}

template <typename T>
ConceptMoI auto MoIThinWalledCylinder()
{
    return MoIDiag<T>{}; // todo
}

template <typename T>
ConceptMoI auto MoIThickWalledCylinder(const T RInner, //
                                       const T ROuter,
                                       const T height,
                                       const T mass)
{
    const T inner2 = RInner * RInner;
    const T outer2 = ROuter * ROuter;
    const T height2 = height * height;

    const T Iz = .5 * mass * (inner2 + outer2);
    const T Ix_y = (1. / 12.) * mass * ((3. * (inner2 + outer2)) + (4. * height2));

    return MoIDiag<T>{Ix_y, Ix_y, Iz}; //
}

template <typename T>
ConceptMoI auto MoIEllipsoid(const T a, //
                             const T b,
                             const T c,
                             const T mass)
{
    // a, b and c are the radii from the center in the x, y, z directions respectively
    const T a2 = a * a;
    const T b2 = b * b;
    const T c2 = c * c;
    return MoIDiag<T>{.2 * mass * (b2 + c2), //
                      .2 * mass * (a2 + c2),
                      .2 * mass * (a2 + b2)};
}

template <typename T>
ConceptMoI auto ParallelAxisTheorem(const T mass, //
                                    const ConceptMoI auto& moi,
                                    const linalg::Vector3<T>& R)
{
    const auto inner = linalg::ElementWiseMultiply(R, R);
    const auto diag = linalg::Diagonal3<T>{inner[0], inner[1], inner[2]};
    return linalg::Add(moi, Matmul(mass, diag));
}

template <typename T>
std::tuple<T, CG<T>, MoIDiag<T>> MoICombine(const T mass1,
                                            const CG<T>& cg1, //
                                            const ConceptMoI auto& moi1,
                                            const T mass2,
                                            const CG<T>& cg2,
                                            const ConceptMoI auto& moi2)
{
    using namespace vic::linalg;

    const T mass = mass1 + mass2;
    const CG<T> cg = Matmul(Add(Matmul(mass1, cg1), Matmul(mass2, cg2)), 1. / mass);
    const MoI<T> moi;
    return std::tuple{mass, cg, moi};
}

} // namespace vic