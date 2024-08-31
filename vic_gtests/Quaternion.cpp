
#include <random>

#include "gtest/gtest.h"

#include "vic/geometry/quaternion.h"

#include "vic/linalg/algorithms/matmul.h"
#include "vic/linalg/algorithms/transpose.h"
#include "vic/linalg/tools.h"

using namespace vic;
using namespace vic::linalg;

TEST(Quaternion, Setup)
{
    // todo: setup a quaternion in multiple ways, or remove this test if not needed
}

TEST(Quaternion, RotationMatrixToQuaternion)
{
    const Matrix33d mat{0., 0., 1., 0., 1., 0., -1., 0., 0.};

    const auto quat = ToQuaternion(mat);

    const auto invSqrt2 = 1. / std::sqrt(2.);
    EXPECT_TRUE(IsEqual(quat, Quaternion<double>{invSqrt2, 0., invSqrt2, 0.}));
}

TEST(Quaternion, QuaternionToRotationMatrix)
{
    const auto invSqrt2 = 1. / std::sqrt(2.);
    const Quaternion<double> quat{invSqrt2, 0., invSqrt2, 0.};

    const auto mat = ToRotationMatrix(quat);

    // check that mat is orthogonal (mat * mat.T == identity)
    EXPECT_TRUE(IsOrthogonal(mat));
    EXPECT_TRUE(IsSpecialOrthogonal(mat));

    // check that mat has the correct value
    EXPECT_TRUE(IsEqual(mat, //
                        Matrix33d{0., 0., 1., 0., 1., 0., -1., 0., 0.}));
}

TEST(Quaternion, Inverse)
{
    // todo
}

TEST(Quaternion, ToRotationMatrix)
{
    // todo
}

TEST(Quaternion, ToQuaternion)
{
    // todo
}

TEST(Quaternion, Randomized)
{
    std::default_random_engine g;
    std::uniform_real_distribution<double> rv(-1., 1.);

    // single quaternions
    for(std::size_t i = 0; i < 100; ++i)
    {
        const Quaternion<double> quat{rv(g), rv(g), rv(g), rv(g)};
        const auto norm = Normalize(quat);

        const auto inv = Inverse(norm);

        EXPECT_TRUE(IsEqual(inv, Conjugate(norm))); // should return the same value for pure rotation quaternions

        EXPECT_TRUE(IsEqual(IdentityQuaternion, Multiply(norm, inv)));
        EXPECT_TRUE(IsEqual(IdentityQuaternion, Multiply(inv, norm)));

        EXPECT_TRUE(IsEqual(quat, ToQuaternion(ToRotationMatrix(quat)))) << i;
    }

    // two quaternions
    for(std::size_t i = 0; i < 100; ++i)
    {
        const auto p = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});
        const auto q = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});

        // (p q)* == q* p*
        EXPECT_TRUE(IsEqual(Conjugate(Multiply(p, q)), //
                            Multiply(Conjugate(q), Conjugate(p))));

        //const auto m1 = ToRotationMatrix(Multiply(p, q));
        //const auto m2 = Matmul(ToRotationMatrix(p), ToRotationMatrix(q));
        //if(!IsEqual(m1, m2, 1e-6))
        //    EXPECT_TRUE(false);
    }
}