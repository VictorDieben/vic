
#include "gtest/gtest.h"

#include "vic/geometry/quaternion.h"

#include "vic/linalg/algorithms/matmul.h"
#include "vic/linalg/algorithms/transpose.h"
#include "vic/linalg/tools.h"

using namespace vic;
using namespace vic::linalg;

TEST(Quaternion, Setup)
{
    //
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
    //
}