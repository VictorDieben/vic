
#include <numbers>
#include <random>

#include "gtest/gtest.h"

#include "vic/math/quaternion.h"

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
    EXPECT_TRUE(IsEqual(mat, Matrix33d{0., 0., 1., 0., 1., 0., -1., 0., 0.}));
}

TEST(Quaternion, Inverse)
{
    // todo
}

TEST(Quaternion, ToRotationMatrix)
{
    const auto xmat = linalg::Matrix33d{1, 0, 0, 0, -1, 0, 0, 0, -1};
    const auto xquat = ToRotationMatrix(rotate_x_180);
    EXPECT_TRUE(IsEqual(xquat, xmat));

    const auto ymat = linalg::Matrix33d{-1, 0, 0, 0, 1, 0, 0, 0, -1};
    const auto yquat = ToRotationMatrix(rotate_y_180);
    EXPECT_TRUE(IsEqual(yquat, ymat, 1e-6));

    const auto zmat = linalg::Matrix33d{-1, 0, 0, 0, -1, 0, 0, 0, 1};
    const auto zquat = ToRotationMatrix(rotate_z_180);
    EXPECT_TRUE(IsEqual(zquat, zmat, 1e-6));
}

TEST(Quaternion, ToQuaternion)
{
    // todo
    const auto xquat = ToQuaternion(linalg::Matrix33d{1, 0, 0, 0, -1, 0, 0, 0, -1});
    EXPECT_TRUE(IsEqual(rotate_x_180, xquat));

    const auto yquat = ToQuaternion(linalg::Matrix33d{-1, 0, 0, 0, 1, 0, 0, 0, -1});
    EXPECT_TRUE(IsEqual(rotate_y_180, yquat));

    const auto zquat = ToQuaternion(linalg::Matrix33d{-1, 0, 0, 0, -1, 0, 0, 0, 1});
    EXPECT_TRUE(IsEqual(rotate_z_180, zquat));
}

TEST(Quaternion, Randomized)
{
    using namespace vic::linalg;

    std::default_random_engine g;
    std::uniform_real_distribution<double> rv(-1., 1.);

    // single quaternions
    for(std::size_t i = 0; i < 100; ++i)
    {
        const auto quat = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});

        const auto inv = Inverse(quat);

        EXPECT_TRUE(IsEqual(inv, Conjugate(quat))); // should return the same value for pure rotation quaternions

        EXPECT_TRUE(IsEqual(IdentityQuaternion, Multiply(quat, inv)));
        EXPECT_TRUE(IsEqual(IdentityQuaternion, Multiply(inv, quat)));

        EXPECT_TRUE(std::abs(Norm(quat) - 1.) < 1e-10);

        // check

        // check to-from rotation matrix
        const auto rotmat = ToRotationMatrix(quat);
        const auto res = ToQuaternion(rotmat);
        //if(!IsEqual(norm, res))
        //    EXPECT_TRUE(false); // todo: multiple quaternions can encode the same matrix
        // EXPECT_TRUE(IsEqual(norm, res)); // << "norm = " << norm << "; res = " << res;

        //const Vector3<double> vec{rv(g), rv(g), rv(g)};
        //const auto matmul = Matmul(rotmat, vec);
        //const auto quadmul = Apply(vec, quat);
        //EXPECT_TRUE(IsEqual(matmul, quadmul));
    }

    // two quaternions
    for(std::size_t i = 0; i < 100; ++i)
    {
        const auto p = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});
        const auto q = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});

        // (p q)* == q* p*
        EXPECT_TRUE(IsEqual(Conjugate(Multiply(p, q)), //
                            Multiply(Conjugate(q), Conjugate(p))));

        const auto m1 = ToRotationMatrix(Multiply(p, q));
        const auto m2 = Matmul(ToRotationMatrix(p), ToRotationMatrix(q));

        // EXPECT_TRUE(IsEqual(m1, m2, 1e-6));
    }

    // three quaternions
    for(std::size_t i = 0; i < 100; ++i)
    {
        const auto a = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});
        const auto b = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});
        const auto c = Normalize(Quaternion<double>{rv(g), rv(g), rv(g), rv(g)});

        // check distributivity: (a*b)*c == a*(b*c)

        const auto ab_c = Multiply(Multiply(a, b), c);
        const auto a_bc = Multiply(a, Multiply(b, c));

        const auto dist = Norm(Add(ab_c, Negation(a_bc)));

        EXPECT_TRUE(dist < 1e-8);
    }
}

TEST(Quaternion, Properties)
{
    // todo: https://www.matec-conferences.org/articles/matecconf/pdf/2019/41/matecconf_cscc2019_01060.pdf
}

TEST(Quaternion, ExponentSmallAngle)
{

    std::default_random_engine g;
    std::uniform_real_distribution<double> rv(-1., 1.);

    for(const auto pow : Powspace(1.e-6, .1, std::numbers::e, 20))
    {
        const double angle = pow;

        auto qtemp = Normalize(Quaternion<double>{0., rv(g), rv(g), rv(g)});
        auto quat = Multiply(qtemp, angle);

        const auto q = Exponent(quat);
        const auto qs = ExponentApprox(quat);

        std::cout << std::format("{:.16e}: \te normal: {:.16e};\te approx: {:.16e};\tdiff: {:.16e}", angle, Norm(q) - 1., Norm(qs) - 1., Norm(Subtract(q, qs))) << std::endl;

        EXPECT_LT(std::abs(Norm(q) - 1.), 1e-8);
        EXPECT_LT(std::abs(Norm(qs) - 1.), 1e-8);

        ASSERT_LT(Norm(Subtract(q, qs)), 1e-6) << "First failing angle: " << angle;
    }

    EXPECT_TRUE(false);
}

TEST(Quaternion, Exponent)
{
    using namespace vic::linalg;

    // full rotation
    const auto twoPi = 2. * std::numbers::pi;
    EXPECT_TRUE(IsEqual(IdentityQuaternion, //
                        Exponent(Quaternion<double>{0., twoPi, 0., 0.})));
    EXPECT_TRUE(IsEqual(IdentityQuaternion, //
                        Exponent(Quaternion<double>{0., 0., twoPi, 0.})));
    EXPECT_TRUE(IsEqual(IdentityQuaternion, //
                        Exponent(Quaternion<double>{0., 0., 0., twoPi})));

    const auto res1 = Exponent(Multiply(Quaternion<double>{0., 0., 0., 1.}, 0.1 / 2.));
    const auto res2 = ExponentApprox(Multiply(Quaternion<double>{0., 0., 0., 1.}, 0.1 / 2.));
    const auto ans = Quaternion<double>{0.998750260394966, 0, 0, 0.0499791692706783};
    EXPECT_TRUE(IsEqual(res1, ans));

    // todo: check other angles, make sure all of them are unit length
    std::default_random_engine g;
    std::uniform_real_distribution<double> rv(-1., 1.);

    auto quat = Normalize(Quaternion<double>{0., rv(g), rv(g), rv(g)});
    quat.w() = rv(g);

    const auto exp = Exponent(quat);
    const auto expInv = Exponent(Conjugate(quat));

    EXPECT_DOUBLE_EQ(Norm(exp), 1.);
    EXPECT_DOUBLE_EQ(Norm(expInv), 1.);

    EXPECT_TRUE(IsEqual(IdentityQuaternion, Multiply(exp, expInv)));
    EXPECT_TRUE(IsEqual(IdentityQuaternion, Multiply(expInv, exp)));

    // make sure Exponent works with small angles
    EXPECT_DOUBLE_EQ(Norm(Exponent(Quaternion<double>{0.0001, 1., 1., 1.})), 1.);
    EXPECT_DOUBLE_EQ(Norm(Exponent(Quaternion<double>{1e-10, 1., 1., 1.})), 1.);
    EXPECT_DOUBLE_EQ(Norm(Exponent(Quaternion<double>{1e-14, 1., 1., 1.})), 1.);

    const auto small1 = Exponent(Quaternion<double>{1e-3, 1., 0., 0.});
    const auto small2 = Exponent(Quaternion<double>{1e-6, 1., 0., 0.});

    // const auto rotated = Apply(Vector3d{0, 1, 0}, small);

    int bla = 1;
}
