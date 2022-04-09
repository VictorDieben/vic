#include "pch.h"

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/math.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/linalg/traits.h"

#include "vic/utils.h"

namespace vic
{
namespace kinematics
{

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto IsEqual(const TMat1& mat1, const TMat2& mat2, const double eps = 1e-10)
{
    if((mat1.GetRows() != mat2.GetRows()) || (mat1.GetColumns() != mat2.GetColumns()))
        return false;
    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat1.GetColumns(); ++j)
            if(std::fabs(mat1.Get(i, j) - mat2.Get(i, j)) > eps)
                return false;
    return true;
}

TEST(TestKinematics, EulerAngles)
{
    // todo: verify if the resulting matrix is as expected

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-100., 100.);

    // Test constructing some random euler angle rotations
    for(std::size_t i = 0; i < 100; ++i)
    {
        Matrix<double, 3, 3> euler = EulerAngles(dist(g), dist(g), dist(g));
        Rotation rot{euler};
        auto inv = rot.Inverse();
        EXPECT_TRUE(IsEqual((rot * inv).ToMatrix(), Identity<double, 3>{}));
    }
}

TEST(TestKinematics, rotate)
{
    constexpr Matrix<double, 3, 3> matrix{Identity<double, 3>{}};
    auto zRotation = Rotate(zAxis, pi / 2.);

    // rotating a vector around the z axis for pi/2
    auto tmp = Matmul(zRotation, Vector3<double>({1, 0, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({0, 1, 0})));

    tmp = Matmul(zRotation, Vector3<double>({0, 1, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({-1, 0, 0})));

    tmp = Matmul(zRotation, Vector3<double>({-1, 0, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({0, -1, 0})));

    tmp = Matmul(zRotation, Vector3<double>({0, -1, 0}));
    EXPECT_TRUE(IsEqual(tmp, Vector3<double>({1, 0, 0})));
}

TEST(TestKinematics, rotation)
{
    Rotation r1{Rotate(zAxis, pi / 2.)};
    Rotation r1Inverse = r1.Inverse();
    Rotation result = r1 * r1Inverse;
    EXPECT_TRUE(IsEqual(result.ToMatrix(), Identity<double, 3>{}));
}

TEST(TestKinematics, transformations)
{
    constexpr Transformation transform{};

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-100., 100.);

    for(std::size_t i = 0; i < 100; ++i)
    {
        const Rotation rot{EulerAngles(dist(g), dist(g), dist(g))};
        const Translation tr{Vector3<DataType>{{dist(g), dist(g), dist(g)}}};

        const Transformation transformation{rot, tr};
        const auto inv = transformation.Inverse();

        EXPECT_TRUE(IsEqual((transformation * inv).ToMatrix(), //
                            Identity<DataType, 4>{}));
    }
}

TEST(TestKinematics, Screw)
{
    Screw screw{Vector6<DataType>{{1, 2, 3, 4, 5, 6}}};
    EXPECT_TRUE(IsEqual(screw.GetAngular(), Vector3<DataType>{{1, 2, 3}}));
    EXPECT_TRUE(IsEqual(screw.GetLinear(), Vector3<DataType>{{4, 5, 6}}));

    Screw screw2{Vector3<DataType>{{1, 2, 3}}, Vector3<DataType>{{4, 5, 6}}};
    EXPECT_TRUE(IsEqual(screw2.GetAngular(), Vector3<DataType>{{1, 2, 3}}));
    EXPECT_TRUE(IsEqual(screw2.GetLinear(), Vector3<DataType>{{4, 5, 6}}));
}

TEST(TestKinematics, ExponentialTransform)
{
    Screw screw{Vector6<DataType>{{0, 0, 1, 0, 0, 0}}}; // screw rotating around z

    // check zero rotation
    auto transform0 = ExponentialTransform(screw, 0.);
    EXPECT_TRUE(IsEqual(transform0, Identity<DataType, 4>{}));

    // check rotating over the same screw for a range of angles
    for(const auto theta : Linspace(0., 2. * pi, 10))
    {
        auto tr = ExponentialTransform(screw, theta);

        auto answer = Matrix<DataType, 4, 4>{Identity<DataType, 4>{}};
        Assign<0, 0>(answer,
                     Matrix<DataType, 2, 2>{{std::cos(theta), //
                                             -std::sin(theta),
                                             std::sin(theta),
                                             std::cos(theta)}});

        EXPECT_TRUE(IsEqual(tr, answer));
    }

    // construct a random screw, make sure that doing the full transform at once equals doing it in steps
    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(0, 1.);
    for(std::size_t i = 0; i < 1; ++i)
    {
        Screw screw{Vector6<DataType>{{dist(g), dist(g), dist(g), dist(g), dist(g), dist(g)}}}; //

        const auto theta1 = dist(g);
        const auto theta2 = dist(g);

        auto transFull = ExponentialTransform(screw, theta1 + theta2);

        auto trans1 = ExponentialTransform(screw, theta1);
        auto trans2 = ExponentialTransform(screw, theta2);
        auto answer = Matmul(trans1, trans2);

        // TODO: solve
        ASSERT_TRUE(IsEqual(transFull, answer));
    }
}

} // namespace kinematics
} // namespace vic