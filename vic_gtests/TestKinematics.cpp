#include "pch.h"

#include "vic/kinematics/algorithms/forward_kinematics.h"
#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/math.h"
#include "vic/kinematics/robot/node.h"
#include "vic/kinematics/robot/robot.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/linalg/traits.h"

#include "vic/utils.h"

#include <optional>

namespace vic
{
namespace kinematics
{

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

    // TODO: for Sequencial 'sxyz', or: rol alpha around x -> pitch beta around (new) y -> yaw gamma around (newest) z,
    // we have to pre-multiply as implemented
    // post multiplication (previous version) would perform the rotations around the origional axis (like in a trackball),
    // which is not Sequential
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
    for(std::size_t i = 0; i < 100; ++i)
    {
        const Vector3<DataType> axis = Normalize(Vector3<DataType>{{dist(g), dist(g), dist(g)}});
        const Vector3<DataType> dir{{dist(g), dist(g), dist(g)}};
        const Screw screw{axis, dir};

        const auto theta1 = dist(g);
        const auto theta2 = dist(g);

        auto transFull = ExponentialTransform(screw, theta1 + theta2);

        auto trans1 = ExponentialTransform(screw, theta1);
        auto trans2 = ExponentialTransform(screw, theta2);
        auto answer = Matmul(trans2, trans1);

        ASSERT_TRUE(IsEqual(transFull, answer));
    }
}

TEST(TestKinematics, CartesianRobot)
{
    const Screw stx{Vector6<DataType>{{0, 0, 0, 1, 0, 0}}}; // pure x translation
    const Screw sty{Vector6<DataType>{{0, 0, 0, 0, 1, 0}}}; // pure y translation
    const Screw stz{Vector6<DataType>{{0, 0, 0, 0, 0, 1}}}; // pure z translation
    const Screw srx{Vector6<DataType>{{1, 0, 0, 0, 0, 0}}}; // pure x rotation
    const Screw sry{Vector6<DataType>{{0, 1, 0, 0, 0, 0}}}; // pure y rotation
    const Screw srz{Vector6<DataType>{{0, 0, 1, 0, 0, 0}}}; // pure z rotation

    robots::ForwardRobot robot{};

    robot.AddJoint(std::nullopt, {}, stx); // x axis
    robot.AddJoint(0u, {}, sty); // y axis
    robot.AddJoint(1u, {}, stz); // z axis

    robot.Update(); // <-- update iterator with new joints

    const std::vector<DataType> theta{1., 2., 3.};
    const auto transforms = algorithms::ForwardKinematics(robot, theta);
    const auto translation = transforms.at(2u).GetTranslation();

    ASSERT_TRUE(IsEqual(translation.ToMatrix(), Vector3<DataType>{{1., 2., 3.}}));

    robot.AddJoint(2u, {}, srx); // x axis
    robot.AddJoint(3u, {}, sry); // y axis
    robot.AddJoint(4u, {}, srz); // z axis

    robot.Update(); // <-- update iterator with new joints

    const Matrix<double, 3, 3> euler = EulerAngles(pi / 2, pi / 3, pi / 4);
    const std::vector<DataType> theta2{3., 2., 1., pi / 2, pi / 3, pi / 4};

    const auto transforms2 = algorithms::ForwardKinematics(robot, theta2);
    const auto rotation = transforms2.at(5u).GetRotation();
    ASSERT_TRUE(IsEqual(rotation.ToMatrix(), euler));
}

} // namespace kinematics
} // namespace vic