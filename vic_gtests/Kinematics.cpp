
#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/kinematics/algorithms/forward_kinematics.h"
#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/math.h"
#include "vic/kinematics/robot/node.h"
#include "vic/kinematics/robot/robot.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"

#include "vic/linalg/linalg.h"

#include "vic/utils.h"

#include <numbers>
#include <optional>
#include <random>

using namespace vic;
using namespace vic::kinematics;

TEST(Kinematics, EulerAngles)
{
    EXPECT_TRUE(IsEqual(EulerAngles(0., 0., 0.), Identity3<double>{}));

    // Test if EulerAngles treats inputs as rotations about X, Y, Z respectively, in rad.

    // create results for 90 deg rotations over any one axis
    Matrix3<double> eulerRx90 = EulerAngles(std::numbers::pi / 2., 0., 0.);
    Matrix3<double> eulerRym90 = EulerAngles(0., -std::numbers::pi / 2., 0.);
    Matrix3<double> eulerRz90 = EulerAngles(0., 0., std::numbers::pi / 2.);
    // manually defined answers
    Matrix3<double> matRx90({1., 0., 0., 0., 0., -1., 0., 1., 0.});
    Matrix3<double> matRym90({0., 0., -1., 0., 1., 0., 1., 0., 0.});
    Matrix3<double> matRz90({0., -1., 0., 1., 0., 0., 0., 0., 1.});

    EXPECT_TRUE(IsEqual(eulerRx90, matRx90, 1e-10));
    EXPECT_TRUE(IsEqual(eulerRym90, matRym90, 1e-10));
    EXPECT_TRUE(IsEqual(eulerRz90, matRz90, 1e-10));

    // Test if EulerAngles implements rotations in INTRINSIC x-y-z sequence.
    // * Intrinsic means that subsequent rotations are fedined in the new intermediade frames
    // * As opposed to Extrinsic, where all rotations are defined in the original reference frame
    // * INTRINSIC wrt consecutive-current: Rot(X0,theta)->Rot(Y1, phi)->Rot(Z2, gamma) = R(X,theta)R(Y,phi)R(Z,gamma)
    // * EXTRINSIC wrt origional:           Rot(X0,theta)->Rot(Y0, phi)->Rot(Z0, gamma) = R(Z,gamma)R(Y,phi)R(X,theta)

    Matrix3<double> euler = EulerAngles(0., std::numbers::pi / 4., std::numbers::pi / 2.);
    Matrix3<double> matIntrinsic({0., -0.70710678118, 0.70710678118, 1., 0., 0., 0., 0.70710678118, 0.70710678118});
    Matrix3<double> matExtrinsic({0., -1., 0., 0.70710678118, 0., 0.70710678118, -0.70710678118, 0., 0.70710678118});
    EXPECT_TRUE(IsEqual(euler, matIntrinsic, 1e-10));
    EXPECT_FALSE(IsEqual(euler, matExtrinsic, 1e-10));

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-100., 100.);

    // Test constructing some random euler angle rotations
    for(std::size_t i = 0; i < 100; ++i)
    {
        Matrix3<double> euler = EulerAngles(dist(g), dist(g), dist(g));
        Rotation rot{euler};
        auto inv = rot.Inverse();
        EXPECT_TRUE(IsEqual((rot * inv).ToMatrix(), Identity3<double>{}));
    }

    // TODO: for Sequencial 'sxyz', or: rol alpha around x -> pitch beta around (new) y -> yaw gamma around (newest) z,
    // we have to pre-multiply as implemented
    // post multiplication (previous version) would perform the rotations around the origional axis (like in a trackball),
    // which is not Sequential
}

TEST(Kinematics, rotate)
{

    EXPECT_TRUE(IsEqual(Rotation<double>{}.ToMatrix(), Identity3<double>{}));

    constexpr auto matrix = ToFull(Identity3<double>{});
    auto zRotation = vic::linalg::Rotate(zAxis, std::numbers::pi / 2.);

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

TEST(Kinematics, rotation)
{
    const auto rotationMat = Rotation<double>{}.ToMatrix();
    EXPECT_TRUE(IsEqual(rotationMat, Identity3<double>{}));

    Rotation r1{vic::linalg::Rotate(zAxis, std::numbers::pi / 2.)};
    Rotation r1Inverse = r1.Inverse();
    Rotation result = r1 * r1Inverse;
    EXPECT_TRUE(IsEqual(result.ToMatrix(), Identity3<double>{}));
}

TEST(Kinematics, translation)
{
    EXPECT_TRUE(IsEqual(Translation<double>{}.ToMatrix(), ZerosMxN<double, 3, 1>{}));

    auto t = Translation{Vector3<double>{{1., 2., 3}}};
    EXPECT_TRUE(IsEqual(t.ToMatrix(), Vector3<double>{{1., 2., 3.}}));
}

TEST(Kinematics, transformations)
{
    EXPECT_TRUE(IsEqual(Transformation<double>{}.ToMatrix(), Identity4<double>{}));

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-100., 100.);

    for(std::size_t i = 0; i < 100; ++i)
    {
        const Rotation rot{EulerAngles(dist(g), dist(g), dist(g))};
        const Translation tr{Vector3<double>{{dist(g), dist(g), dist(g)}}};

        const Transformation transformation{rot, tr};
        const auto inv = transformation.Inverse();

        EXPECT_TRUE(IsEqual((transformation * inv).ToMatrix(), Identity4<double>{}));
    }
}

TEST(Kinematics, Screw)
{
    Screw<double> screw{1., 2., 3., 4., 5., 6.};
    EXPECT_TRUE(IsEqual(screw.GetAngular(), Vector3<double>{{1, 2, 3}}));
    EXPECT_TRUE(IsEqual(screw.GetLinear(), Vector3<double>{{4, 5, 6}}));

    Screw screw2{Vector3<double>{1., 2., 3.}, Vector3<double>{4., 5., 6.}};
    EXPECT_TRUE(IsEqual(screw2.GetAngular(), Vector3<double>{{1, 2, 3}}));
    EXPECT_TRUE(IsEqual(screw2.GetLinear(), Vector3<double>{{4, 5, 6}}));
}

TEST(Kinematics, ExponentialTransform)
{
    Screw<double> screw(0, 0, 1, 0, 0, 0); // screw rotating around z

    // check zero rotation
    auto transform0 = ExponentialTransform(screw, 0.);
    EXPECT_TRUE(IsEqual(transform0, Identity4<double>{}));

    // check rotating over the same screw for a range of angles
    for(const auto& theta : Linspace<double>(0., 2. * std::numbers::pi, 10))
    {
        auto tr = ExponentialTransform(screw, theta);

        auto answer = To<Matrix4<double>>(Identity4<double>{});
        Assign<0, 0>(answer,
                     Matrix2<double>{{std::cos(theta), //
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
        const Vector3<double> axis = Normalize(Vector3<double>{{dist(g), dist(g), dist(g)}});
        const Vector3<double> dir{{dist(g), dist(g), dist(g)}};
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

TEST(Kinematics, CartesianRobot)
{
    const Screw<double> stx{0, 0, 0, 1, 0, 0}; // pure x translation
    const Screw<double> sty{0, 0, 0, 0, 1, 0}; // pure y translation
    const Screw<double> stz{0, 0, 0, 0, 0, 1}; // pure z translation
    const Screw<double> srx{1, 0, 0, 0, 0, 0}; // pure x rotation
    const Screw<double> sry{0, 1, 0, 0, 0, 0}; // pure y rotation
    const Screw<double> srz{0, 0, 1, 0, 0, 0}; // pure z rotation

    robots::ForwardRobot<double> robot{};

    robot.AddJoint(std::nullopt, {}, stx); // x axis
    robot.AddJoint(0u, {}, sty); // y axis
    robot.AddJoint(1u, {}, stz); // z axis

    robot.Update(); // <-- update iterator with new joints

    const std::vector<double> theta{1., 2., 3.};
    const auto transforms = algorithms::ForwardKinematics2(robot, theta);
    const auto translation = transforms.at(2u).GetTranslation();

    ASSERT_TRUE(IsEqual(translation.ToMatrix(), Vector3<double>{{1., 2., 3.}}));

    robot.AddJoint(2u, {}, srx); // x axis
    robot.AddJoint(3u, {}, sry); // y axis
    robot.AddJoint(4u, {}, srz); // z axis

    robot.Update(); // <-- update iterator with new joints

    const Matrix3<double> euler = EulerAngles(std::numbers::pi / 2., std::numbers::pi / 3., std::numbers::pi / 4.);
    const std::vector<double> theta2{3., 2., 1., std::numbers::pi / 2., std::numbers::pi / 3., std::numbers::pi / 4.};

    const auto transforms2 = algorithms::ForwardKinematics2(robot, theta2);
    const auto rotation = transforms2.at(5u).GetRotation();
    ASSERT_TRUE(IsEqual(rotation.ToMatrix(), euler));
}

TEST(Kinematics, Doublependulum)
{
    // defines a simple double pendulum, balanced on a cart.
    // - cart travels along X-axis (positive to the right)
    // - 1st rotary joint on cart's center, roting around Y-axis (poitive into screen)
    // - 1st boom connected to rotor, alligned to z in neurtal pose (upward)
    // - 2nd rotary joint attached to 1st boom's end, same orientation as 1st joint
    // - 2nd boom fixed to the 2nd rotor
    //
    // given state-space variables of the cart and 2 jounts, what is the position and orientation of the end effector?

    const Screw<double> cart{0., 0., 0., 1, 0, 0};
    const Screw<double> rotor1{0., 1., 0., 0, 0, 0};
    const Screw<double> rotor2{0., 1., 0., 0, 0, 0};
    const Screw<double> endEffector{0., 1., 0., 0, 0, 0};

    const Transformation<double> transform12{Rotation<double>{}, Translation{Vector3<double>{{0., 0., 0.5}}}};
    const Transformation<double> transform23{EulerAngles(0., 0., 0.), Vector3<double>{{0., 0., 0.5}}};
    robots::ForwardRobot<double> robot{};

    robot.AddJoint(std::nullopt, {}, cart);
    robot.AddJoint(0u, {}, rotor1);
    robot.AddJoint(1u, transform12, rotor2);
    robot.AddJoint(2u, transform23, endEffector);

    robot.Update();

    const std::vector<double> theta0{1., -std::numbers::pi / 3., 2. * std::numbers::pi / 3., -std::numbers::pi / 3.};

    const auto transform = algorithms::ForwardKinematics2(robot, theta0);

    auto p = transform.at(3u).GetTranslation();
    auto R = transform.at(3u).GetRotation();

    ASSERT_TRUE(IsEqual(p.ToMatrix(), Vector3<double>{{1., 0., 0.5}}));
}
