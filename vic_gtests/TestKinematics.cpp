#include "pch.h"

#include "vic/kinematics/algorithms/forward_kinematics.h"
#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/math.h"
#include "vic/kinematics/robot/node.h"
#include "vic/kinematics/robot/robot.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/linalg/determinant.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/traits.h"
#include "vic/utils.h"

#include <optional>
#include <random>

namespace vic
{
namespace kinematics
{

using namespace vic::linalg;

TEST(TestKinematics, EulerAngles)
{
    EXPECT_TRUE(IsEqual(EulerAngles(0., 0., 0.), Identity<double, 3>{}));

    /* Test if EulerAngles treats inputs as rotations about X, Y, Z respectively, in rad. */

    // create results for 90 deg rotations over any one axis
    Matrix<double, 3, 3> eulerRx90 = EulerAngles(pi / 2, 0., 0.);
    Matrix<double, 3, 3> eulerRym90 = EulerAngles(0., -pi / 2, 0.);
    Matrix<double, 3, 3> eulerRz90 = EulerAngles(0., 0., pi / 2);
    // manually defined answers
    Matrix<double, 3, 3> matRx90({1., 0., 0., 0., 0., -1., 0., 1., 0.});
    Matrix<double, 3, 3> matRym90({0., 0., -1., 0., 1., 0., 1., 0., 0.});
    Matrix<double, 3, 3> matRz90({0., -1., 0., 1., 0., 0., 0., 0., 1.});

    EXPECT_TRUE(IsEqual(eulerRx90, matRx90, 1e-10));
    EXPECT_TRUE(IsEqual(eulerRym90, matRym90, 1e-10));
    EXPECT_TRUE(IsEqual(eulerRz90, matRz90, 1e-10));

    // Test if EulerAngles implements rotations in INTRINSIC x-y-z sequence.
    // Intrinsic means that subsequent rotations are fedined in the new intermediade frames
    // As opposed to Extrinsic, where all rotations are defined in the original reference frame
    // INTRINSIC wrt consecutive-current: Rot(X0,theta)->Rot(Y1, phi)->Rot(Z2, gamma) = R(X,theta)R(Y,phi)R(Z,gamma)
    // EXTRINSIC wrt origional:           Rot(X0,theta)->Rot(Y0, phi)->Rot(Z0, gamma) = R(Z,gamma)R(Y,phi)R(X,theta)
    //
    Matrix<double, 3, 3> euler = EulerAngles(0., pi / 4, pi / 2.);
    Matrix<double, 3, 3> matIntrinsic({0., -0.70710678118, 0.70710678118, 1., 0., 0., 0., 0.70710678118, 0.70710678118});
    Matrix<double, 3, 3> matExtrinsic({0., -1., 0., 0.70710678118, 0., 0.70710678118, -0.70710678118, 0., 0.70710678118});
    EXPECT_TRUE(IsEqual(euler, matIntrinsic, 1e-10));
    EXPECT_FALSE(IsEqual(euler, matExtrinsic, 1e-10));

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

TEST(TestKinematics, Quaternion)
{
    const double eps = 1e-10;

    // check if propeties of mapping and rotation matrix are satisfied
    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-1., 1.); //TODO: include negative values
    for(std::size_t i = 0; i < 100; ++i)
    {
        // generate random valid quaternions:
        const Vector4<double> wxyz = Normalize(Vector4<double>{{dist(g), dist(g), dist(g), dist(g)}});

        //Assert that any Quaternion wxyz results in a valid Rotation R
        // nessesary and sufficient:
        // 1) R^T * R = R * R^T = I (orthogonality)
        // 2) det(R) = +1
        //
        const Matrix<DataType, 3, 3> R = Quaternion(wxyz.Get(0, 0), wxyz.Get(1, 0), wxyz.Get(2, 0), wxyz.Get(3, 0));
        EXPECT_TRUE(IsOrthogonal(R));
        EXPECT_NEAR(Determinant3x3(R), 1., eps);
        // Assert that the mapping is a "representation"
        // nessesary:
        // 1) RotToQuaternion( Quaternion( wxyz ) ) = wxyz | -wxyz, and Quaternion( RotToQuaternion( R ) ) = R
        // 2) a quaternion has Norm 1 (should already be satisfied by generation of wxyz)
        const Vector4<double> wxyz2 = RotToQuaternion(R);
        EXPECT_TRUE(IsEqual(wxyz, wxyz2) | IsEqual(wxyz, Matmul(-1., wxyz2)));
        EXPECT_NEAR(Norm(wxyz2), 1., eps);
    }
    // Generate specific tests for each if - block:
    // todo: re-factor to reduce code duplication
    const Matrix<DataType, 3, 3> R_pos_trace = EulerAngles(pi / 4, 0., 0.); // trace = 1+sqrt(2)
    const Matrix<DataType, 3, 3> R_neg_X_largest = EulerAngles(pi * 3 / 4, 0., 0.); // trace = 1-sqrt(2), R(0,0)=1
    const Matrix<DataType, 3, 3> R_neg_Y_largest = EulerAngles(0., pi * 3 / 4, 0.); // trace = 1-sqrt(2), R(1,1)=1
    const Matrix<DataType, 3, 3> R_neg_Z_largest = EulerAngles(0., 0., pi / 4); // trace = 1-sqrt(2), R(2,2)=1

    const Vector4<double> Q_pos_trace = RotToQuaternion(R_pos_trace);
    const Vector4<double> Q_neg_X_largest = RotToQuaternion(R_neg_X_largest);
    const Vector4<double> Q_neg_Y_largest = RotToQuaternion(R_neg_Y_largest);
    const Vector4<double> Q_neg_Z_largest = RotToQuaternion(R_neg_Z_largest);

    const Matrix<DataType, 3, 3> R_pos_trace2 = Quaternion(Q_pos_trace.Get(0, 0), Q_pos_trace.Get(1, 0), Q_pos_trace.Get(2, 0), Q_pos_trace.Get(3, 0));
    const Matrix<DataType, 3, 3> R_neg_X_largest2 = Quaternion(Q_neg_X_largest.Get(0, 0), Q_neg_X_largest.Get(1, 0), Q_neg_X_largest.Get(2, 0), Q_neg_X_largest.Get(3, 0));
    const Matrix<DataType, 3, 3> R_neg_Y_largest2 = Quaternion(Q_neg_Y_largest.Get(0, 0), Q_neg_Y_largest.Get(1, 0), Q_neg_Y_largest.Get(2, 0), Q_neg_Y_largest.Get(3, 0));
    const Matrix<DataType, 3, 3> R_neg_Z_largest2 = Quaternion(Q_neg_Z_largest.Get(0, 0), Q_neg_Z_largest.Get(1, 0), Q_neg_Z_largest.Get(2, 0), Q_neg_Z_largest.Get(3, 0));

    EXPECT_TRUE(IsEqual(R_pos_trace, R_pos_trace2));
    EXPECT_TRUE(IsEqual(R_neg_X_largest, R_neg_X_largest));
    EXPECT_TRUE(IsEqual(R_neg_Y_largest, R_neg_Y_largest));
    EXPECT_TRUE(IsEqual(R_neg_Z_largest, R_neg_Z_largest));
}

TEST(TestKinematics, Vec6ToRot)
{
    const double eps = 1e-10;
    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-1., 1.);
    for(std::size_t i = 0; i < 100; ++i)
    {
        // generate random valid vector6:
        const Vector6<double> Rep = Vector6<double>{{dist(g), dist(g), dist(g), dist(g), dist(g), dist(g)}};

        //Assert that any Vector6 results in a valid Rotation X
        // nessesary and sufficient:
        // 1) orthogonality
        // 2) det(X) = +1
        //
        const Matrix<double, 3, 3> X = Vec6ToRot(Rep);
        EXPECT_TRUE(IsOrthogonal(X));
        EXPECT_NEAR(Determinant3x3(X), 1., eps);
        // Assert that the mapping is a "representation"
        // nessesary:
        // 1) Vec6ToRot( RotToVec6( X ) ) = X; for any X, and RotToVec6( Vec6ToRot( Rot ) ) = Rot; for any Rot
        //
        const Vector6<double> Rep2 = RotToVec6(X);
        //EXPECT_TRUE(IsEqual(Rep, Rep2)); // multiple vec6 map to the same Rot, because of normalisation
        const Matrix<double, 3, 3> X2 = Vec6ToRot(Rep2);
        EXPECT_TRUE(IsEqual(X, X2));
        EXPECT_TRUE(IsEqual(RotToVec6(X2), Rep2));
    }
}

TEST(TestKinematics, rotate)
{

    EXPECT_TRUE(IsEqual(Rotation{}.ToMatrix(), Identity<double, 3>{}));

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
    EXPECT_TRUE(IsEqual(Rotation{}.ToMatrix(), Identity<double, 3>{}));

    Rotation r1{Rotate(zAxis, pi / 2.)};
    Rotation r1Inverse = r1.Inverse();
    Rotation result = r1 * r1Inverse;
    EXPECT_TRUE(IsEqual(result.ToMatrix(), Identity<double, 3>{}));
}

TEST(TestKinematics, translation)
{
    EXPECT_TRUE(IsEqual(Translation{}.ToMatrix(), Zeros<double, 3, 1>{}));

    auto t = Translation{Vector3<double>{{1., 2., 3}}};
    EXPECT_TRUE(IsEqual(t.ToMatrix(), Vector3<double>{{1., 2., 3.}}));
}

TEST(TestKinematics, transformations)
{
    EXPECT_TRUE(IsEqual(Transformation{}.ToMatrix(), Identity<double, 4>{}));

    // EXPECT_TRUE(IsEqual(Transformation{}.ToMatrix(), Transformation{{}, {}}.ToMatrix() ));

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
    Screw screw{{1, 2, 3, 4, 5, 6}};
    EXPECT_TRUE(IsEqual(screw.GetAngular(), Vector3<DataType>{{1, 2, 3}}));
    EXPECT_TRUE(IsEqual(screw.GetLinear(), Vector3<DataType>{{4, 5, 6}}));

    Screw screw2{Vector3<DataType>{{1, 2, 3}}, Vector3<DataType>{{4, 5, 6}}};
    EXPECT_TRUE(IsEqual(screw2.GetAngular(), Vector3<DataType>{{1, 2, 3}}));
    EXPECT_TRUE(IsEqual(screw2.GetLinear(), Vector3<DataType>{{4, 5, 6}}));
}

TEST(TestKinematics, ExponentialTransform)
{
    Screw screw{{0, 0, 1, 0, 0, 0}}; // screw rotating around z

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
    const Screw stx{{0, 0, 0, 1, 0, 0}}; // pure x translation
    const Screw sty{{0, 0, 0, 0, 1, 0}}; // pure y translation
    const Screw stz{{0, 0, 0, 0, 0, 1}}; // pure z translation
    const Screw srx{{1, 0, 0, 0, 0, 0}}; // pure x rotation
    const Screw sry{{0, 1, 0, 0, 0, 0}}; // pure y rotation
    const Screw srz{{0, 0, 1, 0, 0, 0}}; // pure z rotation

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

    const auto transforms2 = algorithms::ForwardKinematics2(robot, theta2);
    const auto rotation = transforms2.at(5u).GetRotation();
    ASSERT_TRUE(IsEqual(rotation.ToMatrix(), euler));
}

TEST(TestKinematics, Doublependulum)
{
    // defines a simple double pendulum, balanced on a cart.
    //  - cart travels along X-axis (positive to the right)
    //  - 1st rotary joint on cart's center, roting around Y-axis (poitive into screen)
    //  - 1st boom connected to rotor, alligned to z in neurtal pose (upward)
    //  - 2nd rotary joint attached to 1st boom's end, same orientation as 1st joint
    //  - 2nd boom fixed to the 2nd rotor
    //
    //  given state-space variables of the cart and 2 jounts, what is the position and orientation of the end effector?
    //
    const Screw cart{{0., 0., 0., 1, 0, 0}};
    const Screw rotor1{{0., 1., 0., 0, 0, 0}};
    const Screw rotor2{{0., 1., 0., 0, 0, 0}};
    const Screw endEffector{{0., 1., 0., 0, 0, 0}};

    const Transformation transform12{Rotation{}, Translation{Vector3<DataType>{{0., 0., 0.5}}}};
    const Transformation transform23{EulerAngles(0., 0., 0.), Vector3<double>{{0., 0., 0.5}}};
    robots::ForwardRobot robot{};

    robot.AddJoint(std::nullopt, {}, cart);
    robot.AddJoint(0u, {}, rotor1);
    robot.AddJoint(1u, transform12, rotor2);
    robot.AddJoint(2u, transform23, endEffector);

    robot.Update();

    const std::vector<DataType> theta0{1., -pi / 3., 2 * pi / 3., -pi / 3.};

    const auto transform = algorithms::ForwardKinematics2(robot, theta0);

    auto p = transform.at(3u).GetTranslation();
    auto R = transform.at(3u).GetRotation();

    ASSERT_TRUE(IsEqual(p.ToMatrix(), Vector3<DataType>{{1., 0., 0.5}}));
}

} // namespace kinematics
} // namespace vic