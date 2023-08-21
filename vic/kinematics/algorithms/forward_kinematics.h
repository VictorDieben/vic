#pragma once

#include "vic/linalg/linalg.h"

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/kinematics/translation.h"

#include "vic/kinematics/math.h"
#include "vic/kinematics/robot/robot.h"

#include <vector>

namespace vic
{
namespace kinematics
{
namespace algorithms
{

using namespace robots;

template <typename T>
std::vector<Transformation<T>> ForwardKinematics(ForwardRobot<T>& robot, //
                                                 const std::vector<T>& theta)
{
    assert(robot.GetTree().IsContinuous()); // if tree is not continuous, we cannot iterate over it
    // for each node in the robot, calculate the transformation
    const std::size_t nJoints = robot.GetNrJoints();
    std::vector<Transformation<T>> neutralPoses{};
    std::vector<Transformation<T>> exponentials{};
    std::vector<Transformation<T>> result{};
    neutralPoses.resize(nJoints);
    exponentials.resize(nJoints);
    result.resize(nJoints);

    for(auto& joint : robot)
    {
        const auto id = joint.Id();
        const auto parentId = joint.Parent();
        const auto& data = joint.Data();
        if(!data.IsType(NodeType::Joint))
            continue; // ignore frames for now

        const auto transform = data.GetTransformation();
        const auto exponent = ExponentialTransform(data.GetScrew(), theta[id]);
        neutralPoses[id] = neutralPoses[parentId] * transform;
        exponentials[id] = exponentials[parentId] * exponent;
        result[id] = exponentials[id] * neutralPoses[id];
    }

    return result;
}

template <typename T>
std::vector<Transformation<T>> ForwardKinematics2(const ForwardRobot<T>& robot, //
                                                  const std::vector<T>& theta)
{
    assert(robot.GetTree().IsContinuous()); // if tree is not continuous, we cannot iterate over it
    // for each node in the robot, calculate the transformation
    const std::size_t nJoints = robot.GetNrJoints();
    std::vector<Transformation<T>> result{};
    result.resize(nJoints);

    for(auto& joint : robot)
    {
        const auto id = joint.Id();
        const auto parentId = joint.Parent();
        const auto& data = joint.Data();
        if(!data.IsType(NodeType::Joint))
            continue; // ignore frames for now

        const auto transform = data.GetTransformation();
        const auto exponent = ExponentialTransform(data.GetScrew(), theta[id]);
        result[id] = result[parentId] * transform * exponent;
    }

    return result;
}

// Bracket operator extended for twists & wrenches.
// TODO: migrate to linalg/tools.h? and refactor to class?
template <typename T>
Matrix4<T> Bracket6(const std::array<T, 6>& vec)
{
    const Matrix4<T> vec_tilde({0, -vec[2], vec[1], vec[3], vec[2], 0, -vec[0], vec[4], -vec[1], vec[0], 0, vec[5], 0, 0, 0, 0});
    return vec_tilde;
}

// The Adjoint matrix re-expresses:
// Tilde(twist_transformed) = H * Tilde(twist) * H^-1
//                  to:
// twist_transformed = Adjoint(H) * twist
//
template <typename T>
Matrix6<T> Adjoint(const Transformation<T>& transform)
{
    Matrix6<T> adjoint{}; // initialised with zeros
    Rotation R = transform.GetRotation();
    Translation p = transform.GetTranslation().ToMatrix();

    Assign<0, 0>(adjoint, R); // 3x3; top left
    //Assign<0, 3>(adjoint, Mat3());            // 3x3 zeros; top right
    Assign<3, 0>(adjoint, MatMul(Bracket3(p), R)); // 3x3; bottom left
    Assign<3, 3>(adjoint, R); // 3x3; bottom right
    return adjoint;
}

template <typename T>
std::vector<Twist<T>> ForwardKinematicsDot(const ForwardRobot<T>& robot, //
                                           const std::vector<Transformation<T>>& transforms,
                                           const std::vector<T>& thetaDot)
{
    assert(robot.GetTree().IsContinuous()); // if tree is not continuous, we cannot iterate over it
    // TODO
    const std::size_t nJoints = robot.GetNrJoints();
    std::vector<Transformation<T>> exponentials{};
    std::vector<Twist<T>> twists{};
    exponentials.resize(nJoints);
    twists.resize(nJoints);

    for(auto& joint : robot)
    {
        const auto id = joint.Id();
        const auto parentId = joint.Parent();
        const auto& data = joint.Data();
        if(!data.IsType(NodeType::Joint))
            continue; // ignore frames for now

        // TODO: implement derivation of Twist calculations
        //const auto exponent = ExponentialTransform(data.GetScrew(), thetaDot[id]);
        //exponentials[id] = exponentials[parentId] * exponent;
        //twists[id] = twists[parentId] * transforms[id] * exponentials[id];
    }

    return twists;
}

// todo: move to separate file
template <typename T>
std::vector<T> InverseDynamics(const ForwardRobot<T>& robot, //
                               const std::vector<T>& theta, //
                               const std::vector<T>& thetaDot, //
                               const std::vector<T>& thetaDotDot)
{
    // calculate the force on each of the joints, based on theta/derivatives,
    // together with gravity and other forces
    return {};
}

} // namespace algorithms
} // namespace kinematics
} // namespace vic