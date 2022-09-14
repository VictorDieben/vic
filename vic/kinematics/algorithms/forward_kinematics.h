#pragma once

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

std::vector<Transformation> ForwardKinematics(ForwardRobot& robot, //
                                              const std::vector<DataType>& theta)
{
    assert(robot.GetTree().IsContinuous()); // if tree is not continuous, we cannot iterate over it
    // for each node in the robot, calculate the transformation
    const std::size_t nJoints = robot.GetNrJoints();
    std::vector<Transformation> neutralPoses{};
    std::vector<Transformation> exponentials{};
    std::vector<Transformation> result{};
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

std::vector<Transformation> ForwardKinematics2(ForwardRobot& robot, //
                                              const std::vector<DataType>& theta)
{
    assert(robot.GetTree().IsContinuous()); // if tree is not continuous, we cannot iterate over it
    // for each node in the robot, calculate the transformation
    const std::size_t nJoints = robot.GetNrJoints();
    std::vector<Transformation> result{};
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

std::vector<Twist> ForwardKinematicsDot(ForwardRobot& robot,
                                        const std::vector<Transformation> transforms,
                                               const std::vector<DataType>& thetaDot)
{
    assert(robot.GetTree().IsContinuous()); // if tree is not continuous, we cannot iterate over it
    // TODO
    const std::size_t nJoints = robot.GetNrJoints();
    std::vector<Transformation> exponentials{};
    std::vector<Twist> twists{};
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
std::vector<DataType> InverseDynamics(const ForwardRobot& robot, //
                                      const std::vector<DataType>& theta, //
                                      const std::vector<DataType>& thetaDot, //
                                      const std::vector<DataType>& thetaDotDot)
{
    // calculate the force on each of the joints, based on theta/derivatives,
    // together with gravity and other forces
    return {};
}

} // namespace algorithms
} // namespace kinematics
} // namespace vic