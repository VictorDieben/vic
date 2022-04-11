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
        if(joint.IsRoot())
            continue;

        neutralPoses[id] = neutralPoses[parentId] * data.GetTransformation();
        exponentials[id] = exponentials[parentId] * ExponentialTransform(data.GetScrew(), theta[id]);
        result[id] = exponentials[id] * neutralPoses[id];
    }

    return result;
}

// todo: move to separate file
std::vector<DataType> InverseDynamics(const ForwardRobot& robot, //
                                      const std::vector<DataType>& theta, //
                                      const std::vector<DataType>& thetaDot, //
                                      const std::vector<DataType>& thetaDotDot)
{
    // calculate the force on each of the joints, based on theta/derivatives,
    // together with gravity and other forces
    return {}; //
}

} // namespace algorithms
} // namespace kinematics
} // namespace vic