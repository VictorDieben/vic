#pragma once

#include "vic/kinematics/kinematics.h"
#include "vic/kinematics/rotation.h"
#include "vic/kinematics/transformation.h"
#include "vic/kinematics/translation.h"

#include "vic/kinematics/robot/robot.h"

#include <vector>

namespace vic
{
namespace kinematics
{
namespace algorithms
{

using namespace robots;

std::vector<Transformation> ForwardKinematics(const ForwardRobot& robot, //
                                              const std::vector<DataType>& theta)
{
    // for each node in the robot, calculate the transformation
    return {};
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