#pragma once

#include "vic/linalg/linalg.h"

namespace vic
{
namespace kinematics
{

using namespace vic::linalg;
using namespace vic::kinematics;

using DataType = double;

using Rotation = Matrix<DataType, 3, 3>;
using Transformation = Matrix<DataType, 4, 4>;

using Screw = Matrix<DataType, 6, 1>;
using Twist = Matrix<DataType, 6, 1>;
using Wrench = Matrix<DataType, 6, 1>;




}
}