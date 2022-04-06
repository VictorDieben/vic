#pragma once

#include "vic/linalg/linalg.h"

namespace vic
{
namespace kinematics
{

using namespace vic::linalg;

using DataType = double;

// using Rotation = Matrix<DataType, 3, 3>;
// using Transformation = Matrix<DataType, 4, 4>;

using Screw = Matrix<DataType, 6, 1>;
using Twist = Matrix<DataType, 6, 1>;
using Wrench = Matrix<DataType, 6, 1>;

constexpr Vector3<DataType> xAxis{{1, 0, 0}};
constexpr Vector3<DataType> yAxis{{0, 1, 0}};
constexpr Vector3<DataType> zAxis{{0, 0, 1}};

constexpr double pi = 3.14159265358979323846;

} // namespace kinematics
} // namespace vic