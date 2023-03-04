#pragma once

#include <array>
#include <cstddef>

#include "vic/linalg/matrices/matrix.h"

namespace vic
{
namespace geom
{

// bezier curve, 4 control points, cubic polynomial
template <typename T, std::size_t dims>
struct BCurve
{
public:
    using ControlPoint = linalg::VectorN<T, dims>;
    std::array<ControlPoint, 4> mControlPoints{};
};

// bezier curve, 4 control points, cubic polynomial
template <typename T, std::size_t dims>
struct BSurface
{
public:
    using ControlPoint = linalg::VectorN<T, dims>;
    std::array<ControlPoint, 4> mControlPoints{};
};

// Bezier Spline, any number of control points, variable order
template <typename T, std::size_t dims>
struct BSpline
{ };

} // namespace geom
} // namespace vic