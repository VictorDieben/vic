#pragma once

#include "vic/geometry/geometry.h"
#include "vic/utils.h"

#include "vic/linalg/add.h"
#include "vic/linalg/matrices.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/transpose.h"

#include <optional>

// define algorithms such as intersections, distances, bboxes, etc.

namespace vic
{

namespace geom
{
// todo: sort into 2d/3d etc?

// check if the corner p1->p2->p3 is describing a counter clockwise direction
template <typename T>
constexpr T IsCCW(const T& p1x, const T& p1y, const T& p2x, const T& p2y, const T& p3x, const T& p3y)
{
    return ((p2x - p1x) * (p3y - p1y)) - ((p3x - p1x) * (p2y - p1y)); //
}

// check if the corner p1->p2->p3 is describing a counter clockwise direction
template <typename T>
constexpr T IsCCW(const Point<T, 2>& p1, //
                  const Point<T, 2>& p2,
                  const Point<T, 2>& p3)
{
    return IsCCW(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]);
}

// calculate how many times a closed loop circles a certain point
template <typename T>
constexpr int WindingNumber(const std::vector<Point<T, 2>>& polygon, const Point<T, 2>& point)
{
    return 0; //
}

template <typename T, std::size_t dims>
T Project(const Point<T, dims>& origin, const Direction<T, dims>& dir, const Point<T, dims>& point)
{
    const auto relative = Subtract(point, origin);
    return Dot(relative, dir) / Dot(dir, dir);
}

template <typename T>
Point<T, 2> ProjectUV(const Triangle<T, 3>& tri, const Point<T, 3>& pos)
{
    const Direction<T, 3> e1 = Subtract(tri.points[1], tri.points[0]);
    const Direction<T, 3> e2 = Subtract(tri.points[2], tri.points[0]);
    return Point<T, 2>{Project(tri.points[0], e1, pos), Project(tri.points[0], e2, pos)};
}

template <typename T>
T TriangleArea(const Point<T, 2>& p1, //
               const Point<T, 2>& p2,
               const Point<T, 2>& p3)
{
    const auto e1 = Subtract(p2, p1);
    const auto e2 = Subtract(p3, p1);
    const auto c = Cross(e1, e2);
    return c / T{2.};
}

template <typename T>
T TriangleArea(const Triangle<T, 2>& tri)
{
    return TriangleArea(tri.points[0], tri.points[1], tri.points[2]);
}

} // namespace geom
} // namespace vic