#pragma once

#include "vic/geometry/geometry.h"

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

// check that two line segments intersect each other in 2d
template <typename T>
constexpr bool Intersects(const LineSegment<T, 2>& seg1, const LineSegment<T, 2>& seg2)
{
    if(IsCCW(seg1.p1, seg1.p2, seg2.p1) * IsCCW(seg1.p1, seg1.p2, seg2.p2) > 0)
        return false; // both points of segment 2 on one side of seg1

    if(IsCCW(seg2.p1, seg2.p2, seg1.p1) * IsCCW(seg2.p1, seg2.p2, seg1.p2) > 0)
        return false; // both points of segment 1 on one side of seg2

    return true;
}

template <typename T>
constexpr std::optional<Point<T, 2>> Intersection(const LineSegment<T, 2>& seg1, const LineSegment<T, 2>& seg2)
{
    return std::nullopt; //
}

// calculate how many times a closed loop circles a certain point
template <typename T>
constexpr int WindingNumber(const std::vector<Point<T, 2>>& polygon, const Point<T, 2>& point)
{
    return 0; //
}

//
//
//
//

template <typename T>
struct TriLineIntersectionResult
{
    T u{}, v{}, t{};
};

// Return UV pair, indicating where the intersection is,
// along the [tri1, tri2] segment and [tri1, tri3] segment
template <typename T>
constexpr TriLineIntersectionResult<T> TriLineIntersection(const Point<T, 3>& tri1, //
                                                           const Point<T, 3>& tri2, //
                                                           const Point<T, 3>& tri3, //
                                                           const Point<T, 3>& linePos, //
                                                           const Point<T, 3>& lineDir)
{
    using namespace vic::linalg;

    const Point<T, 3> e1 = Subtract(tri2, tri1);
    const Point<T, 3> e2 = Subtract(tri3, tri1);
    const auto n = Cross(e1, e2);
    const auto dot = -1. * Dot(lineDir, n);
    const auto invDot = 1. / dot;

    const auto diff = Subtract(linePos, tri1);
    const auto dirCrossDiff = Cross(diff, lineDir);

    const T u{Dot(e2, dirCrossDiff) * invDot};
    const T v{-1. * Dot(e1, dirCrossDiff) * invDot};
    const T t{Dot(diff, n) * invDot};

    return TriLineIntersectionResult{u, v, t};
}

template <typename T>
constexpr TriLineIntersectionResult<T> TriLineIntersection(const Triangle<T, 3>& tri, //
                                                           const LineSegment<T, 3>& segment)
{
    const auto neg = Negative(segment.p1);
    const auto dir = Add(segment.p2, neg);
    return TriLineIntersection(tri.p1, tri.p2, tri.p3, segment.p1, dir);
}

template <typename T>
constexpr TriLineIntersectionResult<T> TriLineIntersection(const Triangle<T, 3>& tri, //
                                                           const Line<T, 3>& line)
{
    return TriLineIntersection(tri.p1, tri.p2, tri.p3, line.pos, line.dir);
}

//
//
//
//

template <typename T>
struct SphereLineIntersectionResult
{
    uint8_t nIntersections{};
    T u1{};
    T u2{};
};

// http://portal.ku.edu.tr/~cbasdogan/Courses/Robotics/projects/IntersectionLineSphere.pdf
template <typename T>
constexpr SphereLineIntersectionResult<T> SphereLineIntersection(const Sphere<T, 3>& sphere, //
                                                                 const Point<T, 3>& linePos, //
                                                                 const Point<T, 3>& lineDir)
{
    using namespace vic::linalg;

    const auto& p3 = sphere.pos;
    const auto& p1 = linePos;
    const auto p3p1 = Subtract(linePos, p3); // vec from p3 to p1
    const T& r = sphere.rad;

    const T a = Dot(lineDir, lineDir);
    const T b = 2. * Dot(lineDir, p3p1);
    const T c = Dot(p1, p1) + Dot(p3, p3) - (2. * Dot(p1, p3)) - Pow<2>(r);

    const T det = (b * b) - (4. * a * c);
    if(det < 0.)
        return SphereLineIntersectionResult<T>{0, 0., 0.};

    // todo: remove? not really needed
    //if(det == 0.)
    //{
    //    const T u = -b / (2 * a);
    //    return SphereLineIntersectionResult<T>{1, u, u};
    //}

    const T u1 = (-b - Sqrt(det)) / (2. * a);
    const T u2 = (-b + Sqrt(det)) / (2. * a);

    // todo: det will never be _exactly_ 0. so there will be either 0 or 2 intersections.
    // furthermore, even if we miss the circle, we now have useful information about the closest point to the sphere.
    // maybe we can do nice things with that information.
    return SphereLineIntersectionResult<T>{2, u1, u2};
}

template <typename T>
constexpr SphereLineIntersectionResult<T> SphereLineIntersection(const Sphere<T, 3>& sphere, //
                                                                 const LineSegment<T, 3>& segment)
{
    // todo: with a line segment, we need to check if the intersections are on the interval of the segment
    return SphereLineIntersection<T>(sphere, segment.p1, Subtract(segment.p2, segment.p1));
}

template <typename T>
constexpr SphereLineIntersectionResult<T> SphereLineIntersection(const Sphere<T, 3>& sphere, //
                                                                 const Line<T, 3>& line)
{
    return SphereLineIntersection<T>(sphere, line.p1, line.p2);
}

} // namespace geom
} // namespace vic