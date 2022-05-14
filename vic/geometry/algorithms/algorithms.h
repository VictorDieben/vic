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
    return TriLineIntersection(tri.points[0], tri.points[1], tri.points[2], segment.p1, dir);
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
    // todo: make branchless
    return SphereLineIntersectionResult<T>{2, u1, u2};
}

template <typename T>
constexpr SphereLineIntersectionResult<T> SphereLineIntersection(const Sphere<T, 3>& sphere, //
                                                                 const LineSegment<T, 3>& segment)
{
    // todo: with a line segment, we need to check if the intersections are on the interval of the segment,
    // e.g. if u1&u2 are within [0; 1]
    return SphereLineIntersection<T>(sphere, segment.p1, Subtract(segment.p2, segment.p1));
}

template <typename T>
constexpr SphereLineIntersectionResult<T> SphereLineIntersection(const Sphere<T, 3>& sphere, //
                                                                 const Line<T, 3>& line)
{
    return SphereLineIntersection<T>(sphere, line.p1, line.p2);
}

// find out which of the points of tri is uniquely on one side of dir.
// returns nullopt if all points are on the same side
template <typename T>
std::optional<std::size_t> GetUniqueSide(const Triangle<T, 3>& tri, //
                                         const Point<T, 3>& pos, //
                                         const Point<T, 3>& dir)
{
    const auto sign1 = Sign(Dot(Subtract(tri.p1, pos), dir));
    const auto sign2 = Sign(Dot(Subtract(tri.p2, pos), dir));
    const auto sign3 = Sign(Dot(Subtract(tri.p3, pos), dir));

    // todo: can this be made branchless?
    // I assume there is some multiplication that can give the right answer
    if(sign1 == sign2)
    {
        if(sign1 != sign3)
            return 2u; // third point
        else
            return std::nullopt;
    }
    else if(sign1 == sign3)
    {
        return 1u; // second point
    }
    else
    {
        return 0u;
    }
}

template <typename T>
constexpr Interval<T> GetTriIntersectionInterval(const Point<T, 3>& v1, //
                                                 const Point<T, 3>& v2,
                                                 const Point<T, 3>& v3,
                                                 const Point<T, 3>& pos,
                                                 const Point<T, 3>& dir)
{
    // project vertices onto line
    const T t1 = Project(pos, dir, v1);
    const T t2 = Project(pos, dir, v2);
    const T t3 = Project(pos, dir, v3);

    // calculate how far each vertex is from nearest point on line
    const T p1 = Norm(Subtract(v1, Add(pos, Matmul(dir, t1))));
    const T p2 = Norm(Subtract(v2, Add(pos, Matmul(dir, t2))));
    const T p3 = Norm(Subtract(v3, Add(pos, Matmul(dir, t3))));

    const T i12 = t1 + ((t2 - t1) * (-p1) / (p2 - p1));
    const T i13 = t1 + ((t3 - t1) * (-p1) / (p3 - p1));

    // todo: choose impl
    return Interval<T>{Min(i12, i13), Max(i12, i13)};
    //if(i12 > i13)
    //    std::swap(i12, i13);
    //return Interval<T>{i12, i13};
}

template <typename T>
struct TriTriIntersectionResult
{
    Point<T, 3> pos{};
    Point<T, 3> dir{};
    Interval<T> interval{};
};

template <typename T>
TriTriIntersectionResult<T> TriTriIntersection(const Triangle<T, 3>& tri1, const Triangle<T, 3>& tri2)
{
    const Point<T, 3> e11 = Subtract(tri1.points[1], tri1.points[0]);
    const Point<T, 3> e12 = Subtract(tri1.points[2], tri1.points[0]);

    const Point<T, 3> e21 = Subtract(tri2.points[1], tri2.points[0]);
    const Point<T, 3> e22 = Subtract(tri2.points[2], tri2.points[0]);

    const auto n1 = Cross(e11, e12);
    const auto n2 = Cross(e21, e22);

    const auto sign11 = Sign(Dot(Subtract(tri1.points[0], tri2.points[0]), n2));
    const auto sign12 = Sign(Dot(Subtract(tri1.points[1], tri2.points[0]), n2));
    const auto sign13 = Sign(Dot(Subtract(tri1.points[2], tri2.points[0]), n2));

    if(sign11 == sign12 && sign12 && sign13)
        return {}; // all points of tri1 are on one side of tri2

    const auto sign21 = Sign(Dot(Subtract(tri2.points[0], tri1.points[0]), n1));
    const auto sign22 = Sign(Dot(Subtract(tri2.points[1], tri1.points[0]), n1));
    const auto sign23 = Sign(Dot(Subtract(tri2.points[2], tri1.points[0]), n1));

    if(sign21 == sign22 && sign22 && sign23)
        return {}; // all points of tri2 are on one side of tri1

    // At this point, we know the triangles could be intersecting,
    // so we have to perform the projections
    const auto intersectionDir = Cross(n1, n2);

    // make indices offset, such that the uniquely-sided point is on index 0
    // todo: branchless?
    const std::size_t offset1 = (sign12 == sign13) ? 0u : //
                                    (sign11 == sign12) ? 2u
                                                       : 1u;
    const std::size_t offset2 = (sign22 == sign23) ? 0u : //
                                    (sign21 == sign22) ? 2u
                                                       : 1u;

    // calculate the position/direction of the intersection
    const auto intersectionPos = Point<T, 3>{};

    // find the interval of the intersection of tri1
    const auto interval1 = GetTriIntersectionInterval(tri1.points[(0 + offset1) % 3], //
                                                      tri1.points[(1 + offset1) % 3],
                                                      tri1.points[(2 + offset1) % 3],
                                                      intersectionPos,
                                                      intersectionDir);

    // find the interval of the intersection of tri2
    const auto interval2 = GetTriIntersectionInterval(tri2.points[(0 + offset2) % 3], //
                                                      tri2.points[(1 + offset2) % 3],
                                                      tri2.points[(2 + offset2) % 3],
                                                      intersectionPos,
                                                      intersectionDir);

    // calculate the overlap of the two intersections (if any)
    return TriTriIntersectionResult{intersectionPos, intersectionDir, Overlap(interval1, interval2)};
};

} // namespace geom
} // namespace vic