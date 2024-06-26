#pragma once

#include "vic/geometry/algorithms/algorithms.h"

#include "vic/geometry/geometry.h"
#include "vic/geometry/parametric.h"
#include "vic/utils.h"

namespace vic::geom
{

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

template <typename T>
struct TriLineIntersectionResult
{
    T u{};
    T v{};
    T t{};
    bool Hits() const { return 0 <= u && 0. <= v && (u + v) < 1.; }
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

template <typename T>
struct SphereLineIntersectionResult
{
    // todo: we could store u1 and u2 as an interval, and consider it a hit if u1 < u2
    uint8_t nIntersections{};
    T u1{};
    T u2{};
    bool Hits() const { return nIntersections; }
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
    bool Hits() const { return interval.min <= interval.max; }
};

template <typename T>
TriTriIntersectionResult<T> TriTriIntersection(const Triangle<T, 3>& tri1, const Triangle<T, 3>& tri2)
{
    return TriTriIntersection(tri1.points[0], //
                              tri1.points[1],
                              tri1.points[2],
                              tri2.points[0],
                              tri2.points[1],
                              tri2.points[2]);
}

template <typename T>
Point<T, 3> Projection(const Point<T, 3>& a, const Point<T, 3>& b)
{
    const auto a_dot_b = Dot(a, b);
    const auto b_dot_b = Dot(b, b);
    return Matmul(a_dot_b / b_dot_b, b);
}

template <typename T>
std::pair<Point<T, 3>, Point<T, 3>> ProjectionRejection(const Point<T, 3>& a, const Point<T, 3>& b)
{
    const auto projection = Projection(a, b);
    const auto rejection = Subtract(a, projection);
    return std::pair(projection, rejection);
}

template <typename T>
TriTriIntersectionResult<T> TriTriIntersection(const Point<T, 3>& tri1_1, //
                                               const Point<T, 3>& tri1_2,
                                               const Point<T, 3>& tri1_3,
                                               const Point<T, 3>& tri2_1, //
                                               const Point<T, 3>& tri2_2,
                                               const Point<T, 3>& tri2_3)
{
    const Point<T, 3> e11 = Subtract(tri1_2, tri1_1);
    const Point<T, 3> e12 = Subtract(tri1_3, tri1_1);

    const Point<T, 3> e21 = Subtract(tri2_2, tri2_1);
    const Point<T, 3> e22 = Subtract(tri2_3, tri2_1);

    const auto n1 = Cross(e11, e12);
    const auto n2 = Cross(e21, e22);

    const auto sign11 = Sign(Dot(Subtract(tri1_1, tri2_1), n2));
    const auto sign12 = Sign(Dot(Subtract(tri1_2, tri2_1), n2));
    const auto sign13 = Sign(Dot(Subtract(tri1_3, tri2_1), n2));

    if(sign11 == sign12 && sign12 == sign13)
        return {}; // all points of tri1 are on one side of tri2

    const auto sign21 = Sign(Dot(Subtract(tri2_1, tri1_1), n1));
    const auto sign22 = Sign(Dot(Subtract(tri2_2, tri1_1), n1));
    const auto sign23 = Sign(Dot(Subtract(tri2_3, tri1_1), n1));

    if(sign21 == sign22 && sign22 == sign23)
        return {}; // all points of tri2 are on one side of tri1

    // At this point, we know the triangles could be intersecting,
    // so we have to perform the projections
    const auto intersectionDir = Cross(n1, n2);

    // calculate some point on the line, by taking one of the points as an imaginary extra plane
    const auto intersectionPos = PlanePlanePlaneIntersection(tri1_1, n1, tri2_1, n2, tri1_1, intersectionDir);

    Interval<T> interval1;
    Interval<T> interval2;
    if(sign12 == sign13)
        interval1 = GetTriIntersectionInterval(tri1_1, tri1_2, tri1_3, intersectionPos, intersectionDir);
    else if(sign11 == sign12)
        interval1 = GetTriIntersectionInterval(tri1_3, tri1_2, tri1_1, intersectionPos, intersectionDir);
    else
        interval1 = GetTriIntersectionInterval(tri1_2, tri1_1, tri1_3, intersectionPos, intersectionDir);

    if(sign22 == sign23)
        interval2 = GetTriIntersectionInterval(tri2_1, tri2_2, tri2_3, intersectionPos, intersectionDir);
    else if(sign21 == sign22)
        interval2 = GetTriIntersectionInterval(tri2_3, tri2_2, tri2_1, intersectionPos, intersectionDir);
    else
        interval2 = GetTriIntersectionInterval(tri2_2, tri2_1, tri2_3, intersectionPos, intersectionDir);

    // calculate the overlap of the two intersections (if any)
    return TriTriIntersectionResult{intersectionPos, intersectionDir, Overlap(interval1, interval2)};
};

//
//
//

template <typename T>
Interval<T> IntervalIntersection(const Interval<T>& interval, const T origin, const T direction)
{
    const T v1 = (interval.min - origin) / direction;
    const T v2 = (interval.max - origin) / direction;

    // todo: this is incredibly unlikely to occur, but we do need to check for it.
    // is there a way to perform the division such that nan can never occur?
    if(std::isnan(v1) || std::isnan(v2))
        return Interval<T>{-std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity()};

    return Interval<T>{std::min({v1, v2}), std::max({v1, v2})};
}

template <typename T>
struct AABBLineIntersectionResult
{
    Interval<T> interval{};
    bool Hits() const { return interval.min <= interval.max; }
};

template <typename T>
AABBLineIntersectionResult<T> AABBLineIntersection(const AABB<T, 3>& aabb, //
                                                   const Line<T, 3>& line)
{
    // for each dimension separately, calculate the interval of the intersection.
    // The 3d interval is equal to the overlap of all dimensions combined
    const Interval<T> x = IntervalIntersection(aabb.intervals[0], line.pos.Get(0), line.dir.Get(0));
    const Interval<T> y = IntervalIntersection(aabb.intervals[1], line.pos.Get(1), line.dir.Get(1));
    const Interval<T> z = IntervalIntersection(aabb.intervals[2], line.pos.Get(2), line.dir.Get(2));
    return AABBLineIntersectionResult<T>{Interval<T>{std::max({x.min, y.min, z.min}), //
                                                     std::min({x.max, y.max, z.max})}};
}

template <typename T>
auto ParametricPatchLineIntersection(const BSurface<T, 3>& surface, //
                                     const Line<T, 3>& line)
{
    // paper on the subject:
    // https://dl.acm.org/doi/pdf/10.1145/800064.801287
    return false; //
}

template <typename T>
Sphere<T, 2> CircumscribedCircle(const Point<T, 2>& v1, //
                                 const Point<T, 2>& v2,
                                 const Point<T, 2>& v3)
{
    using namespace vic::linalg;
    const T& ax = v1.Get(0);
    const T& ay = v1.Get(1);

    const T& bx = v2.Get(0);
    const T& by = v2.Get(1);

    const T& cx = v3.Get(0);
    const T& cy = v3.Get(1);

    const T oneOverD = 1. / (2. * ((ax * (by - cy)) + (bx * (cy - ay)) + (cx * (ay - by))));

    const T asq = (ax * ax) + (ay * ay);
    const T bsq = (bx * bx) + (by * by);
    const T csq = (cx * cx) + (cy * cy);

    const T ux = oneOverD * ((asq * (by - cy)) + (bsq * (cy - ay)) + (csq * (ay - by)));
    const T uy = oneOverD * ((asq * (cx - bx)) + (bsq * (ax - cx)) + (csq * (bx - ax)));

    const auto u = Point<T, 2>{ux, uy};
    const T r = Norm(Subtract(u, v1));

    return Sphere<T, 2>{u, r};
}

template <typename T, std::size_t dims>
bool PointInsideSphere(const Point<T, dims>& point, const Sphere<T, dims>& sphere)
{
    using namespace vic::linalg;
    // return SquaredNorm(Subtract(sphere.pos, point)) < (sphere.rad * sphere.rad);
    return Norm(Subtract(sphere.pos, point)) < sphere.rad;
}

template <typename T>
struct PlanePlaneIntersectionResult
{
    Point<T, 3> origin;
    Point<T, 3> direction;
};

template <typename T>
PlanePlaneIntersectionResult<T> PlanePlaneIntersection(const Point<T, 3>& p1, //
                                                       const Point<T, 3>& n1,
                                                       const Point<T, 3>& p2,
                                                       const Point<T, 3>& n2)
{
    PlanePlaneIntersectionResult<T> result{};

    result.direction = Cross(n1, n2);
    // result.origin = TODO: find a solution that does not depend on setting one dimension to a fixed value, for numerical accuracy reasons

    return result;
}

template <typename T>
Point<T, 3> PlanePlanePlaneIntersection(const Point<T, 3>& p1, //
                                        const Point<T, 3>& n1,
                                        const Point<T, 3>& p2,
                                        const Point<T, 3>& n2,
                                        const Point<T, 3>& p3,
                                        const Point<T, 3>& n3)
{
    const auto t1 = Matmul(Dot(p1, n1), Cross(n2, n3));
    const auto t2 = Matmul(Dot(p2, n2), Cross(n3, n1));
    const auto t3 = Matmul(Dot(p3, n3), Cross(n1, n2));

    const T n1n2n3 = Matrix3<T>(t1.Get(0), //
                                t2.Get(0),
                                t3.Get(0),
                                t1.Get(1),
                                t2.Get(1),
                                t3.Get(1),
                                t1.Get(2),
                                t2.Get(2),
                                t3.Get(2));

    const auto invDet = (T)1. / Determinant3x3(n1n2n3);

    return Matmul(invDet, Add(t1, t2, t3));
}

template <typename T>
Plane<T, 3> PointNormalToPlane(const Point<T, 3>& p, //
                               const Point<T, 3>& n)
{
    return {};
}

template <typename T>
Plane<T, 3> PlaneThroughPoints(const Point<T, 3>& p1, //
                               const Point<T, 3>& p2,
                               const Point<T, 3>& p3)
{
    //
}

} // namespace vic::geom