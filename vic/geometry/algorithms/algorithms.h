#pragma once

#include "vic/geometry/geometry.h"
// define algorithms such as intersections, distances, bboxes, etc.

namespace vic
{

namespace geom
{

// check if the corner p1->p2->p3 is describing a counter clockwise direction
template <typename T>
constexpr T IsCCW(const T& p1x, const T& p1y, const T& p2x, const T& p2y, const T& p3x, const T& p3y)
{
    return (p2x - p1x) * (p3y - p1y) - (p3x - p1x) * (p2y - p1y); //
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

// calculate how many times a closed loop circles a certain point
template <typename T>
constexpr int WindingNumber(const std::vector<Point<T, 2>>& polygon, const Point<T, 2>& point)
{
    return 0;
    //
}

//
} // namespace geom
} // namespace vic