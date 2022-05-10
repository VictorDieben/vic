#pragma once

#include "vic/linalg/linalg.h"

// This file defines shapes such as triangles, squares, circles, spheres, etc.
namespace vic
{
namespace geom
{

// TODO: use definitions from linalg.h?

template <typename T, std::size_t dims>
using Point = vic::linalg::Matrix<T, dims, 1>;

template <typename T, std::size_t dims>
using Direction = vic::linalg::Matrix<T, dims, 1>;

template <typename T, std::size_t dims>
struct Line // pos + (dir * x)
{
    Point<T, dims> pos{};
    Direction<T, dims> dir{};
};

template <typename T, std::size_t dims>
struct LineSegment
{
    Point<T, dims> p1{}; // position start
    Point<T, dims> p2{}; // position end
};

template <typename T, std::size_t dims>
struct Triangle
{
    Point<T, dims> p1{};
    Point<T, dims> p2{};
    Point<T, dims> p3{};
};

// todo: Circle is just a 2d sphere?
//template <typename T, std::size_t dims>
//struct Circle
//{
//    Point<T, dims> pos{};
//    T rad{};
//};

template <typename T, std::size_t dims>
struct Sphere
{
    Point<T, dims> pos{};
    T rad{};
};

template <typename T>
struct Interval
{
    T min{};
    T max{};
};

// todo: bbox is basically an n-dimensional Interval
template <typename T, std::size_t dims>
struct CubeAxisAligned
{
    std::array<Interval<T>, dims> intervals{};
};

// TODO: does a non-3d cylinder make sense?
template <typename T, std::size_t dims>
struct Cylinder
{
    Point<T, dims> p1{};
    Point<T, dims> p2{};
    T rad{};
};

// list of definitions

using Point2d = Point<double, 2>;
using Point2i = Point<int, 2>;

using Point3d = Point<double, 3>;
using Point3i = Point<int, 3>;

using Direction2d = Direction<double, 2>;
using Direction3d = Direction<double, 3>;

using Line2d = Line<double, 2>;
using Line3d = Line<double, 3>;

// todo: euler spiral (linearly changing curvature curve)

} // namespace geom
} // namespace vic