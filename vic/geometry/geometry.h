#pragma once

#include "vic/linalg/linalg.h"

// This file defines shapes such as triangles, squares, circles, spheres, etc.
namespace vic
{
namespace geom
{

// TODO(vicdie): use definitions from linalg.h?

template <typename T, std::size_t dims>
struct Point
{
    std::array<T, dims> pos;
};

template <typename T, std::size_t dims>
struct Direction
{
    std::array<T, dims> pos;
};

template <typename T, std::size_t dims>
struct Line // pos + (dir * x)
{
    Point<T, dims> pos;
    Direction<T, dims> dir;
};

template <typename T, std::size_t dims>
struct LineSegment
{
    Point<T, dims> p1;
    Point<T, dims> p2;
};

template <typename T, std::size_t dims>
struct Triangle
{
    Point<T, dims> p1{};
    Point<T, dims> p2{};
    Point<T, dims> p3{};
};

// todo(vicdie): Circle is just a 2d sphere?
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

template <typename T, std::size_t dims>
struct CubeAxisAligned
{
    std::array<Interval<T>, dims> min{};
};

} // namespace geom
} // namespace vic