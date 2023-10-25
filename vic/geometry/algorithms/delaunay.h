#pragma once

#include "vic/linalg/linalg.h"

namespace vic
{
namespace geom
{

// Delaunay Triangulation:
// https://en.wikipedia.org/wiki/Delaunay_triangulation
template <typename T>
auto Delaunay2d(std::vector<vic::linalg::Vector<T, 2>>& points)
{
    return 1;
}
} // namespace geom
} // namespace vic
