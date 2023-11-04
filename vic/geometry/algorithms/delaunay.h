#pragma once

#include "vic/linalg/linalg.h"

#include "vic/utils.h"

#include "vic/geometry/algorithms/intersections.h"

#include <tuple>
#include <vector>

namespace vic
{
namespace geom
{

namespace detail
{

template <typename T>
struct TriangleData
{
    std::tuple<std::size_t, std::size_t, std::size_t> tri{};
    Sphere<T, 2> circumscribedCircle{};
    bool isCorrect = true;
};
} // namespace detail

// find the three vertices that together describe the circumcircle for all other points
template <typename T>
std::tuple<std::size_t, std::size_t, std::size_t> SuperTriangle(const std::vector<vic::linalg::Vector2<T>>& points)
{
    assert(points.size() >= 3);

    std::tuple<std::size_t, std::size_t, std::size_t> tri{0, 1, 2};
    Sphere<T, 2> circumCircle = CircumscribedCircle(points.at(0), points.at(1), points.at(2));

    for(std::size_t i = 3; i < points.size(); ++i)
    {
        const auto& point = points.at(i);
        if(PointInsideSphere(point, circumCircle))
            continue; // this point is inside the current circle

        const auto [a, b, c] = tri;

        const auto circle1 = CircumscribedCircle(points.at(a), points.at(b), points.at(i));
        if(PointInsideSphere(points.at(c), circle1))
        {
            tri = {a, b, i};
            circumCircle = circle1;
            continue;
        }

        const auto circle2 = CircumscribedCircle(points.at(a), points.at(c), points.at(i));
        if(PointInsideSphere(points.at(b), circle2))
        {
            tri = {a, c, i};
            circumCircle = circle2;
            continue;
        }

        const auto circle3 = CircumscribedCircle(points.at(b), points.at(c), points.at(i));
        if(PointInsideSphere(points.at(a), circle3))
        {
            tri = {b, c, i};
            circumCircle = circle3;
            continue;
        }
    }

    return tri;
}

// example implementation:
// https://github.com/bl4ckb0ne/delaunay-triangulation/blob/master/dt/delaunay.cpp

// Delaunay Triangulation:
// https://en.wikipedia.org/wiki/Delaunay_triangulation
template <typename T>
auto Delaunay2d(const std::vector<vic::linalg::Vector2<T>>& points)
{
    using namespace vic::linalg;

    using Edge = std::pair<std::size_t, std::size_t>;
    using Triangle = std::tuple<std::size_t, std::size_t, std::size_t>;

    struct EdgeData
    {
        Edge edge;
        bool isCorrect = true;
    };

    using TriangleData = detail::TriangleData<T>;

    const auto [s1, s2, s3] = SuperTriangle(points);

    std::vector<std::size_t> indices;
    for(std::size_t i = 0; i < points.size(); ++i)
        if(i != s1 && i != s2 && i != s3)
            indices.push_back(i);

    auto sortedX = indices;
    std::sort(sortedX.begin(), sortedX.end(), [&points](const auto& a, const auto& b) {
        return points.at(a).Get(0) < points.at(b).Get(0); //
    });

    const auto constructCircumCircle = [&](const Triangle& tri) {
        const auto& [i, j, k] = tri;
        return CircumscribedCircle(points.at(i), points.at(j), points.at(k));
    };

    std::vector<TriangleData> triangles; // working data;

    {
        const auto newTri = Triangle(s1, s2, s3);
        triangles.push_back(TriangleData(newTri, constructCircumCircle(newTri)));
    }

    std::vector<EdgeData> edges; // loop data

    for(const auto& vertexIndex : sortedX)
    {
        const auto& vertex = points.at(vertexIndex);

        edges.clear();

        // check if new vertex is inside circumscribed circle of any tri
        for(auto& tridata : triangles)
        {
            if(PointInsideSphere(vertex, tridata.circumscribedCircle))
            {
                tridata.isCorrect = false;
                const auto [i, j, k] = tridata.tri;
                edges.push_back(EdgeData{{i, j}});
                edges.push_back(EdgeData{{j, k}});
                edges.push_back(EdgeData{{k, i}});
            }
        }

        // remove bad triangles
        triangles.erase(std::remove_if(triangles.begin(),
                                       triangles.end(),
                                       [](const TriangleData& data) -> bool {
                                           return !data.isCorrect; //
                                       }),
                        triangles.end());

        // remove duplicate edges
        // todo: can be sped up by pre-sorting the edges
        for(auto it1 = edges.begin(); it1 != edges.end(); ++it1)
        {
            const auto& [i1, j1] = it1->edge;
            for(auto it2 = std::next(it1); it2 != edges.end(); ++it2)
            {
                const auto& [i2, j2] = it2->edge;
                if((i1 == i2 && j1 == j2) || (i1 == j2 && j1 == i2))
                {
                    it1->isCorrect = false;
                    it2->isCorrect = false;
                }
            }
        }
        edges.erase(std::remove_if(edges.begin(),
                                   edges.end(),
                                   [](const EdgeData& data) -> bool {
                                       return !data.isCorrect; //
                                   }),
                    edges.end());

        // for each remaining edge, create a new tri, together with the point we were checking
        for(const auto& edge : edges)
        {
            const auto& [e1, e2] = edge.edge;
            const auto newTri = Triangle(e1, e2, vertexIndex);
            triangles.push_back(TriangleData(newTri, constructCircumCircle(newTri)));
        }
    }

    std::vector<Triangle> tris;
    tris.reserve(triangles.size());
    for(const auto& tri : triangles)
        tris.push_back(tri.tri);
    return tris;
}

} // namespace geom
} // namespace vic
