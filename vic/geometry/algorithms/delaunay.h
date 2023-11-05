#pragma once

#include "vic/linalg/linalg.h"

#include "vic/utils.h"

#include "vic/geometry/algorithms/algorithms.h"
#include "vic/geometry/algorithms/intersections.h"

#include <cmath>
#include <execution>
#include <tuple>
#include <vector>

namespace vic
{
namespace geom
{

namespace detail
{
template <typename TValue, typename TIndex>
struct TriangleData
{
    std::tuple<TIndex, TIndex, TIndex> tri;
    Sphere<TValue, 2> circumscribedCircle;
    bool isCorrect;
};
} // namespace detail

// modified Graham’s scan algorithm
template <typename T>
std::vector<std::size_t> ConvexHull(const std::vector<vic::linalg::Vector2<T>>& points)
{
    assert(points.size() > 2);
    using namespace vic::linalg;

    // todo: abstract with bbox
    double xmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymin = std::numeric_limits<double>::max();
    double ymax = std::numeric_limits<double>::lowest();

    for(const auto& p : points)
    {
        xmin = std::min(xmin, p.Get(0));
        xmax = std::max(xmax, p.Get(0));
        ymin = std::min(ymin, p.Get(1));
        ymax = std::max(ymax, p.Get(1));
    }

    const Vector2<T> pivotPoint((xmin + xmax) / 2., (ymin + ymax) / 2.);

    struct PointData
    {
        std::size_t idx;
        double angle;
        bool ccw;
    };

    std::vector<PointData> data;
    data.reserve(points.size());
    for(std::size_t i = 0; i < points.size(); ++i)
    {
        const auto delta = Subtract(points[i], pivotPoint);
        data.push_back(PointData(i, //
                                 std::atan2(delta.Get(1), delta.Get(0)),
                                 true));
    }
    data.push_back(data[0]);

    std::sort(std::execution::par_unseq, data.begin(), data.end(), [](const auto& a, const auto& b) { return a.angle < b.angle; });

    // note: experiment with paralellizable algorithm
    while(true)
    {
        const auto foreachData = [&data = std::as_const(data), &points = std::as_const(points)](auto& item) {
            const std::size_t idx = &item - &data[0]; //
            const auto previous = idx == 0 ? data.size() - 1 : idx - 1;
            const auto next = (idx == data.size() - 1) ? 0 : idx + 1;

            const auto& p1 = points.at(data[previous].idx);
            const auto& p2 = points.at(data[idx].idx);
            const auto& p3 = points.at(data[next].idx);

            item.ccw = (IsCCW(p1, p2, p3) > 0.);
        };

        // todo: is execution always multithreaded?
        // if(data.size() > 1000)
        std::for_each(std::execution::par_unseq, data.begin(), data.end(), foreachData);
        //else
        //    std::for_each(std::execution::unseq, data.begin(), data.end(), foreachData);

        const std::size_t preSize = data.size();
        data.erase(std::remove_if(data.begin(),
                                  data.end(),
                                  [](const PointData& data) -> bool {
                                      return !data.ccw; //
                                  }),
                   data.end());

        const std::size_t postSize = data.size();

        if(preSize == postSize)
            break;
    }

    std::vector<std::size_t> result;
    result.reserve(data.size());
    for(const auto& item : data)
        result.push_back(item.idx);
    return result;
}

// find the three vertices that together describe the circumcircle for all other points
template <typename T>
std::tuple<std::size_t, std::size_t, std::size_t> SuperTriangle(const std::vector<vic::linalg::Vector2<T>>& points)
{
    assert(points.size() >= 3);

    // todo: this algorithm can probably be sped up by first calculating which vertices are part of the convex hull

    std::tuple<std::size_t, std::size_t, std::size_t> tri{0, 1, 2};
    Sphere<T, 2> circumCircle = CircumscribedCircle(points[0], points[1], points[2]);

    for(std::size_t i = 3; i < points.size(); ++i)
    {
        const auto& point = points[i];
        if(PointInsideSphere(point, circumCircle))
            continue; // this point is inside the current circle

        const auto [a, b, c] = tri;

        const auto circle1 = CircumscribedCircle(points[a], points[b], points[i]);
        if(PointInsideSphere(points[c], circle1))
        {
            tri = {a, b, i};
            circumCircle = circle1;
            continue;
        }

        const auto circle2 = CircumscribedCircle(points[a], points[c], points[i]);
        if(PointInsideSphere(points[b], circle2))
        {
            tri = {a, c, i};
            circumCircle = circle2;
            continue;
        }

        const auto circle3 = CircumscribedCircle(points[b], points[c], points[i]);
        if(PointInsideSphere(points[a], circle3))
        {
            tri = {b, c, i};
            circumCircle = circle3;
            continue;
        }

        assert(false); // problem in algorithm
    }

    return tri;
}

// example implementation:
// https://github.com/bl4ckb0ne/delaunay-triangulation/blob/master/dt/delaunay.cpp

// Delaunay Triangulation:
// https://en.wikipedia.org/wiki/Delaunay_triangulation

// Delaunay Triangulation:
// https://en.wikipedia.org/wiki/Delaunay_triangulation
template <typename T>
auto Delaunay2d(const std::vector<vic::linalg::Vector2<T>>& points)
{
    using namespace vic::linalg;

    using IndexType = uint32_t;

    using Edge = std::pair<IndexType, IndexType>;
    using Triangle = std::tuple<IndexType, IndexType, IndexType>;

    struct EdgeData
    {
        Edge edge;
        bool isCorrect;
    };

    using TriangleData = detail::TriangleData<T, IndexType>;

    std::vector<IndexType> indices;
    indices.reserve(points.size() + 3);
    for(IndexType i = 0; i < points.size(); ++i)
        indices.push_back(i);

    std::sort(indices.begin(), indices.end(), [&](const auto& a, const auto& b) { return points[a].Get(0) < points[b].Get(0); });

    // placeholders for temp vertices
    const IndexType tmp1 = points.size();
    const IndexType tmp2 = points.size() + 1;
    const IndexType tmp3 = points.size() + 2;
    indices.push_back(tmp1);
    indices.push_back(tmp2);
    indices.push_back(tmp3);

    // todo: find workaround for copying all data, just to add three more items
    const auto allPoints = [&]() {
        std::vector<Vector2<T>> copy;
        copy.reserve(points.size() + 3);
        copy = points;
        // todo: calculate what points are far enough
        copy.push_back(Vector2<T>(-1e6, -1e6));
        copy.push_back(Vector2<T>(1e6, -1e6));
        copy.push_back(Vector2<T>(0, 1e6));
        return copy;
    }();

    const auto constructCircumCircle = [&](const Triangle& tri) {
        const auto& [i, j, k] = tri;
        return CircumscribedCircle(allPoints[i], allPoints[j], allPoints[k]);
    };

    const auto sortTriangles = [](const TriangleData& a, const TriangleData& b) {
        return a.circumscribedCircle.pos.Get(0) + a.circumscribedCircle.rad < b.circumscribedCircle.pos.Get(0) + b.circumscribedCircle.rad;
    };

    std::vector<TriangleData> triangles;
    triangles.reserve(points.size() * 2); // todo: how many do we expect?

    {
        const auto newTri = Triangle(tmp1, tmp2, tmp3);
        triangles.push_back(TriangleData(newTri, constructCircumCircle(newTri)));
    }

    std::vector<EdgeData> edges; // loop data
    edges.reserve(std::ceil(std::sqrt(points.size())));

    for(const auto& vertexIndex : indices)
    {
        const auto& vertex = allPoints[vertexIndex];

        edges.clear();

        // check if new vertex is inside circumscribed circle of any tri
        for(auto& tridata : triangles)
        {
            if(PointInsideSphere(vertex, tridata.circumscribedCircle))
            {
                // todo: add the vertex indices in sorted order, so we can speed up the edge removal below
                tridata.isCorrect = false;
                const auto& [i, j, k] = tridata.tri;
                edges.push_back(EdgeData({i, j}, true));
                edges.push_back(EdgeData({j, k}, true));
                edges.push_back(EdgeData({k, i}, true));
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
        if(edges.size() > 1)
        {
            for(auto it1 = edges.begin(); it1 != std::prev(edges.end()); ++it1)
            {
                const auto& [i1, j1] = it1->edge;
                for(auto it2 = std::next(it1); it2 != edges.end(); ++it2)
                {
                    const auto& [i2, j2] = it2->edge;
                    if(((i1 == i2) && (j1 == j2)) || ((i1 == j2) && (j1 == i2)))
                    {
                        it1->isCorrect = false;
                        it2->isCorrect = false;
                    }
                }
            }
        }
        edges.erase(std::remove_if(edges.begin(),
                                   edges.end(),
                                   [](const EdgeData& data) -> bool {
                                       return !data.isCorrect; //
                                   }),
                    edges.end());

        const auto oldSize = triangles.size();

        // for each remaining edge, create a new tri, together with the point we were checking
        for(const auto& edge : edges)
        {
            const auto& [e1, e2] = edge.edge;
            const auto newTri = Triangle(e1, e2, vertexIndex);
            triangles.push_back(TriangleData(newTri, constructCircumCircle(newTri), true));
        }

        // sort triangles based on circle x+rad, so we don't need to check all tris (vertices are sorted already)
        // std::sort(triangles.begin() + oldSize, triangles.end(), sortTriangles); // sort end part
        // std::sort(trianglesEnd, trianglesNewEnd, sortTriangles); // merge two sorted parts
    }

    // remove all triangles that contain any of the temporary vertices
    triangles.erase(std::remove_if(triangles.begin(),
                                   triangles.end(),
                                   [&](const TriangleData& data) -> bool {
                                       const auto& [v1, v2, v3] = data.tri;
                                       return ((v1 == tmp1 || v1 == tmp2 || v1 == tmp3) || //
                                               (v2 == tmp1 || v2 == tmp2 || v2 == tmp3) || //
                                               (v3 == tmp1 || v3 == tmp2 || v3 == tmp3));
                                   }),
                    triangles.end());

    std::vector<Triangle> tris;
    tris.reserve(triangles.size());
    for(const auto& tri : triangles)
        tris.push_back(tri.tri);
    return tris;
}

} // namespace geom
} // namespace vic
