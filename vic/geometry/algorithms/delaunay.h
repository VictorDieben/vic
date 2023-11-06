#pragma once

#include "vic/linalg/linalg.h"

#include "vic/utils.h"
#include "vic/utils/algorithms.h"

#include "vic/geometry/algorithms/algorithms.h"
#include "vic/geometry/algorithms/intersections.h"

#include <array>
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
    std::array<TIndex, 3> tri;
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

    using Edge = std::array<IndexType, 2>;
    using Triangle = std::array<IndexType, 3>;

    using TriangleData = detail::TriangleData<T, IndexType>;

    std::vector<IndexType> indices;
    indices.reserve(points.size() + 3);
    for(IndexType i = 0; i < points.size(); ++i)
        indices.push_back(i);

    // pre-sort in x direction, reduces swaps/deletes
    std::sort(indices.begin(), indices.end(), [&](const auto& a, const auto& b) { return points[a].Get(0) < points[b].Get(0); });

    // placeholders for temp vertices
    const IndexType tmp1 = points.size();
    const IndexType tmp2 = points.size() + 1;
    const IndexType tmp3 = points.size() + 2;

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

    //const auto constructCircumCircle = [&](const Triangle& tri) {
    //    const auto& [i, j, k] = tri;
    //    return CircumscribedCircle(allPoints[i], allPoints[j], allPoints[k]);
    //};

    const auto sortTriangles = [](const TriangleData& a, const TriangleData& b) {
        return a.circumscribedCircle.pos.Get(0) + a.circumscribedCircle.rad < b.circumscribedCircle.pos.Get(0) + b.circumscribedCircle.rad;
    };

    const auto sameEdgeLambda = [](const Edge& first, const Edge& second) -> bool {
        return first[0] == second[0] && first[1] == second[1]; //
    };

    const auto sortEdgeLambda = [](const Edge& first, const Edge& second) {
        if(first[0] == second[0])
            return first[1] < second[1];
        return first[0] < second[0];
    };

    std::vector<TriangleData> triangles;
    triangles.reserve(points.size() * 2); // todo: how many do we expect?

    triangles.push_back(TriangleData(Triangle({tmp1, tmp2, tmp3}), CircumscribedCircle(allPoints[tmp1], allPoints[tmp2], allPoints[tmp3]), true));

    std::vector<Edge> edges; // loop data
    edges.reserve(std::ceil(std::sqrt(points.size())));

    for(const auto& vertexIndex : indices)
    {
        const auto& vertex = allPoints[vertexIndex];
        edges.clear();

        const auto itFirstLarger = std::lower_bound(triangles.begin(), triangles.end(), vertex.Get(0), [](const TriangleData& item, const T xCoord) {
            return item.circumscribedCircle.pos.Get(0) + item.circumscribedCircle.rad < xCoord; //
        });

        // check if new vertex is inside circumscribed circle of any tri
        std::for_each(itFirstLarger, triangles.end(), [&](TriangleData& tridata) {
            if(PointInsideSphere(vertex, tridata.circumscribedCircle))
            {
                tridata.isCorrect = false;
                const auto& [i, j, k] = tridata.tri;
                edges.push_back(Edge{i, j}); // note: add edge indices in sorted order
                edges.push_back(Edge{i, k});
                edges.push_back(Edge{j, k});
            }
        });

        // remove bad triangles
        triangles.erase(std::remove_if(triangles.begin(),
                                       triangles.end(),
                                       [](const TriangleData& tri) -> bool {
                                           return !tri.isCorrect; //
                                       }),
                        triangles.end());

        std::sort(edges.begin(), edges.end(), sortEdgeLambda);
        edges.erase(vic::remove_duplicates(edges.begin(), edges.end(), sameEdgeLambda), edges.end());

        const auto oldSize = triangles.size();

        // for each remaining edge, create a new tri, together with the point we were checking
        for(const auto& [e1, e2] : edges)
        {
            auto newTri = Triangle{e1, e2, vertexIndex};
            std::sort(newTri.begin(), newTri.end());
            triangles.push_back(TriangleData(newTri, CircumscribedCircle(allPoints[e1], allPoints[e2], allPoints[vertexIndex]), true));
        }
        std::sort(triangles.begin() + oldSize, triangles.end(), sortTriangles); // pre-sort newly added tris
        std::inplace_merge(triangles.begin(), triangles.begin() + oldSize, triangles.end(), sortTriangles); // merge the two sorted ranges
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
