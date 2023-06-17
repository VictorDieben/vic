#pragma once

#include "vic/geometry/geometry.h"
#include "vic/linalg/linalg.h"

#include <array>
#include <cmath>
#include <numbers> // std::numbers
#include <tuple>
#include <vector>

namespace vic
{
namespace mesh
{

using Vertex = vic::geom::Point<double, 3>;
using MeshIndex = uint32_t;
using Edge = std::pair<MeshIndex, MeshIndex>;
using Tri = std::tuple<MeshIndex, MeshIndex, MeshIndex>;

// todo:
//struct Tri
//{
//    std::array<MeshIndex, 3> vertices;
//
//    MeshIndex First() const { return vertices.at(0); }
//    MeshIndex Second() const { return vertices.at(1); }
//    MeshIndex Third() const { return vertices.at(2); }
//    MeshIndex Get(const std::size_t i) const { return vertices.at(i); }
//};

Vertex ToVertex(const double x, const double y, const double z)
{
    // helper, linalg vecs don't have this constructor
    return Vertex{{x, y, z}};
}

struct EdgeMesh
{
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
};

// todo: this is a tri mesh (each tri points to 3 indices in the vertex vector)
// also make different types of meshes
struct TriMesh
{
    std::vector<Vertex> vertices;
    std::vector<Tri> tris;
};

template <typename T>
TriMesh GenerateCube(const vic::geom::AABB<T, 3>& box)
{
    const auto lx = box.intervals.at(0).min;
    const auto ux = box.intervals.at(0).max;
    const auto ly = box.intervals.at(1).min;
    const auto uy = box.intervals.at(1).max;
    const auto lz = box.intervals.at(2).min;
    const auto uz = box.intervals.at(2).max;

    TriMesh result;
    result.vertices.reserve(8);
    result.tris.reserve(12);

    result.vertices.push_back(Vertex{{lx, ly, lz}}); // 0
    result.vertices.push_back(Vertex{{ux, ly, lz}}); // 1
    result.vertices.push_back(Vertex{{lx, uy, lz}}); // 2
    result.vertices.push_back(Vertex{{ux, uy, lz}}); // 3

    result.vertices.push_back(Vertex{{lx, ly, uz}}); // 4
    result.vertices.push_back(Vertex{{ux, ly, uz}}); // 5
    result.vertices.push_back(Vertex{{lx, uy, uz}}); // 6
    result.vertices.push_back(Vertex{{ux, uy, uz}}); // 7

    result.tris.push_back(Tri{0, 1, 2}); // bottom
    result.tris.push_back(Tri{2, 1, 3});
    result.tris.push_back(Tri{0, 1, 4}); // front
    result.tris.push_back(Tri{4, 1, 5});
    result.tris.push_back(Tri{0, 2, 4}); // left
    result.tris.push_back(Tri{2, 4, 6});
    result.tris.push_back(Tri{4, 5, 6}); // top
    result.tris.push_back(Tri{6, 5, 7});
    result.tris.push_back(Tri{2, 3, 6}); // back
    result.tris.push_back(Tri{6, 3, 7});
    result.tris.push_back(Tri{1, 3, 5}); // right
    result.tris.push_back(Tri{5, 3, 7});

    return result;
}

template <typename T>
TriMesh GenerateCubeSphere(const T radius, const uint32_t subdivisions)
{
    // todo: name
    vic::geom::Interval<T> interval{-1., 1.};
    const vic::geom::AABB<T, 3> bbox{interval, interval, interval};

    auto mesh = GenerateCube(bbox);

    for(uint32_t i = 0; i < subdivisions; ++i)
        mesh = Subdivide(mesh);

    for(auto& vertex : mesh.vertices)
    {
        const auto norm = vic::linalg::Norm(vertex);
        vertex = vic::linalg::Matmul(radius / norm, vertex);
    }

    return mesh;
}

template <typename T>
TriMesh GenerateUVSphere(const T rad, //
                         const MeshIndex nu,
                         const MeshIndex nv)
{
    // simple uv sphere mesh
    Vertex top{{0., 0., rad}};
    Vertex bottom{{0., 0., -rad}};

    std::vector<Vertex> vertices;

    for(MeshIndex iu = 0; iu < nu; ++iu)
    {
        const auto ratio_u = (double)iu / (double)nu;
        const auto x = std::sin(ratio_u * 2. * std::numbers::pi);
        const auto y = std::cos(ratio_u * 2. * std::numbers::pi);

        for(MeshIndex iv = 0; iv < nv; ++iv)
        {
            const auto ratio_v = (double)(iv + 1) / (double)(nv + 1);
            const auto z = -rad * std::cos(ratio_v * std::numbers::pi); // start at bottom, move up

            // calculate the scaling of x and y
            const auto scale = std::sqrt((rad * rad) - (z * z));

            vertices.push_back(Vertex({x * scale, y * scale, z}));
        }
    }
    vertices.push_back(bottom);
    vertices.push_back(top);
    const MeshIndex ibottom = (MeshIndex)vertices.size() - 2;
    const MeshIndex itop = (MeshIndex)vertices.size() - 1;

    std::vector<Tri> tris;

    const auto index = [&](const MeshIndex iu, const MeshIndex iv) -> MeshIndex {
        return MeshIndex(iu + (iv * nu)); //
    };

    // add main quads
    for(MeshIndex iu = 0; iu < nu; ++iu)
    {
        const MeshIndex iu_next = (iu + 1) % nu;
        for(MeshIndex iv = 0; iv < (nv - 1); ++iv)
        {
            tris.push_back(Tri{index(iu, iv), //
                               index(iu_next, iv),
                               index(iu, iv + 1)});

            tris.push_back(Tri{index(iu_next, iv), //
                               index(iu_next, iv + 1),
                               index(iu, iv + 1)});
        }
    }

    // add bottom triangles, that connect 1 point to the bottom vertex
    for(MeshIndex iu = 0; iu < nu; ++iu)
        tris.push_back(Tri{iu, MeshIndex{(iu + 1) % nu}, ibottom});

    // add top triangles
    for(MeshIndex iu = 0; iu < nu; ++iu)
    {
        const MeshIndex iv_last = nv - 2;
        tris.push_back(Tri{index(iu, iv_last), //
                           index((iu + 1) % nu, iv_last),
                           itop});
    }

    return TriMesh{vertices, tris};
}

template <typename T>
TriMesh GenerateCone(const T rad, //
                     const T height,
                     const MeshIndex n)
{
    // simple uv sphere mesh
    const Vertex top = ToVertex(0., 0., height);
    const Vertex bottom = ToVertex(0., 0., 0.);

    TriMesh mesh{};
    mesh.vertices.reserve(n + 2);
    mesh.tris.reserve(n * 2);

    for(MeshIndex i = 0; i < n; ++i)
    {
        const auto ratio = (double)i / (double)n;
        const auto x = rad * std::sin(ratio * 2. * std::numbers::pi);
        const auto y = rad * std::cos(ratio * 2. * std::numbers::pi);
        mesh.vertices.push_back(Vertex{{x, y, 0.}});
    }
    mesh.vertices.push_back(top);
    mesh.vertices.push_back(bottom);

    const MeshIndex itop = (MeshIndex)mesh.vertices.size() - 2;
    const MeshIndex ibottom = (MeshIndex)mesh.vertices.size() - 1;

    for(MeshIndex i = 0; i < n; ++i)
    {
        const MeshIndex i2 = (i + 1) % n;
        mesh.tris.push_back(Tri{i, i2, itop}); // top vertex
        mesh.tris.push_back(Tri{i2, i, ibottom}); // bottom vertex, note: reversed
    }

    return mesh;
}

TriMesh Subdivide(const TriMesh& mesh)
{
    using namespace vic::linalg;

    // todo: Verify that no edges are between the same vertex?
    // todo: Right now i construct an std::map of edges. This will not scale well. Maybe change to a sorted vector

    // construct list of edges
    std::map<std::pair<MeshIndex, MeshIndex>, MeshIndex> uniqueEdges;
    auto addEdge = [&](const MeshIndex v1, const MeshIndex v2) {
        if(v1 < v2)
            uniqueEdges[{v1, v2}] = {};
        else
            uniqueEdges[{v2, v1}] = {};
    };
    for(const auto& tri : mesh.tris)
    {
        const auto& [v0, v1, v2] = tri;
        addEdge(v0, v1);
        addEdge(v1, v2);
        addEdge(v2, v0);
    }

    // initialize result
    TriMesh result;
    result.vertices = mesh.vertices;
    result.tris.reserve(mesh.tris.size() * 4);

    //for each of the edges, add a new vertex, halfway
    for(auto& [key, value] : uniqueEdges)
    {
        const auto& [v1, v2] = key;
        auto newVertex = Matmul(0.5, Add(mesh.vertices.at(v1), mesh.vertices.at(v2)));
        result.vertices.push_back(newVertex);

        value = (MeshIndex)result.vertices.size() - 1;
    }

    // iterate over all triangles in the original mesh. perform subdivision

    for(const auto& tri : mesh.tris)
    {
        const auto& [v0, v1, v2] = tri;

        const auto e0 = v0 < v1 ? std::pair(v0, v1) : std::pair(v1, v0);
        const auto e1 = v1 < v2 ? std::pair(v1, v2) : std::pair(v2, v1);
        const auto e2 = v2 < v0 ? std::pair(v2, v0) : std::pair(v0, v2);

        const auto s0 = uniqueEdges.at(e0);
        const auto s1 = uniqueEdges.at(e1);
        const auto s2 = uniqueEdges.at(e2);

        result.tris.push_back(Tri{v0, s0, s2});
        result.tris.push_back(Tri{s2, s0, s1});
        result.tris.push_back(Tri{s0, v1, s1});
        result.tris.push_back(Tri{s2, s1, v2});
    }

    return result;
}

template <typename T>
EdgeMesh GenerateCircle(const T radius, const std::size_t n)
{
    EdgeMesh result;

    for(std::size_t i = 0; i < n; ++i)
    {
        const auto ratio = (double)i / (double)n;
        const double x = std::sin(ratio * 2. * std::numbers::pi);
        const double y = std::cos(ratio * 2. * std::numbers::pi);
        result.vertices.push_back(ToVertex(x, y, 0.));
        result.edges.push_back({i, (i + 1) % n});
    }

    return result;
}

TriMesh Revolve(const EdgeMesh& mesh, //
                const Vertex& upVector,
                const std::size_t n)
{
    TriMesh result;
    return result;
}

template <typename T>
TriMesh GenerateTorus(const T R, //
                      const T r,
                      const std::size_t nR,
                      const std::size_t nr)
{
    TriMesh result;

    auto circleMesh = GenerateCircle(r, nr);

    // circleMesh will be in the xy plane, in order to revolve, we need to switch x and z coords
    for(auto& vertex : circleMesh.vertices)
        vertex = ToVertex(vertex.Get(0, 0), vertex.Get(1, 0), vertex.Get(2, 0));

    return result;
}

} // namespace mesh
} // namespace vic