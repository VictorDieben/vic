#pragma once

#include "vic/geometry/geometry.h"
#include "vic/linalg/linalg.h"
#include "vic/utils/math.h"

#include <array>
#include <cmath>
#include <numbers> // std::numbers
#include <tuple>
#include <vector>

namespace vic
{
namespace mesh
{

template <typename T>
using Vertex = vic::geom::Point<T, 3>;

template <typename T>
using Normal = vic::geom::Direction<T, 3>;

template <typename T>
using UV = vic::geom::Point<T, 2>;

using MeshIndex = uint32_t;
using Edge = std::pair<MeshIndex, MeshIndex>;
using Tri = std::tuple<MeshIndex, MeshIndex, MeshIndex>;
using Quad = std::tuple<MeshIndex, MeshIndex, MeshIndex>;

template <typename T>
Vertex<T> ToVertex(const T x, const T y, const T z)
{
    // helper, linalg vecs don't have this constructor
    return Vertex<T>{{x, y, z}};
}

template <typename T>
struct EdgeMesh
{
    std::vector<Vertex<T>> vertices;
    std::vector<Edge> edges;
};

// todo: this is a tri mesh (each tri points to 3 indices in the vertex vector)
// also make different types of meshes
template <typename T>
struct TriMesh
{
    std::vector<Vertex<T>> vertices;
    std::vector<Tri> tris;
};

template <typename T>
struct UVTriMesh
{
    std::vector<UV<T>> uvs;
    std::vector<Tri> tris;
};

template <typename T>
bool IsClosed(const vic::mesh::TriMesh<T>& mesh)
{
    // Use Euler-Poincare characteristic, to determine of the triangle mesh is a closed 2d manifold

    // verify that the mesh is closed, by checking that all edges occur exactly twice.
    // we could extend it to also require opposite directions, so all surfaces have the same normal
    using namespace vic::mesh;

    std::map<std::pair<MeshIndex, MeshIndex>, int> edges;
    auto addEdge = [&](const MeshIndex v1, const MeshIndex v2) {
        if(v1 < v2)
            edges[{v1, v2}] += 1;
        else
            edges[{v2, v1}] += 1;
    };

    for(const auto& tri : mesh.tris)
    {
        const auto& [v0, v1, v2] = tri;
        addEdge(v0, v1);
        addEdge(v1, v2);
        addEdge(v2, v0);
    }

    for(const auto& [key, value] : edges)
        if(value != 2)
            return false;

    return true;
}

template <typename T>
bool IsClosed(const vic::mesh::EdgeMesh<T>& mesh)
{
    std::vector<uint8_t> in;
    in.resize(mesh.vertices.size());

    std::vector<uint8_t> out;
    out.resize(mesh.vertices.size());

    for(const auto& edge : mesh.edges)
    {
        in[edge.first] += 1;
        out[edge.second] += 1;
    }

    for(std::size_t i = 0; i < mesh.vertices.size(); ++i)
        if(in[i] != 1 || out[i] != 1)
            return false;

    return true;
}

template <typename T>
TriMesh<T> GenerateCube(const vic::geom::AABB<T, 3>& box)
{
    const auto lx = box.intervals.at(0).min;
    const auto ux = box.intervals.at(0).max;
    const auto ly = box.intervals.at(1).min;
    const auto uy = box.intervals.at(1).max;
    const auto lz = box.intervals.at(2).min;
    const auto uz = box.intervals.at(2).max;

    TriMesh<T> result;
    result.vertices.reserve(8);
    result.tris.reserve(12);

    result.vertices.push_back(Vertex<T>{{lx, ly, lz}}); // 0
    result.vertices.push_back(Vertex<T>{{ux, ly, lz}}); // 1
    result.vertices.push_back(Vertex<T>{{lx, uy, lz}}); // 2
    result.vertices.push_back(Vertex<T>{{ux, uy, lz}}); // 3

    result.vertices.push_back(Vertex<T>{{lx, ly, uz}}); // 4
    result.vertices.push_back(Vertex<T>{{ux, ly, uz}}); // 5
    result.vertices.push_back(Vertex<T>{{lx, uy, uz}}); // 6
    result.vertices.push_back(Vertex<T>{{ux, uy, uz}}); // 7

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
TriMesh<T> GenerateCubeSphere(const T radius, const uint32_t subdivisions)
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
TriMesh<T> GenerateUVSphere(const T rad, //
                            const MeshIndex nu,
                            const MeshIndex nv)
{
    // simple uv sphere mesh
    Vertex<T> top{{0., 0., rad}};
    Vertex<T> bottom{{0., 0., -rad}};

    std::vector<Vertex<T>> vertices;

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

            vertices.push_back(Vertex<T>({x * scale, y * scale, z}));
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
TriMesh<T> GenerateCone(const T rad, //
                        const T height,
                        const MeshIndex n)
{
    // simple uv sphere mesh
    const Vertex<T> top = ToVertex(0., 0., height);
    const Vertex<T> bottom = ToVertex(0., 0., 0.);

    TriMesh<T> mesh{};
    mesh.vertices.reserve(n + 2);
    mesh.tris.reserve(n * 2);

    for(MeshIndex i = 0; i < n; ++i)
    {
        const auto ratio = (double)i / (double)n;
        const auto x = rad * std::sin(ratio * 2. * std::numbers::pi);
        const auto y = rad * std::cos(ratio * 2. * std::numbers::pi);
        mesh.vertices.push_back(Vertex<T>{{x, y, 0.}});
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

template <typename T>
TriMesh<T> Subdivide(const TriMesh<T>& mesh)
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
    TriMesh<T> result;
    result.vertices = mesh.vertices;
    result.tris.reserve(mesh.tris.size() * 4);

    //for each of the edges, add a new vertex, halfway
    for(auto& [key, value] : uniqueEdges)
    {
        const auto& [v1, v2] = key;
        auto newVertex = Matmul(0.5f, Add(mesh.vertices.at(v1), mesh.vertices.at(v2)));
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
EdgeMesh<T> GenerateCircle(const T radius, const uint32_t n)
{
    EdgeMesh<T> result;

    for(MeshIndex i = 0; i < n; ++i)
    {
        const T ratio = (T)i / (T)n;
        const T x = radius * (T)std::sin(ratio * 2. * std::numbers::pi);
        const T y = radius * (T)std::cos(ratio * 2. * std::numbers::pi);
        result.vertices.push_back(ToVertex<T>(x, y, 0.));
        result.edges.push_back({i, (i + 1) % n});
    }

    return result;
}

template <typename T>
TriMesh<T> Revolve(const EdgeMesh<T>& mesh, //
                   const std::size_t n,
                   const bool close) // determines if a vertex is added at the bottom and top (only for open curves)
{
    static constexpr Vertex<T> zAxis{{0, 0, 1}};

    const MeshIndex stepSize = (MeshIndex)mesh.vertices.size();

    // revolve around z axis
    TriMesh<T> result;

    for(MeshIndex ring = 0; ring < n; ++ring)
    {
        const auto zRotation = vic::linalg::Rotate<T>(zAxis, (T)(((T)ring / (T)n) * std::numbers::pi * 2.));
        for(const auto& vertex : mesh.vertices)
            result.vertices.push_back(Matmul(zRotation, vertex));
    }

    for(MeshIndex ring = 0; ring < n; ++ring)
    {
        const MeshIndex nextRing = (ring + 1) % n;
        for(const auto& edge : mesh.edges)
        {
            const MeshIndex ia = edge.first + (ring * stepSize);
            const MeshIndex ib = edge.second + (ring * stepSize);
            const MeshIndex ic = edge.first + (nextRing * stepSize);
            const MeshIndex id = edge.second + (nextRing * stepSize);
            result.tris.push_back(Tri{ia, ib, ic});
            result.tris.push_back(Tri{ic, ib, id});
        }
    }

    // todo: if close == true, find all open vertices, close the mesh on them

    return result;
}

template <typename T>
TriMesh<T> GenerateTorus(const T R, //
                         const T r,
                         const uint32_t nR,
                         const uint32_t nr)
{
    TriMesh<T> result;

    auto circleMesh = GenerateCircle<T>(r, nr);

    // circleMesh will be in the xy plane, in order to revolve, we need to switch y and z coords
    // todo: rotate mesh using a helper
    for(auto& vertex : circleMesh.vertices)
        vertex = ToVertex<T>(R + vertex.Get(0), vertex.Get(2), vertex.Get(1));

    return Revolve<T>(circleMesh, nR, false);
}

template <typename T>
std::vector<Normal<T>> GenerateTriNormals(const TriMesh<T>& mesh)
{
    using namespace vic::linalg;

    std::vector<Normal<T>> normals;
    normals.reserve(mesh.tris.size());

    for(const auto& tri : mesh.tris)
    {
        const auto [ia, ib, ic] = tri;
        const auto ab = Subtract(mesh.vertices.at(ib), mesh.vertices.at(ia));
        const auto ac = Subtract(mesh.vertices.at(ic), mesh.vertices.at(ia));

        normals.push_back(Normalize(Cross(ab, ac)));
    }

    return normals;
}

template <typename T>
std::vector<Normal<T>> GenerateVertexNormals(const TriMesh<T>& mesh, const std::vector<Normal<T>>& triNormals)
{
    // Generate the vertex normals, based on the average of all the tri normals that use the vertex
    std::vector<Normal<T>> normals;
    normals.resize(mesh.vertices.size());

    // combine the normals of each triangle touching the vertex
    for(std::size_t i = 0; i < mesh.tris.size(); ++i)
    {
        const auto& triNormal = triNormals.at(i);
        const auto [ia, ib, ic] = mesh.tris.at(i);

        normals.at(ia) = Add(normals.at(ia), triNormal);
        normals.at(ib) = Add(normals.at(ib), triNormal);
        normals.at(ic) = Add(normals.at(ic), triNormal);
    }

    // normalize the resulting normal
    for(auto& normal : normals)
        normal = Normalize(normal);

    return normals;
}

template <typename T>
std::tuple<T, T, T> ToSphericalCoordinates(const Vertex<T>& direction)
{
    const T x = direction.Get(0, 0);
    const T y = direction.Get(1, 0);
    const T z = direction.Get(2, 0);

    const T radius = vic::linalg::Norm(direction);
    const T inclination = std::acos(z / radius);
    const T azimuth = vic::math::sign(y) * std::sqrt(x / ((x * x) + (y * y)));

    return std::tuple(radius, inclination, azimuth);
}

template <typename T>
Vertex<T> FromSphericalCoordinates(const T radius, const T inclination, const T azimuth)
{
    const T sinInclination = std::sin(inclination);

    const T x = radius * sinInclination * std::cos(azimuth);
    const T y = radius * sinInclination * std::sin(azimuth);
    const T z = radius * std::cos(inclination);
    return Vertex<T>{{x, y, z}};
}

template <typename T>
std::vector<UV<T>> GenerateVertexUVsPolar(const TriMesh<T>& mesh, const Vertex<T>& origin)
{
    std::vector<UV<T>> uvs;
    uvs.reserve(mesh.vertices.size());

    // todo: convert the polar coordinates of each vertex to a 2d value between 0 and 1

    for(const auto& vertex : mesh.vertices)
    {
        const auto direction = Subtract(vertex, origin);
        const auto [r, inclination, azimuth] = ToSphericalCoordinates(direction);
        uvs.push_back(UV<double>{{inclination / std::numbers::pi, azimuth / (2. * std::numbers::pi)}}); // todo: scale to [0; 1]
    }

    return uvs;
}

} // namespace mesh
} // namespace vic