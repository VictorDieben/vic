#pragma once

#include "vic/geometry/geometry.h"
#include "vic/linalg/linalg.h"
#include "vic/utils/math.h"

#include <array>
#include <cmath>
#include <numbers> // std::numbers
#include <optional>
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
using Quad = std::tuple<MeshIndex, MeshIndex, MeshIndex, MeshIndex>;
using Poly = std::vector<MeshIndex>;

template <typename T>
struct EdgeMesh
{
    std::vector<Vertex<T>> vertices;
    std::vector<Edge> edges;
};

// todo: this is a tri mesh (each tri points to 3 indices in the vertex vector)
// also make different types of meshes, and maybe concepts for each of the abilities (normals,uvs, etc.)
template <typename T>
struct TriMesh
{
    std::vector<Vertex<T>> vertices;
    std::vector<Tri> tris;
};

template <typename T>
struct Mesh
{
    std::vector<Vertex<T>> vertices;
    std::vector<Tri> tris;
    std::vector<Quad> quads;
    std::vector<Poly> polys;
};

inline std::array<Tri, 2> ToTris(const Quad& quad)
{
    const auto [a, b, c, d] = quad;
    return std::array<Tri, 2>{Tri{a, b, d}, Tri{b, c, d}};
}

inline std::vector<Tri> ToTris(const Poly& poly)
{
    // todo: this is the simplest algorithm i could come up with,
    // make a selector for the strategy, like reducing narrow tris or something
    const auto size = poly.size();
    assert(size > 2);
    std::vector<Tri> tris(size - 2);
    for(std::size_t i = 2; i < size; ++i)
        tris[i] = Tri{poly[0], poly[i - 1], poly[i]};
    return tris;
}

template <typename T>
TriMesh<T> ToTris(const Mesh<T>& mesh)
{
    TriMesh<T> result;
    result.vertices = mesh.vertices;
    result.tris.reserve(mesh.tris.size() //
                        + (2 * mesh.quads.size())); // todo: reserve for polys
    result.tris = mesh.tris;
    for(const auto& quad : mesh.quads)
    {
        const auto [t1, t2] = ToTris(quad);
        result.tris.push_back(t1);
        result.tris.push_back(t2);
    }
    for(const auto& poly : mesh.polys)
        for(const auto& tri : ToTris(poly))
            result.tris.push_back(tri);

    return result;
}

template <typename T>
bool IsClosed(const vic::mesh::TriMesh<T>& mesh)
{
    // Use Euler-Poincare characteristic, to determine of the triangle mesh is a closed 2d manifold

    // verify that the mesh is closed, by checking that all edges occur exactly twice.
    // we could extend it to also require opposite directions, so all surfaces have the same normal
    using namespace vic::mesh;

    std::map<std::pair<MeshIndex, MeshIndex>, int> edges;
    auto addEdge = [&](const MeshIndex v1, const MeshIndex v2) {
        assert(v1 != v2);
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
bool IsClosedContinuous(const vic::mesh::TriMesh<T>& mesh)
{
    // verify that the mesh is closed, _and_ that each edge occurs once in either directions
    using namespace vic::mesh;

    std::vector<Edge> edges;
    edges.reserve(mesh.tris.size() * 3);
    for(const auto& tri : mesh.tris)
    {
        const auto& [v0, v1, v2] = tri;
        edges.push_back(Edge{v0, v1});
        edges.push_back(Edge{v1, v2});
        edges.push_back(Edge{v2, v0});
    }

    // todo: Should we verify that the vertices mentioned in the tris do actually exist?
    // I think we should just skip this, and assume they do.
    // Make verification a separate step

    if(edges.size() % 2)
        return false; // odd number of edges cannot be closed

    // partition all edges based on if they are increasing or decreasing
    const auto it = std::partition(edges.begin(), edges.end(), [](const auto& edge) { return edge.first < edge.second; });

    if(std::distance(edges.begin(), it) != std::distance(it, edges.end()))
        return false; // number of up and down edges are not equal

    // sort first half on first index
    std::sort(edges.begin(), it, [](const auto& a, const auto& b) -> bool {
        return a.first == b.first //
                   ? a.second < b.second
                   : a.first < b.first; // note: basically default pair sorting
    });

    // sort second half on second index
    std::sort(it, edges.end(), [](const auto& a, const auto& b) -> bool {
        return a.second == b.second //
                   ? a.first < b.first
                   : a.second < b.second;
    });

    // make sure each up edge is matched with a down edge
    auto itFirst = edges.begin();
    auto itSecond = it;
    for(; itFirst < it && itSecond < edges.end(); itFirst++, itSecond++)
        if(itFirst->first != itSecond->second || itFirst->second != itSecond->first)
            return false;

    return true;
}

// A mesh is closed if each vertex used in the list of edges is used once as a source, and once as a sink
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
Mesh<T> GenerateQuadCube()
{
    static constexpr std::array<Vertex<T>, 8> verts = {
        Vertex<T>(-1, -1, -1), //
        Vertex<T>(-1, -1, 1), // 1
        Vertex<T>(-1, 1, -1), // 2
        Vertex<T>(-1, 1, 1), // 3
        Vertex<T>(1, -1, -1), // 4
        Vertex<T>(1, -1, 1), // 5
        Vertex<T>(1, 1, -1), // 6
        Vertex<T>(1, 1, 1) // 7
    };

    static constexpr std::array<Quad, 6> quads = {Quad{0, 1, 3, 2}, //
                                                  Quad{1, 5, 7, 3},
                                                  Quad{5, 4, 6, 7},
                                                  Quad{4, 0, 2, 6},
                                                  Quad{2, 3, 7, 6},
                                                  Quad{4, 5, 1, 0}};

    Mesh<T> result;
    result.vertices = std::vector<Vertex<T>>{verts.begin(), verts.end()};
    result.quads = std::vector<Quad>{quads.begin(), quads.end()};
    return result;
}

template <typename T>
TriMesh<T> GenerateCube()
{
    return ToTris(GenerateQuadCube<T>());
}

template <typename T>
TriMesh<T> GenerateCubeSphere(const T radius, const uint32_t subdivisions)
{
    auto mesh = GenerateCube<T>();

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
    Vertex<T> top(0., 0., rad);
    Vertex<T> bottom(0., 0., -rad);

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
    const auto top = Vertex<T>((T)0., (T)height, (T)0.);
    const auto bottom = Vertex<T>((T)0., (T)0., (T)0.);

    TriMesh<T> mesh{};
    mesh.vertices.reserve(n + 2);
    mesh.tris.reserve(n * 2);

    for(MeshIndex i = 0; i < n; ++i)
    {
        const T ratio = (T)i / (T)n;
        const T x = -rad * (T)std::sin(ratio * 2. * std::numbers::pi);
        const T z = rad * (T)std::cos(ratio * 2. * std::numbers::pi);
        mesh.vertices.push_back(Vertex<T>{x, (T)0., z});
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

inline std::array<Edge, 3> Edges(const Tri& tri)
{
    const auto& [v0, v1, v2] = tri;
    return std::array<Edge, 3>{Edge{v0, v1}, Edge{v1, v2}, Edge{v2, v0}};
}

inline std::array<Edge, 4> Edges(const Quad& quad)
{
    const auto& [v0, v1, v2, v3] = quad;
    return std::array<Edge, 4>{Edge{v0, v1}, Edge{v1, v2}, Edge{v2, v3}, Edge{v3, v0}};
}

inline std::vector<Edge> Edges(const Poly& poly)
{
    const auto size = poly.size();
    std::vector<Edge> edges;
    edges.reserve(size);
    for(std::size_t i = 0; i < size; ++i)
        edges.push_back(Edge{poly[i], poly[i % size]});
    return edges;
}

// make list of edges. An edge pointing between the same vertices but in different direction is considered another edge
template <typename T>
std::vector<std::pair<MeshIndex, MeshIndex>> Edges(const TriMesh<T>& mesh)
{
    std::vector<Edge> result;
    result.reserve(mesh.tris.size() * 3);

    // stick all edges in the list without checking if it is already in there
    for(const auto& tri : mesh.tris)
    {
        const auto edges = Edges(tri);
        result.insert(result.end(), edges.begin(), edges.end());
    }

    return result;
}

// Make list of edges, edge a->b is the same as b->a
template <typename T>
std::vector<std::pair<MeshIndex, MeshIndex>> UniqueEdges(const TriMesh<T>& mesh)
{
    std::vector<std::pair<MeshIndex, MeshIndex>> result = Edges(mesh);

    // for each edge, check if we put it in from lowest idx vertex to heighest
    for(auto& [v1, v2] : result)
        if(v2 < v1)
            std::swap(v1, v2);

    // make the list unique. This requires first sorting them. unique set will also be sorted
    std::sort(result.begin(), result.end());
    const auto it = std::unique(result.begin(), result.end());
    result.erase(it, result.end());

    return result;
}

template <typename T>
TriMesh<T> Subdivide(const TriMesh<T>& mesh)
{
    using namespace vic::linalg;

    // todo: Verify that no edges are between the same vertex?
    // todo: Right now i construct an std::map of edges. This will not scale well. Maybe change to a sorted vector

    // construct list of edges
    const auto uniqueEdgeList = UniqueEdges(mesh);
    std::map<std::pair<MeshIndex, MeshIndex>, MeshIndex> uniqueEdges;
    for(const auto& [v1, v2] : uniqueEdgeList)
        uniqueEdges[{v1, v2}] = {};

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
    for(const auto& [v0, v1, v2] : mesh.tris)
    {
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
        const T x = radius * (T)std::sin(ratio * (T)2. * std::numbers::pi);
        const T y = radius * (T)std::cos(ratio * (T)2. * std::numbers::pi);
        result.vertices.push_back(Vertex<T>(x, y, (T)0.));
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
TriMesh<T> RevolveClosed(const EdgeMesh<T>& mesh, //
                         const std::size_t n)
{
    assert(mesh.vertices.size() > 2);
    TriMesh<T> result;

    // revolve the edge around the y axis, the start and end vertices should lay on the y axis

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
        vertex = Vertex<T>(R + vertex.Get(0), vertex.Get(2), vertex.Get(1));

    return Revolve<T>(circleMesh, nR, false);
}

template <typename T>
std::vector<Normal<T>> GenerateTriNormals(const TriMesh<T>& mesh)
{
    using namespace vic::linalg;

    std::vector<Normal<T>> normals;
    normals.reserve(mesh.tris.size());

    for(const auto& [ia, ib, ic] : mesh.tris)
    {
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
    return Vertex<T>(x, y, z);
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
        uvs.push_back(UV<T>(inclination / std::numbers::pi, azimuth / (2. * std::numbers::pi))); // todo: scale to [0; 1]
    }

    return uvs;
}

template <typename T>
vic::mesh::TriMesh<T> GenerateSquareMesh()
{
    using namespace vic::mesh;
    TriMesh<T> mesh;
    mesh.vertices = {{Vertex((T)-0.5, (T)-0.5, (T)0.), //
                      Vertex((T)0.5, (T)0.5, (T)0.),
                      Vertex((T)-0.5, (T)0.5, (T)0.),
                      Vertex((T)0.5, (T)-0.5, (T)0.)}};
    mesh.tris = {Tri{0, 1, 2}, Tri{0, 1, 3}};
    return mesh;
}

//template <typename T>
//struct TriMesh
//{
//    std::vector<Vertex<T>> vertices;
//    std::vector<Tri> tris;
//};

// combine two meshes, but do not calculate mesh intersections.
template <typename T>
vic::mesh::TriMesh<T> Merge(const vic::mesh::TriMesh<T>& mesh1, const vic::mesh::TriMesh<T>& mesh2)
{
    vic::mesh::TriMesh<T> result = mesh1; // copy

    result.vertices.insert(result.vertices.end(), mesh2.vertices.begin(), mesh2.vertices.end());

    result.tris.reserve(mesh1.tris.size() + mesh2.tris.size());
    const auto offset = mesh1.vertices.size();

    for(const auto& tri : mesh2.tris)
        result.tris.push_back(Tri{std::get<0>(tri) + offset, //
                                  std::get<1>(tri) + offset,
                                  std::get<2>(tri) + offset});

    return result;
}

template <typename T>
vic::mesh::TriMesh<T> Union(const vic::mesh::TriMesh<T>& mesh1, const vic::mesh::TriMesh<T>& mesh2)
{
    //

    return {};
}

template <typename T>
vic::mesh::TriMesh<T> BevelVertices(const vic::mesh::TriMesh<T>& mesh, const T distance)
{
    vic::mesh::TriMesh<T> beveled;

    return beveled; // todo
}

template <typename T>
vic::mesh::Mesh<T> BevelEdges(const vic::mesh::TriMesh<T>& mesh, const T distance)
{
    assert(IsClosedContinuous(mesh)); // temporary, open edges need special attention

    const auto& originalTris = mesh.tris;
    const auto& originalVertices = mesh.vertices;

    vic::mesh::Mesh<T> beveled;

    // for each of the original tris, create three new vertices and the new inner tri
    for(std::size_t i = 0; i < originalTris.size(); ++i)
    {
        const auto& [ov1, ov2, ov3] = originalTris.at(i);
        const auto& v1 = originalVertices[ov1];
        const auto& v2 = originalVertices[ov2];
        const auto& v3 = originalVertices[ov3];

        const auto delta12 = Normalize(Subtract(v2, v1));
        const auto delta13 = Normalize(Subtract(v3, v1));
        const auto delta23 = Normalize(Subtract(v3, v2));

        const MeshIndex startSize = (MeshIndex)beveled.vertices.size();

        beveled.vertices.push_back(Add(v1, //
                                       Matmul(distance, delta12),
                                       Matmul(distance, delta13)));

        beveled.vertices.push_back(Add(v2, //
                                       Matmul(-distance, delta12),
                                       Matmul(distance, delta23)));

        beveled.vertices.push_back(Add(v3, //
                                       Matmul(-distance, delta13),
                                       Matmul(-distance, delta23)));

        beveled.tris.push_back(Tri{startSize, startSize + 1, startSize + 2});
    }

    // todo: same step as before, but now for quads/polys

    // note: for each of the open edges, we need to add an extra vertex. For now, assume all meshes are closed

    const auto uniqueEdges = UniqueEdges(mesh);

    using OptionalIndex = std::optional<MeshIndex>;
    std::vector<std::pair<OptionalIndex, OptionalIndex>> trisUsingEdges;
    trisUsingEdges.resize(uniqueEdges.size());

    for(std::size_t i = 0; i < originalTris.size(); ++i)
    {
        const auto& tri = originalTris.at(i);
        const auto edges = Edges(tri);
        for(auto [e1, e2] : edges)
        {
            const bool up = e1 < e2;
            if(!up)
                std::swap(e1, e2);

            // find the index of this (unique) edge, write result to temp vector
            auto it = std::find(uniqueEdges.begin(), uniqueEdges.end(), Edge{e1, e2});
            auto& item = trisUsingEdges.at(std::distance(uniqueEdges.begin(), it));

            if(up)
            {
                assert(!item.first.has_value());
                item.first = i;
            }
            else
            {
                assert(!item.second.has_value());
                item.second = i;
            }
        }
    }

    // tmp: check that all optionals are filled
    for(const auto& pair : trisUsingEdges)
        if((!pair.first.has_value()) || (!pair.second.has_value()))
            throw std::runtime_error("invalid mesh!");

    const auto vertexIndex = [](const Tri& tri, const MeshIndex vertexId) {
        if(std::get<0>(tri) == vertexId)
            return 0;
        else if(std::get<1>(tri) == vertexId)
            return 1;
        else if(std::get<2>(tri) == vertexId)
            return 2;
        // std::cout << "Problem!" << std::endl; // todo: turn last "else if" into "else", once algorithm is stable
        return -1;
    };

    // for each of the unique edges, add the corner quad
    for(std::size_t i = 0; i < uniqueEdges.size(); ++i)
    {
        const auto& [e1, e2] = uniqueEdges.at(i);
        const MeshIndex triIndex1 = trisUsingEdges.at(i).first.value(); // we previously verified existance
        const MeshIndex triIndex2 = trisUsingEdges.at(i).second.value();

        const MeshIndex idx1 = (3 * triIndex1) + vertexIndex(originalTris.at(triIndex1), e1);
        const MeshIndex idx2 = (3 * triIndex1) + vertexIndex(originalTris.at(triIndex1), e2);

        const MeshIndex idx3 = (3 * triIndex2) + vertexIndex(originalTris.at(triIndex2), e2); // note: other direction
        const MeshIndex idx4 = (3 * triIndex2) + vertexIndex(originalTris.at(triIndex2), e1);

        beveled.quads.push_back(Quad{idx1, idx2, idx3, idx4});

        // add root corner tri
        const auto rootCornerid = (MeshIndex)beveled.vertices.size();
        beveled.vertices.push_back(originalVertices.at(e1));
        beveled.tris.push_back(Tri{rootCornerid, idx1, idx4});

        // add tip corner tri
        const auto tipCornerid = (MeshIndex)beveled.vertices.size();
        beveled.vertices.push_back(originalVertices.at(e2));
        beveled.tris.push_back(Tri{tipCornerid, idx3, idx2});
    }

    return beveled; // todo
}

} // namespace mesh
} // namespace vic