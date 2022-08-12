#pragma once

#include "vic/geometry/algorithms/algorithms.h"
#include "vic/geometry/geometry.h"
#include "vic/linalg/matrices.h"
#include "vic/utils.h"

// paper by Jos Stam: Real-Time fluid dynamics for Games:
// https://damassets.autodesk.net/content/dam/autodesk/research/publications-assets/pdf/realtime-fluid-dynamics-for.pdf

namespace vic
{
namespace simulations
{
namespace cfd
{
using namespace vic::linalg;

struct Mesh
{
    using Vertex = Vector2<double>;
    using VertexId = std::size_t;
    using Edge = std::array<VertexId, 2>;
    using EdgeId = std::size_t;
    using Area = std::array<EdgeId, 4>;
    using AreaId = std::size_t;

    Mesh() = default;

    VertexId AddVertex(const Vertex vertex)
    {
        const auto idx = mPoints.size();
        mPoints.push_back(vertex);
        return idx;
    }
    EdgeId AddEdge(const VertexId source, const VertexId target)
    {
        const auto idx = mEdges.size();
        mEdges.push_back(Edge{source, target});
        return idx;
    }
    AreaId AddArea(const EdgeId e1, const EdgeId e2, const EdgeId e3, const EdgeId e4)
    {
        // todo: verify that the edges are connected in the right direction
        const auto idx = mAreas.size();
        mAreas.push_back(Area{e1, e2, e3, e4});
        return idx;
    }

    const std::vector<Vertex>& Points() const { return mPoints; }
    const std::vector<Edge>& Edges() const { return mEdges; }
    const std::vector<Area>& Areas() const { return mAreas; }

    std::vector<Vertex> mPoints{};
    std::vector<Edge> mEdges{};
    std::vector<Area> mAreas{};
};

std::vector<bool> DetectBoundary2D(const Mesh& mesh)
{
    // Edges that only appear once are part of the boundary
    std::vector<int> edgeOccurences{};
    edgeOccurences.resize(mesh.Edges().size());
    const auto& areas = mesh.Areas();
    for(const auto& area : areas)
        for(const auto& edge : area)
            edgeOccurences[edge] += 1;

    // find the edges that occur exactly once
    std::vector<bool> isBoundary;
    for(const auto& edge : edgeOccurences)
        isBoundary.push_back({edge == 1});

    return isBoundary;
}

std::vector<std::array<std::size_t, 2>> SortEdges(const std::vector<std::array<std::size_t, 2>>& edges)
{
    std::vector<std::array<std::size_t, 2>> result = edges; // make a copy that can be sorted
    std::size_t currentId = result.at(0).at(1);
    for(std::size_t i = 0; i < result.size() - 1; ++i)
    {
        for(std::size_t j = i + 1; j < result.size(); ++j)
        {
            if(currentId != result.at(j).at(0) && currentId != result.at(j).at(1))
                continue;

            // this is the successor edge, swap with position i+1 if we are not there currently
            currentId = (currentId == result.at(j).at(0)) ? result.at(j).at(1) : result.at(j).at(0);
            if(j != i + 1)
                std::swap(result.at(i + 1), result.at(j));
            break;
        }
    }
    return result;
}

// Sort a list where two objects can either link or not. Not really a sorting algorithm,
// but similar interface nontheless
template <typename Iterator, typename TLambda>
void SortLinked(Iterator begin, Iterator end, TLambda neighbouring)
{
    for(auto it = begin; it < end - 1; ++it)
    {
        for(auto itNext = it + 1; it < end; ++itNext)
        {
            if(!neighbouring(*it, *itNext))
                continue;
            if(it + 1 != itNext)
                std::iter_swap(it, itNext);
            break;
        }
    }
}

std::vector<Mesh::EdgeId> SortMeshEdges(const Mesh& mesh)
{
    using Item = std::pair<Mesh::EdgeId, std::array<Mesh::VertexId, 2>>;
    const auto& edges = mesh.Edges();
    for(const auto& area : mesh.Areas())
    {
        std::vector<Item> result;
        for(const auto& edgeId : area)
            result.push_back({edgeId, edges.at(edgeId)});

        // sort the edges such that they describe a closed loop
        SortLinked(result.begin(), result.end(), [&](const auto& itL, const auto& itR) {
            return (itL.second[0] == itR.second[0] || //
                    itL.second[0] == itR.second[1] || //
                    itL.second[1] == itR.second[0] || //
                    itL.second[1] == itR.second[1]);
        });
    }

    return {};
}

void PrepareMesh(const Mesh& mesh)
{
    // sort edges
    for(auto& area : mesh.Areas())
    {
        // area = SortEdges(area);
    }
}

std::vector<std::size_t> ToPolygon(const std::vector<std::array<std::size_t, 2>>& edges)
{
    // O(n^2) compares, O(n) swaps (i think)
    const auto sortedEdges = SortEdges(edges);

    std::vector<std::size_t> answer{};
    answer.push_back(sortedEdges.at(0).at(0));
    for(const auto& pair : sortedEdges)
        answer.push_back((answer.back() == pair.at(0)) ? pair.at(1) : pair.at(0));

    return answer;
}

std::vector<double> CaclulateSurfaceArea(const Mesh& mesh)
{
    std::vector<double> surfaceArea{};
    const auto& points = mesh.Points();
    const auto& edges = mesh.Edges();
    const auto& areas = mesh.Areas();
    for(const auto& area : areas)
    {
        // convert several edge ids to a sequence of vertex ids, that describe teh same polygon
        std::vector<std::array<std::size_t, 2>> areaEdges;
        for(const auto& edge : area)
            areaEdges.push_back({edges.at(edge)});
        const auto poly = ToPolygon(areaEdges);

        double areaSum = 0.;
        const auto& p0 = poly.at(0);
        for(std::size_t i = 1; i < poly.size() - 1; ++i)
        {
            // todo: check if edge is forward or backward, add +- factor
            areaSum += vic::geom::TriangleArea(points[p0], points[poly.at(i)], points[poly.at(i + 1)]);
        }
        surfaceArea.push_back(areaSum);
    }

    return surfaceArea;
}

std::vector<Vector2<double>> CalculateAreaCenters(const Mesh& mesh)
{
    const auto& points = mesh.Points();
    const auto& areas = mesh.Areas();
    std::vector<Vector2<double>> centers{};
    centers.resize(mesh.Areas().size());

    for(const auto& area : areas)
    {
        //
    }

    return centers;
}

Mesh InitializeGridMesh(const std::size_t nx, const std::size_t ny)
{
    const double sx = 1., sy = 1.;
    const auto xarr = Linspace(0., sx, nx);
    const auto yarr = Linspace(0., sy, ny);

    Mesh mesh{};
    for(const auto y : yarr)
        for(const auto x : xarr)
            mesh.AddVertex(Mesh::Vertex{{x, y}});

    const auto vertexIndex = [&](const std::size_t i, const std::size_t j) { return std::size_t(i + (j * nx)); };

    // construct horizontal edges
    for(std::size_t i = 0; i < nx - 1; ++i)
        for(std::size_t j = 0; j < ny; ++j)
            mesh.AddEdge(vertexIndex(i, j), vertexIndex(i + 1, j));
    // construct vertical edges
    for(std::size_t i = 0; i < nx; ++i)
        for(std::size_t j = 0; j < ny - 1; ++j)
            mesh.AddEdge(vertexIndex(i, j), vertexIndex(i, j + 1));

    const auto horizontalEdgeIndex = [&](const std::size_t i, const std::size_t j) {
        return uint32_t(i + (j * nx)); //
    };
    const auto verticalEdgeIndex = [&](const std::size_t i, const std::size_t j) {
        return ((nx - 1) * ny) + uint32_t(i + (j * nx)); //
    };

    // construct areas
    for(std::size_t i = 0; i < nx - 1; ++i)
        for(std::size_t j = 0; j < ny - 1; ++j)
            mesh.AddArea(horizontalEdgeIndex(i, j), //
                         verticalEdgeIndex(i, j),
                         horizontalEdgeIndex(i + 1, j),
                         verticalEdgeIndex(i, j + 1));

    return mesh;
}

class Simulation2D
{
public:
    using Velocity = vic::linalg::Vector2<double>;
    using Pressure = vic::linalg::Vector1<double>;

    using VelocityChain = std::vector<Velocity>;
    using PressureChain = std::vector<Pressure>;

    struct State
    {
        std::vector<VelocityChain> velocities;
        std::vector<PressureChain> pressures;
    };

    Simulation2D(const Mesh& mesh)
        : mMesh(mesh)
    { }

    State Step(const State& current, const double dt)
    {
        State next{};

        return next;
    }

private:
    const Mesh& mMesh;
};

} // namespace cfd
} // namespace simulations
} // namespace vic