
#include "gtest/gtest.h"

#include "vic/simulations/cfd/mesh.h"

#include <random>

using namespace vic;

namespace vic
{
namespace simulations
{
namespace cfd
{

TEST(TestSimulations, Mesh)
{
    Mesh mesh = InitializeGridMesh(11, 11);
    EXPECT_EQ(mesh.Points().size(), 11 * 11);
    EXPECT_EQ(mesh.Edges().size(), 2 * 10 * 11);
    EXPECT_EQ(mesh.Areas().size(), 10 * 10);
    // todo: verify areas are all properly closed
}

TEST(TestSimulations, Simulation)
{
    Mesh mesh = InitializeGridMesh(101, 101);
    Simulation2D simulation{mesh};
}

TEST(TestSimulations, ToPolygon)
{
    const std::size_t nItems = 10;
    std::default_random_engine g;
    std::uniform_int_distribution<std::size_t> randomIndex(1, nItems);

    std::vector<std::array<std::size_t, 2>> edges;
    for(std::size_t i = 0; i < nItems; ++i)
        edges.push_back({i, i + 1});
    edges.push_back({nItems, 0});

    // todo: switch first and second for some edges
    for(const auto i : Range(100))
    {
        std::size_t ridx = randomIndex(g);
        const auto switched = std::array<std::size_t, 2>{edges[ridx].at(1), edges[ridx].at(0)};
        edges[ridx] = switched;
    }

    std::shuffle(edges.begin() + 1, edges.end(), g);

    const auto sorted = ToPolygon(edges);

    for(std::size_t i = 0; i < nItems; ++i)
        EXPECT_EQ(i, sorted.at(i));
    EXPECT_EQ(sorted.front(), sorted.back());
}

TEST(TestSimulations, SurafaceArea)
{
    Mesh mesh{};
    mesh.AddVertex(Mesh::Vertex{{0, 0}});
    mesh.AddVertex(Mesh::Vertex{{1, 0}});
    mesh.AddVertex(Mesh::Vertex{{1, 1}});
    mesh.AddVertex(Mesh::Vertex{{0, 1}});
    mesh.AddEdge(0, 1);
    mesh.AddEdge(1, 2);
    mesh.AddEdge(2, 3);
    mesh.AddEdge(3, 0);
    mesh.AddArea(0, 1, 2, 3);

    const auto res = CaclulateSurfaceArea(mesh);
    EXPECT_DOUBLE_EQ(res.at(0), 1.);
}

} // namespace cfd
} // namespace simulations
} // namespace vic