
#include "gtest/gtest.h"

#include "vic/graph2/graph.h"

#include <random>

using namespace vic;

using namespace vic::graph2;

struct TestVertexData
{
    using VertexIdType = uint16_t;
    double x;
    double y;
};
struct TestEdgeData
{
    using VertexIdType = uint16_t;
    using EdgeIdType = uint16_t;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
    os << "[";
    for(auto it = vec.begin(); it < vec.end(); ++it)
    {
        os << *it;
        if(it < std::prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

using TestVertexId = uint16_t;
using TestEdgeId = uint16_t;
using TestGraph = Graph<TestVertexId, TestEdgeId>;

TestGraph ConstructGridGraph(const std::size_t nx, const std::size_t ny)
{
    const auto indexLambda = [&](const std::size_t i, const std::size_t j) { return uint16_t(i + (j * nx)); };

    std::vector<TestGraph::EdgeType> edges;

    // construct horizontal edges
    for(std::size_t i = 0; i < nx - 1; ++i)
        for(std::size_t j = 0; j < ny; ++j)
            edges.emplace_back(indexLambda(i, j), indexLambda(i + 1, j));

    // construct vertical edges
    for(std::size_t i = 0; i < nx; ++i)
        for(std::size_t j = 0; j < ny - 1; ++j)
            edges.emplace_back(indexLambda(i, j), indexLambda(i, j + 1));

    return TestGraph{nx * ny, edges};
}

template <typename TGraph>
bool VerifyPathEdgeConnected(const TGraph& graph, const std::vector<typename TGraph::VertexIdType>& path)
{
    return true; //
}

TEST(Graph2, Startup)
{
    constexpr std::size_t nx = 13, ny = 7;
    TestGraph graph = ConstructGridGraph(nx, ny);

    ASSERT_EQ(graph.NumVertices(), nx * ny);
    constexpr std::size_t expectedNrEdges = (nx * (ny - 1)) + ((nx - 1) * ny);
    ASSERT_EQ(graph.NumEdges(), expectedNrEdges);

    // setup a random edge cost functor
    std::mt19937 rng(1234);
    std::uniform_real_distribution dist(1., 2.);
    std::array<double, expectedNrEdges> costs{};
    std::generate(costs.begin(), costs.end(), [&]() { return dist(rng); });
    const auto costLambda = [&](const TestVertexId from, const TestEdgeId edge, const TestVertexId to) -> double { return costs.at(edge); };

    // const auto costLambda = [&](TestVertexId, TestEdgeId, TestVertexId) -> double { return 1.; };

    // test dijkstra solver

    const auto fw = FloydWarshall(graph, costLambda);

    const VertexIterator vertexIterator{graph};

    const auto oppositeCornerVertex = (nx * ny) - 1;
    const auto path_dijkstra = vic::graph2::Dijkstra(graph, costLambda, 0, oppositeCornerVertex);

    EXPECT_EQ(path_dijkstra.size(), nx + ny - 1);
    EXPECT_EQ(path_dijkstra.front(), 0);
    EXPECT_EQ(path_dijkstra.back(), oppositeCornerVertex);
    EXPECT_TRUE(VerifyPathEdgeConnected(graph, path_dijkstra));

    for(const auto& v1 : vertexIterator)
    {
        for(const auto& v2 : vertexIterator)
        {
            const auto path_dijkstra = vic::graph2::Dijkstra(graph, costLambda, v1, v2);
            const auto cost_dijkstra = vic::graph2::PathCost(graph, costLambda, path_dijkstra);

            const auto path_fw = PolicyPath(graph, fw, v1, v2);
            const auto cost_fw = fw[v1][v2].cost;

            EXPECT_EQ(path_dijkstra, path_fw);
            EXPECT_NEAR(cost_dijkstra, cost_fw, 0.0001);
        }
    }
}
