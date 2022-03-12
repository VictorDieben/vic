#include "pch.h"

#include "vic/graph/algorithms.h"
#include "vic/graph/graph.h"
#include "vic/graph/iterators.h"

using namespace vic;

TEST(TestGraph, Startup)
{
    using namespace vic::graph;
    using Vertex = vic::graph::Vertex<>;
    using Edge = vic::graph::Edge<Vertex::VertexIdType>;
    using TestGraph = vic::graph::BaseGraph<Vertex, Edge>;

    TestGraph graph;

    for(int i = 0; i < 5; ++i)
        graph.AddVertex();

    EXPECT_EQ(graph.GetVertex(0).Id(), 0);
    EXPECT_EQ(graph.GetVertex(4).Id(), 4);

    // iterate over all vertices
    size_t sum = 0;
    for(const auto& vertex : VertexIterator(graph))
        sum += vertex.mId;

    EXPECT_EQ(sum, 0 + 1 + 2 + 3 + 4);

    // add some edges
    //   3
    // 0 1 2
    //   4
    graph.AddEdge(0, 1);
    graph.AddEdge(1, 2);
    graph.AddEdge(1, 3);
    graph.AddEdge(4, 1);

    // iterate over all edges
    sum = 0;
    for(const auto& edge : EdgeIterator(graph))
        sum += edge.mId;

    EXPECT_EQ(sum, 0 + 1 + 2 + 3);

    // test out edges
    OutIterator<TestGraph, true> outIterator(graph); // valid untill vertices/edges change in graph
    outIterator.Update();

    sum = 0;
    std::vector<Vertex::VertexIdType> outVertices;
    for(const auto& edgeId : outIterator.OutEdges(1))
    {
        const auto& edge = graph.GetEdge(edgeId);
        outVertices.push_back(edge.Sink());
        sum += edge.Sink();
    }

    EXPECT_EQ(outVertices.size(), 2);
    EXPECT_EQ(sum, 2 + 3);

    // Test out edges for non-directed graph
    OutIterator<TestGraph, false> outIterator2(graph);
    outIterator2.Update();
    outVertices.clear();
    for(const auto& edgeId : outIterator2.OutEdges(1))
    {
        const auto& edge = graph.GetEdge(edgeId);
        outVertices.push_back(edge.Sink());
    }
    EXPECT_EQ(outVertices.size(), 4);
}

auto ConstructGridGraph(const std::size_t nx, const std::size_t ny)
{
    using namespace vic::graph;
    struct VertexData
    {
        using VertexIdType = uint16_t;
        double x;
        double y;
    };
    using Vertex = vic::graph::Vertex<VertexData>;
    using Edge = vic::graph::Edge<Vertex::VertexIdType>;
    using TestGraph = vic::graph::BaseGraph<Vertex, Edge>;

    TestGraph graph;

    // construct vertices
    for(std::size_t j = 0; j < ny; ++j)
        for(std::size_t i = 0; i < nx; ++i)
            graph.AddVertex({double(i), double(j)});

    const auto indexLambda = [&](const std::size_t i, const std::size_t j) { return uint16_t(i + (j * nx)); };

    // construct horizontal edges
    for(std::size_t i = 0; i < nx - 1; ++i)
        for(std::size_t j = 0; j < ny; ++j)
            graph.AddEdge(indexLambda(i, j), indexLambda(i + 1, j));

    // construct vertical edges
    for(std::size_t i = 0; i < nx; ++i)
        for(std::size_t j = 0; j < ny - 1; ++j)
            graph.AddEdge(indexLambda(i, j), indexLambda(i, j + 1));
    return graph;
}

TEST(TestGraph, TestFloydWarshall)
{
    using namespace vic::graph;

    const std::size_t nx = 3, ny = 5;
    auto graph = ConstructGridGraph(nx, ny);

    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
    ASSERT_EQ(graph.GetNumEdges(), (nx * (ny - 1)) + ((nx - 1) * ny));

    // test dijkstra solver
    const auto costLambda = [](const auto& edge) { return 1.; };
    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    floydWarshall.Update(); // perform calculation

    const auto n = graph.GetNumVertices();

    for(const auto& v1 : VertexIterator(graph))
    {
        for(const auto& v2 : VertexIterator(graph))
        {
            const auto expected = std::abs(v1.mData.x - v2.mData.x) + std::abs(v1.mData.y - v2.mData.y);
            const auto fw = floydWarshall.Get(v1.Id(), v2.Id());
            EXPECT_NEAR(fw, expected, 0.0001);
        }
    }

    // TODO(vicdie): check policy
}

TEST(TestGraph, TestDijkstra)
{
    using namespace vic::graph;
    auto graph = ConstructGridGraph(11, 11);

    ASSERT_EQ(graph.GetNumVertices(), 11 * 11);
    ASSERT_EQ(graph.GetNumEdges(), 11 * 10 + 10 * 11);

    // test dijkstra solver
    const auto costLambda = [](const auto& edge) { return 1.; }; // every edge costs 1
    algorithms::Dijkstra dijkstra{graph, costLambda};

    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    floydWarshall.Update(); // perform calculation

    for(const auto& v1 : VertexIterator(graph))
    {
        for(const auto& v2 : VertexIterator(graph))
        {
            //const auto cost_dijkstra = dijkstra.Calculate(v1.Id(), v2.Id());
            //const auto cost_fw = floydWarshall.Get(v1.Id(), v2.Id());
            //EXPECT_NEAR(cost_dijkstra, cost_fw, 0.0001);
        }
    }
}

TEST(TestGraph, TestAStar)
{
    using namespace vic::graph;
    auto graph = ConstructGridGraph(11, 11);

    ASSERT_EQ(graph.GetNumVertices(), 11 * 11);
    ASSERT_EQ(graph.GetNumEdges(), 11 * 10 + 10 * 11);

    const auto costLambda = [](const auto& edge) { return 1.; }; // every edge costs 1

    // construct a floydWarshall instance for heuristic.
    // this is a perfect heuristic (all precomputed)
    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    const auto heuristicLambda = [&](const auto& source, const auto& sink) { return floydWarshall.Get(source, sink); };

    // test AStar solver
    algorithms::AStar astar{graph, costLambda, heuristicLambda};

    for(const auto& v1 : VertexIterator(graph))
    {
        for(const auto& v2 : VertexIterator(graph))
        {
            //const auto cost_astar = astar.Calculate(v1.Id(), v2.Id());
            //const auto cost_fw = floydWarshall.Get(v1.Id(), v2.Id());
            //EXPECT_NEAR(cost_dijkstra, cost_fw, 0.0001);
        }
    }
}