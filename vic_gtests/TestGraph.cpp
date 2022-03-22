#include "pch.h"

#include "vic/graph/algorithms.h"
#include "vic/graph/graph.h"
#include "vic/graph/iterators.h"
#include "vic/graph/tensor_graph.h"

#include <algorithm>
#include <random>
#include <ranges>

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
    using Vertex = vic::graph::Vertex<TestVertexData>;
    using Edge = vic::graph::Edge<TestEdgeData>;
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

    const std::size_t nx = 25, ny = 25;
    auto graph = ConstructGridGraph(nx, ny);
    using VertexIdType = decltype(graph)::VertexIdType;
    using EdgeIdType = decltype(graph)::EdgeIdType;

    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
    ASSERT_EQ(graph.GetNumEdges(), (nx * (ny - 1)) + ((nx - 1) * ny));

    const auto costLambda = [](const VertexIdType, const EdgeIdType, const VertexIdType) { return 1.; };
    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    floydWarshall.Update(); // perform calculation

    const auto n = graph.GetNumVertices();

    for(const auto& v1 : VertexIterator(graph))
    {
        for(const auto& v2 : VertexIterator(graph))
        {
            const auto expected = std::abs(v1.mData.x - v2.mData.x) + std::abs(v1.mData.y - v2.mData.y);
            const auto fw = floydWarshall.Get(v1.Id(), v2.Id());
            ASSERT_NEAR(fw, expected, 0.0001);
        }
    }

    // TODO(vicdie): check policy
}

TEST(TestGraph, TestDijkstra)
{
    using namespace vic::graph;

    constexpr std::size_t nx = 21, ny = 14;
    auto graph = ConstructGridGraph(nx, ny);
    using VertexIdType = decltype(graph)::VertexIdType;
    using EdgeIdType = decltype(graph)::EdgeIdType;

    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
    constexpr std::size_t expectedNrEdges = (nx * (ny - 1)) + ((nx - 1) * ny);
    ASSERT_EQ(graph.GetNumEdges(), expectedNrEdges);

    // setup a random edge cost functor
    std::mt19937 rng(1234);
    std::uniform_real_distribution dist(1., 2.);
    std::array<double, expectedNrEdges> costs;
    std::generate(costs.begin(), costs.end(), [&]() { return dist(rng); });
    const auto costLambda = [&](VertexIdType, const EdgeIdType& id, VertexIdType) -> double { return costs.at(id); };

    // test dijkstra solver
    algorithms::Dijkstra dijkstra{graph, costLambda};

    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    floydWarshall.Update(); // perform calculation

    for(const auto& v1 : VertexIterator(graph))
    {
        for(const auto& v2 : VertexIterator(graph))
        {
            const auto path_dijkstra = dijkstra.Calculate(v1.Id(), v2.Id());
            const auto cost_dijkstra = dijkstra.GetCost(path_dijkstra);
            const auto cost_fw = floydWarshall.Get(v1.Id(), v2.Id());
            ASSERT_NEAR(cost_dijkstra, cost_fw, 0.0001);
        }
    }
}

TEST(TestGraph, TestAStar)
{
    using namespace vic::graph;
    constexpr std::size_t nx = 16, ny = 15;
    auto graph = ConstructGridGraph(nx, ny);
    using CostType = double;
    using VertexIdType = decltype(graph)::VertexIdType;
    using EdgeIdType = decltype(graph)::EdgeIdType;

    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
    constexpr std::size_t expectedNrEdges = (nx * (ny - 1)) + ((nx - 1) * ny);
    ASSERT_EQ(graph.GetNumEdges(), expectedNrEdges);

    // setup a random edge cost functor
    std::mt19937 rng(1234);
    std::uniform_real_distribution dist(1., 2.);
    std::array<double, expectedNrEdges> costs;
    std::generate(costs.begin(), costs.end(), [&]() { return dist(rng); });
    const auto costLambda = [&](const VertexIdType&, const EdgeIdType& id, const VertexIdType&) -> CostType { return costs.at(id); };

    // construct a floydWarshall instance for heuristic.
    // this is a perfect heuristic (all precomputed)
    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    floydWarshall.Update();
    const auto heuristicLambda = [&](const VertexIdType& source, const VertexIdType& sink) -> CostType { return floydWarshall.Get(source, sink); };

    // test AStar solver
    algorithms::AStar astar{graph, costLambda, heuristicLambda};
    astar.Update();

    for(const auto& v1 : VertexIterator(graph))
    {
        for(const auto& v2 : VertexIterator(graph))
        {
            const auto path_astar = astar.Calculate(v1.Id(), v2.Id());
            const auto cost_astar = astar.GetCost(path_astar);
            const auto cost_fw = floydWarshall.Get(v1.Id(), v2.Id());
            ASSERT_NEAR(cost_astar, cost_fw, 1E-10);
        }
    }
}

TEST(TestGraph, TestTensorGraph)
{
    using namespace vic::graph;
    constexpr std::size_t nx = 10, ny = 10;
    auto graph = ConstructGridGraph(nx, ny);
    using VertexIdType = decltype(graph)::VertexIdType;
    using VertexType = decltype(graph)::VertexType;
    using VertexIdType = decltype(graph)::VertexIdType;

    auto tensorgraph = TensorGraph(graph);
    using TensorVertexType = typename TensorVertex<VertexType>;

    ASSERT_EQ(tensorgraph.NumTensorVertices(), nx * ny);

    tensorgraph.SetDimensions(2);
    ASSERT_EQ(tensorgraph.NumTensorVertices(), Pow<2>(nx * ny));

    tensorgraph.SetDimensions(3);
    ASSERT_EQ(tensorgraph.NumTensorVertices(), Pow<3>(nx * ny));

    std::vector<VertexIdType> ids;
    for(const auto& vert : VertexIterator(graph))
        ids.push_back(vert.Id());

    TensorVertex<VertexType> tensorBackConverted;

    // convert all valid 3d tensor vertices to tensor ids, and then back.
    // make sure the conversions are accurate
    for(const VertexIdType& v1 : ids)
    {
        for(const VertexIdType& v2 : ids)
        {
            for(const VertexIdType& v3 : ids)
            {
                // create a tensor vertex [v1, v2, v3]
                const auto verts = std::vector<VertexIdType>{{v1, v2, v3}};
                TensorVertexType tvert{tensorgraph, verts};

                // convert it to a tensor id
                TensorVertexId tensor_id = tvert.ToId(tensorgraph);

                // convert it back to a tensor vertex
                tensorBackConverted.FromId(tensorgraph, tensor_id);

                // compare the objects
                const auto verts1 = tvert.GetVertices();
                const auto verts2 = tensorBackConverted.GetVertices();
                if(!(verts1 == verts2))
                    ASSERT_TRUE(false);
            }
        }
    }
}

TEST(TestGraph, TestTensorOutIter)
{
    using namespace vic::graph;
    constexpr std::size_t nx = 3, ny = 3;

    // 6, 7, 8
    // 3, 4, 5
    // 0, 1, 2

    // setup graph
    auto graph = ConstructGridGraph(nx, ny);
    auto tensorgraph = TensorGraph(graph);
    tensorgraph.SetDimensions(2);
    using VertexType = decltype(graph)::VertexType;
    using TensorVertexType = typename TensorVertex<VertexType>;

    TensorOutIterator iter{tensorgraph};
    iter.Update();

    std::size_t count = 0;
    const auto lambda = [&](const auto&) { count++; };

    // verify center point in a 2d 3x3 grid
    TensorVertexType tvert{tensorgraph, {4, 4}};
    iter.ForeachOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, 25);

    // verify corner + edge in 3x3 grid
    tvert = TensorVertexType(tensorgraph, {0, 3});
    count = 0;
    iter.ForeachOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, 3 * 4);

    // verify 2 opposing edges, in 3x3 grid
    tvert = TensorVertexType(tensorgraph, {3, 5});
    count = 0;
    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, (4 * 4) - 1);

    // verify that an invalid start node has no outs
    tvert = TensorVertexType(tensorgraph, {2, 2});
    count = 0;
    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, 0);

    // verify 2 same nodes, in 3x3 grid
    tvert = TensorVertexType(tensorgraph, {4, 4});
    count = 0;
    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, 0);

    // verify 2 closeby edges
    tvert = TensorVertexType(tensorgraph, {1, 3});
    count = 0;
    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, (4 * 4) - 2);

    // verify 3 directly connected vertices
    tensorgraph.SetDimensions(3);
    tvert = TensorVertexType(tensorgraph, {1, 4, 7});
    count = 0;
    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
    ASSERT_EQ(count, 3 * 3 * 3);
}

TEST(TestGraph, TestTensorAStar)
{
    using namespace vic::graph;
    constexpr std::size_t nx = 3, ny = 3;

    // setup graph
    auto graph = ConstructGridGraph(nx, ny);
    auto tensorgraph = TensorGraph(graph);
    tensorgraph.SetDimensions(2);
    using Graphtype = decltype(graph);
    using VertexType = Graphtype::VertexType;
    using VertexIdType = Graphtype::VertexIdType;
    using EdgeIdType = Graphtype::EdgeIdType;
    using TensorVertexType = typename TensorVertex<VertexType>;

    // setup cost and heuristic
    const auto costLambda = [&](VertexIdType, const EdgeIdType& id, VertexIdType) -> double { return 1.; };
    algorithms::FloydWarshall floydWarshall{graph, costLambda};
    floydWarshall.Update(); // perform calculation

    const auto tensorCostLambda = [](const TensorVertexType& v1, const TensorVertexType& v2) -> double {
        // for now, just assume the edge exists
        return 1.;
    };
    const auto tensorHeuristicLambda = [&](const TensorVertexType& v1, const TensorVertexType& v2) -> double {
        const auto& verts1 = v1.GetVertices();
        const auto& verts2 = v2.GetVertices();
        double maxVal = 0;
        for(std::size_t i = 0; i < verts1.size(); ++i)
            maxVal = std::max(maxVal, floydWarshall.Get(verts1.at(i), verts2.at(i)));
        return maxVal;
    };

    // setup solver
    TensorAStar tensorAStar(tensorgraph, tensorCostLambda, tensorHeuristicLambda);
    tensorAStar.Update();

    // calculate a few cases, check total duration
    auto res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {0, 2}).ToId(tensorgraph), //
                                     TensorVertexType(tensorgraph, {8, 6}).ToId(tensorgraph));
    ASSERT_EQ(res.size(), 5);

    res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {0, 2}).ToId(tensorgraph), //
                                TensorVertexType(tensorgraph, {2, 0}).ToId(tensorgraph));
    ASSERT_EQ(res.size(), 5);
    

    res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {3, 8}).ToId(tensorgraph), //
                                TensorVertexType(tensorgraph, {8, 3}).ToId(tensorgraph));
    std::vector<TensorVertexType> vertices;
    for(const TensorVertexId& item : res)
        vertices.push_back(TensorVertexType(tensorgraph, item));
    ASSERT_EQ(res.size(), 4);

}