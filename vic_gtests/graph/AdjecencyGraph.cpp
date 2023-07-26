
#include "gtest/gtest.h"

#include "vic/graph/graph_types/adjecency_list.h"
#include "vic/graph/graph_types/cartesian_product_graph.h"
#include "vic/graph/traits.h"

#include <random>

using namespace vic;

namespace vic
{
namespace graph
{

struct TestVertex : public BaseVertex<TestVertex>
{
    double x{};
    double y{};
};

struct TestEdge : public BaseEdge<TestVertex, TestEdge>
{
    double cost{};
};

TEST(Graph, startup)
{
    ASSERT_TRUE(ConceptVertex<TestVertex>);
    ASSERT_TRUE(ConceptEdge<TestEdge>);

    //using GraphType = AdjecencyGraph<TestVertex, TestEdge>;
    //ASSERT_TRUE(ConceptGraph<GraphType>);
    //GraphType graph;
    //(void)graph;
}

TEST(Graph, CartesianGraph)
{
    ASSERT_TRUE(ConceptVertex<TestVertex>);
    ASSERT_TRUE(ConceptEdge<TestEdge>);
    // using AdjecencyGraphType = AdjecencyGraph<TestVertex, TestEdge>; //
    // using CartesianGraphType = CartesianGraph<AdjecencyGraphType, AdjecencyGraphType>;
}

//    TestGraph graph;
//
//    for(int i = 0; i < 5; ++i)
//        graph.AddVertex();
//
//    EXPECT_EQ(graph.GetVertex(0).Id(), 0);
//    EXPECT_EQ(graph.GetVertex(4).Id(), 4);
//
//    // iterate over all vertices
//    size_t sum = 0;
//    for(const auto& vertex : VertexIterator(graph))
//        sum += vertex.mId;
//
//    EXPECT_EQ(sum, 0 + 1 + 2 + 3 + 4);
//
//    // add some edges
//    //   3
//    // 0 1 2
//    //   4
//    graph.AddEdge(0, 1);
//    graph.AddEdge(1, 2);
//    graph.AddEdge(1, 3);
//    graph.AddEdge(4, 1);
//
//    // iterate over all edges
//    sum = 0;
//    for(const auto& edge : EdgeIterator(graph))
//        sum += edge.mId;
//
//    EXPECT_EQ(sum, 0 + 1 + 2 + 3);
//
//    // test out edges
//    OutIterator<TestGraph, true> outIterator(graph);
//    outIterator.Update();
//
//    sum = 0;
//    std::vector<TestVertex::VertexIdType> outVertices;
//    for(const auto& edgeId : outIterator.OutEdges(1))
//    {
//        const auto& edge = graph.GetEdge(edgeId);
//        outVertices.push_back(edge.Sink());
//        sum += edge.Sink();
//    }
//
//    EXPECT_EQ(outVertices.size(), 2);
//    EXPECT_EQ(sum, 2 + 3);
//
//    // Test out edges for non-directed graph
//    OutIterator<TestGraph, false> outIterator2(graph);
//    outIterator2.Update();
//    outVertices.clear();
//    for(const auto& edgeId : outIterator2.OutEdges(1))
//    {
//        const auto& edge = graph.GetEdge(edgeId);
//        outVertices.push_back(edge.Sink());
//    }
//    EXPECT_EQ(outVertices.size(), 4);
//}
//
//TestGraph ConstructGridGraph(const std::size_t nx, const std::size_t ny)
//{
//    TestGraph graph;
//
//    // construct vertices
//    for(std::size_t j = 0; j < ny; ++j)
//        for(std::size_t i = 0; i < nx; ++i)
//            graph.AddVertex({double(i), double(j)});
//
//    const auto indexLambda = [&](const std::size_t i, const std::size_t j) { return uint16_t(i + (j * nx)); };
//
//    // construct horizontal edges
//    for(std::size_t i = 0; i < nx - 1; ++i)
//        for(std::size_t j = 0; j < ny; ++j)
//            graph.AddEdge(indexLambda(i, j), indexLambda(i + 1, j));
//
//    // construct vertical edges
//    for(std::size_t i = 0; i < nx; ++i)
//        for(std::size_t j = 0; j < ny - 1; ++j)
//            graph.AddEdge(indexLambda(i, j), indexLambda(i, j + 1));
//    return graph;
//}
//
//auto ConstructStarGraph(const std::size_t arms)
//{
//    // star shaped graph, for tensor problem
//    TestGraph graph;
//    auto& centerVertex = graph.AddVertex({0, 0});
//    // add star points
//    for(std::size_t i = 0; i < arms; ++i)
//    {
//        auto& newVertex = graph.AddVertex();
//        graph.AddEdge(centerVertex.Id(), newVertex.Id());
//    }
//    return graph;
//}
//
//TEST(TestGraph, TestFloydWarshall)
//{
//    const std::size_t nx = 100, ny = 100;
//    TestGraph graph = ConstructGridGraph(nx, ny);
//
//    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
//    ASSERT_EQ(graph.GetNumEdges(), (nx * (ny - 1)) + ((nx - 1) * ny));
//
//    const auto costLambda = [](const TestVertexId, const TestEdgeId, const TestVertexId) { return 1.; };
//    algorithms::FloydWarshall floydWarshall{graph, costLambda};
//    floydWarshall.Update();
//
//    const auto n = graph.GetNumVertices();
//
//    for(const auto& v1 : VertexIterator(graph))
//    {
//        for(const auto& v2 : VertexIterator(graph))
//        {
//            const auto expected = std::abs(v1.mData.x - v2.mData.x) + std::abs(v1.mData.y - v2.mData.y);
//            const auto fw = floydWarshall.Get(v1.Id(), v2.Id());
//            ASSERT_NEAR(fw, expected, 0.0001);
//        }
//    }
//
//    // policy
//    std::vector<TestVertexId> vec;
//
//    floydWarshall.PolicyPath(vec, 0, 24);
//    ASSERT_EQ(vec.size(), 25);
//
//    floydWarshall.PolicyPath(vec, 24, 0);
//    ASSERT_EQ(vec.size(), 25);
//}
//
//TEST(TestGraph, TestDijkstra)
//{
//    constexpr std::size_t nx = 13, ny = 14;
//    TestGraph graph = ConstructGridGraph(nx, ny);
//
//    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
//    constexpr std::size_t expectedNrEdges = (nx * (ny - 1)) + ((nx - 1) * ny);
//    ASSERT_EQ(graph.GetNumEdges(), expectedNrEdges);
//
//    // setup a random edge cost functor
//    std::mt19937 rng(1234);
//    std::uniform_real_distribution dist(1., 2.);
//    std::array<double, expectedNrEdges> costs;
//    std::generate(costs.begin(), costs.end(), [&]() { return dist(rng); });
//    const auto costLambda = [&](TestVertexId, const TestEdgeId& id, TestVertexId) -> double { return costs.at(id); };
//
//    // test dijkstra solver
//    algorithms::Dijkstra dijkstra{graph, costLambda};
//
//    algorithms::FloydWarshall floydWarshall{graph, costLambda};
//    floydWarshall.Update();
//
//    for(const auto& v1 : VertexIterator(graph))
//    {
//        for(const auto& v2 : VertexIterator(graph))
//        {
//            const auto path_dijkstra = dijkstra.Calculate(v1.Id(), v2.Id());
//            const auto cost_dijkstra = dijkstra.GetCost(path_dijkstra);
//            const auto cost_fw = floydWarshall.Get(v1.Id(), v2.Id());
//            ASSERT_NEAR(cost_dijkstra, cost_fw, 0.0001);
//        }
//    }
//}
//
//TEST(TestGraph, TestAStar)
//{
//    constexpr std::size_t nx = 13, ny = 14;
//    TestGraph graph = ConstructGridGraph(nx, ny);
//    using CostType = double;
//
//    ASSERT_EQ(graph.GetNumVertices(), nx * ny);
//    constexpr std::size_t expectedNrEdges = (nx * (ny - 1)) + ((nx - 1) * ny);
//    ASSERT_EQ(graph.GetNumEdges(), expectedNrEdges);
//
//    // setup a random edge cost functor
//    std::mt19937 rng(1234);
//    std::uniform_real_distribution dist(1., 2.);
//    std::array<double, expectedNrEdges> costs;
//    std::generate(costs.begin(), costs.end(), [&]() { return dist(rng); });
//    const auto costLambda = [&](const TestVertexId&, const TestEdgeId& id, const TestVertexId&) -> CostType { return costs.at(id); };
//
//    // construct a floydWarshall instance for heuristic.
//    // this is a perfect heuristic (all precomputed)
//    algorithms::FloydWarshall floydWarshall{graph, costLambda};
//    floydWarshall.Update();
//    const auto heuristicLambda = [&](const TestVertexId& source, const TestVertexId& sink) -> CostType { return floydWarshall.Get(source, sink); };
//
//    // test AStar solver
//    algorithms::AStar astar{graph, costLambda, heuristicLambda};
//    astar.Update();
//
//    for(const auto& v1 : VertexIterator(graph))
//    {
//        for(const auto& v2 : VertexIterator(graph))
//        {
//            const auto path_astar = astar.Calculate(v1.Id(), v2.Id());
//            const auto cost_astar = astar.GetCost(path_astar);
//            const auto cost_fw = floydWarshall.Get(v1.Id(), v2.Id());
//            ASSERT_NEAR(cost_astar, cost_fw, 1E-10);
//        }
//    }
//}
//
//TEST(TestGraph, TestTensorGraph)
//{
//    constexpr std::size_t nx = 10, ny = 10;
//    constexpr std::size_t nVertices = nx * ny;
//    TestGraph graph = ConstructGridGraph(nx, ny);
//
//    TensorGraph tensorgraph(graph);
//    using TensorVertexType = typename TensorVertex<TestVertex>;
//
//    ASSERT_EQ(tensorgraph.NumTensorVertices(), nVertices);
//
//    tensorgraph.SetDimensions(2);
//    ASSERT_EQ(tensorgraph.NumTensorVertices(), Pow<2>(nVertices));
//
//    tensorgraph.SetDimensions(3);
//    ASSERT_EQ(tensorgraph.NumTensorVertices(), Pow<3>(nVertices));
//
//    std::vector<TestVertexId> ids;
//    for(const auto& vert : VertexIterator(graph))
//        ids.push_back(vert.Id());
//
//    TensorVertex<TestVertex> tensorBackConverted;
//
//    // convert all valid 3d tensor vertices to tensor ids, and then back.
//    // make sure the conversions are accurate
//    for(const TestVertexId& v1 : ids)
//    {
//        for(const TestVertexId& v2 : ids)
//        {
//            for(const TestVertexId& v3 : ids)
//            {
//                // create a tensor vertex [v1, v2, v3]
//                const auto verts = std::vector<TestVertexId>{{v1, v2, v3}};
//                TensorVertexType tvert{tensorgraph, verts};
//
//                // convert it to a tensor id
//                TensorVertexId tensor_id = tvert.ToId(tensorgraph);
//
//                // convert it back to a tensor vertex
//                tensorBackConverted.FromId(tensorgraph, tensor_id);
//
//                // compare the objects
//                const auto verts1 = tvert.GetVertices();
//                const auto verts2 = tensorBackConverted.GetVertices();
//                if(!(verts1 == verts2))
//                    ASSERT_TRUE(false);
//            }
//        }
//    }
//}
//
//TEST(TestGraph, TestTensorOutIter)
//{
//    constexpr std::size_t nx = 3, ny = 3;
//
//    // 6, 7, 8
//    // 3, 4, 5
//    // 0, 1, 2
//
//    // setup graph
//    TestGraph graph = ConstructGridGraph(nx, ny);
//    TensorGraph tensorgraph(graph);
//    tensorgraph.SetDimensions(2);
//    using TensorVertexType = typename TensorVertex<TestVertex>;
//
//    TensorOutIterator iter{tensorgraph};
//    iter.Update();
//
//    std::set<TensorVertexId> ids;
//    const auto lambda = [&](const TensorVertexType& vert) { ids.insert(vert.ToId(tensorgraph)); };
//
//    // verify center point in a 2d 3x3 grid
//    TensorVertexType tvert{tensorgraph, {4, 4}};
//    ids.clear();
//    iter.ForeachOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), 5 * 5);
//
//    // verify corner + edge in 3x3 grid
//    tvert = TensorVertexType(tensorgraph, {0, 3});
//    ids.clear();
//    iter.ForeachOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), 3 * 4);
//
//    // verify 2 opposing edges, in 3x3 grid
//    tvert = TensorVertexType(tensorgraph, {3, 5});
//    ids.clear();
//    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), (4 * 4) - 1);
//
//    // verify that an invalid start node has no outs
//    tvert = TensorVertexType(tensorgraph, {2, 2});
//    ids.clear();
//    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), 0);
//
//    // verify that a different invalid start node has no outs
//    tvert = TensorVertexType(tensorgraph, {4, 4});
//    ids.clear();
//    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), 0);
//
//    // verify 2 closeby edges
//    tvert = TensorVertexType(tensorgraph, {1, 3});
//    ids.clear();
//    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), (4 * 4) - 2);
//
//    // verify 3 directly connected vertices
//    tensorgraph.SetDimensions(3);
//    tvert = TensorVertexType(tensorgraph, {1, 4, 7});
//    ids.clear();
//    iter.ForeachValidOut(tvert.ToId(tensorgraph), lambda);
//    ASSERT_EQ(ids.size(), 3 * 3 * 3);
//}
//
//TEST(TestGraph, TestTensorAStar)
//{
//    // setup graph
//    constexpr std::size_t nx = 3, ny = 3;
//    TestGraph graph = ConstructGridGraph(nx, ny);
//    TensorGraph<TestGraph> tensorgraph(graph);
//    tensorgraph.SetDimensions(2);
//
//    using TensorVertexType = typename TensorVertex<TestVertex>;
//
//    // setup cost and heuristic
//    const auto costLambda = [&](TestVertexId, const TestEdgeId& id, TestVertexId) -> double { return 1.; };
//    algorithms::FloydWarshall floydWarshall{graph, costLambda};
//    floydWarshall.Update();
//
//    const auto tensorCostLambda = [](const TensorVertexType& v1, const TensorVertexType& v2) -> double {
//        // for now, just assume the edge exists
//        return 1.;
//    };
//    const auto tensorHeuristicLambda = [&](const TensorVertexType& v1, const TensorVertexType& v2) -> double {
//        const auto& verts1 = v1.GetVertices();
//        const auto& verts2 = v2.GetVertices();
//        double maxVal = 0;
//        for(std::size_t i = 0; i < verts1.size(); ++i)
//            maxVal = std::max(maxVal, floydWarshall.Get(verts1.at(i), verts2.at(i)));
//        return maxVal;
//    };
//
//    // setup solver
//    TensorAStar tensorAStar(tensorgraph, tensorCostLambda, tensorHeuristicLambda);
//    tensorAStar.Update();
//
//    // calculate a few cases, check total duration
//    auto res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {0, 2}).ToId(tensorgraph), //
//                                     TensorVertexType(tensorgraph, {8, 6}).ToId(tensorgraph));
//    ASSERT_EQ(res.size(), 5);
//
//    res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {0, 2}).ToId(tensorgraph), //
//                                TensorVertexType(tensorgraph, {2, 0}).ToId(tensorgraph));
//    ASSERT_EQ(res.size(), 5);
//
//    res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {3, 8}).ToId(tensorgraph), //
//                                TensorVertexType(tensorgraph, {8, 3}).ToId(tensorgraph));
//    ASSERT_EQ(res.size(), 4);
//
//    res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {0, 1}).ToId(tensorgraph), //
//                                TensorVertexType(tensorgraph, {0, 4}).ToId(tensorgraph));
//    ASSERT_EQ(res.size(), 2);
//
//    // test with 3 dimensions
//    tensorgraph.SetDimensions(3);
//    res = tensorAStar.Calculate(TensorVertexType(tensorgraph, {0, 1, 2}).ToId(tensorgraph), //
//                                TensorVertexType(tensorgraph, {6, 7, 8}).ToId(tensorgraph));
//    std::vector<TensorVertexType> vertices;
//    for(const TensorVertexId& item : res)
//        vertices.push_back(TensorVertexType(tensorgraph, item));
//    ASSERT_EQ(res.size(), 3);
//}
//
//TEST(TestGraph, TestTensorAStarHighDim)
//{
//    // setup graph
//    TestGraph graph = ConstructGridGraph(10, 10);
//    TensorGraph<TestGraph> tensorgraph(graph);
//    using TensorVertexType = typename TensorVertex<TestVertex>;
//
//    // setup cost and heuristic
//    const auto costLambda = [&](TestVertexId, const TestEdgeId& id, TestVertexId) -> double { return 1.; };
//
//    algorithms::FloydWarshall floydWarshall{graph, costLambda};
//
//    const auto tensorCostLambda = [](const TensorVertexType& v1, const TensorVertexType& v2) -> double {
//        // for now, just assume the edge exists
//        return 1.;
//    };
//    const auto tensorHeuristicLambda = [&](const TensorVertexType& v1, const TensorVertexType& v2) -> double {
//        const auto& verts1 = v1.GetVertices();
//        const auto& verts2 = v2.GetVertices();
//        double maxVal = 0;
//        double cost = 0.;
//        for(std::size_t i = 0; i < verts1.size(); ++i)
//        {
//            const auto fw = floydWarshall.Get(verts1.at(i), verts2.at(i));
//            maxVal = std::max(maxVal, fw);
//            cost += fw;
//        }
//        return 2.001 * cost / double(verts1.size());
//    };
//
//    TensorAStar tensorAStar(tensorgraph, tensorCostLambda, tensorHeuristicLambda);
//
//    // setup solver
//    for(const auto dims : {5u})
//    {
//        tensorgraph.SetDimensions(dims);
//        floydWarshall.Update();
//        tensorAStar.Update();
//
//        std::vector<TestVertexId> start{};
//        std::vector<TestVertexId> end{};
//        for(std::size_t i = 0; i < dims; ++i)
//        {
//            start.push_back(TestVertexId(i));
//            end.push_back(TestVertexId(99 - i));
//        }
//
//        auto res = tensorAStar.Calculate(TensorVertexType(tensorgraph, start).ToId(tensorgraph), //
//                                         TensorVertexType(tensorgraph, end).ToId(tensorgraph));
//
//        std::vector<TensorVertexType> vertices;
//        for(const TensorVertexId& item : res)
//        {
//            vertices.push_back(TensorVertexType(tensorgraph, item));
//        }
//
//        for(const auto& vert : vertices)
//            std::cout << vert.GetVertices() << std::endl;
//
//        std::cout << "========" << std::endl;
//    }
//}

} // namespace graph
} // namespace vic