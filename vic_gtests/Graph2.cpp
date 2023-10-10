
#include "gtest/gtest.h"

#include "vic/graph2/graph.h"

#include <random>

using namespace vic;

using namespace vic::graph2;

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

TEST(Graph2, Traits)
{
    constexpr std::size_t nx = 3, ny = 3;
    TestGraph graph = ConstructGridGraph(nx, ny);

    static_assert(ConceptGraph<TestGraph>);
    static_assert(ConceptGraphEdgeList<TestGraph>);
    static_assert(!ConceptGraphCartesian<TestGraph>);

    CartesianGraph cartesian{graph, 2};

    static_assert(ConceptGraph<decltype(cartesian)>);
    static_assert(!ConceptGraphEdgeList<decltype(cartesian)>);
    static_assert(ConceptGraphCartesian<decltype(cartesian)>);
}

TEST(Graph2, Iterators)
{
    constexpr std::size_t nx = 3, ny = 3;
    TestGraph graph = ConstructGridGraph(nx, ny);
    const auto graphIter = OutVertexIterator<TestGraph>{graph};

    CartesianGraph cartesian{graph, 2};
    OutVertexIterator<decltype(cartesian)> cartesianIter{cartesian};
}

TEST(Graph2, Startup)
{
    constexpr std::size_t nx = 6, ny = 7;
    TestGraph graph = ConstructGridGraph(nx, ny);

    static_assert(ConceptGraph<TestGraph>);
    static_assert(ConceptGraphEdgeList<TestGraph>);

    ASSERT_EQ(graph.NumVertices(), nx * ny);
    constexpr std::size_t expectedNrEdges = (nx * (ny - 1)) + ((nx - 1) * ny);
    ASSERT_EQ(graph.NumEdges(), expectedNrEdges);

    //// setup a random edge cost functor
    //std::mt19937 rng(1234);
    //std::uniform_real_distribution dist(1., 2.);
    //std::array<double, expectedNrEdges> costs{};
    //std::generate(costs.begin(), costs.end(), [&]() { return dist(rng); });
    //const auto costLambda = [&](const TestVertexId from,   const TestVertexId to) -> double { return costs.at(edge); };

    const auto costLambda = [&](const TestVertexId, const TestVertexId) -> double { return 1.; };

    const auto fw = FloydWarshall(graph, costLambda);

    const auto heuristicLambda = [&](const TestVertexId from, const TestVertexId to) -> double { return fw[from][to].cost; };

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
            const auto path_fw = PolicyPath(graph, fw, v1, v2);
            const auto cost_fw = fw[v1][v2].cost;

            const auto path_dijkstra = vic::graph2::Dijkstra(graph, costLambda, v1, v2);
            const auto cost_dijkstra = vic::graph2::PathCost(graph, costLambda, path_dijkstra);

            const auto path_astar = vic::graph2::AStar(graph, costLambda, heuristicLambda, v1, v2);
            const auto cost_astar = vic::graph2::PathCost(graph, costLambda, path_astar);

            // todo: paths don't need to be equal, but we do need to check the validity
            //EXPECT_EQ(path_dijkstra, path_fw);
            //EXPECT_EQ(path_astar, path_fw);

            EXPECT_NEAR(cost_dijkstra, cost_fw, 0.0001);
            EXPECT_NEAR(cost_astar, cost_fw, 0.0001);
        }
    }
}

TEST(Graph2, Cartesian)
{
    constexpr std::size_t nx = 4, ny = 4;
    constexpr std::size_t lastVertex = (nx * ny) - 1;
    TestGraph graph = ConstructGridGraph(nx, ny);
    CartesianGraph cartesian{graph, 2};

    static_assert(ConceptGraph<decltype(cartesian)>);
    static_assert(ConceptGraphCartesian<decltype(cartesian)>);

    const auto costLambda = [&](TestVertexId, TestVertexId) -> double { return 1.; };

    const std::vector<TestGraph::VertexIdType> startVec{0, lastVertex};
    const auto startId = ToId<CartesianVertexIdType>(std::vector<TestVertexId>{0, lastVertex}, //
                                                     cartesian.NumDimensions(),
                                                     graph.NumVertices());

    const auto endId = ToId<CartesianVertexIdType>(std::vector<TestVertexId>{lastVertex, 0}, //
                                                   cartesian.NumDimensions(),
                                                   graph.NumVertices());

    const auto dijkstraPath = Dijkstra(cartesian, costLambda, startId, endId);

    const auto heuristicLambda = [&](const TestVertexId from, const TestVertexId to) -> double {
        if(from == to)
            return 0;
        return 1;
    };

    const auto astarPath = AStar(cartesian, costLambda, heuristicLambda, startId, endId);

    const auto astarCost = PathCost(cartesian, costLambda, astarPath);

    int a = 1;
}

TEST(Graph2, PolicyPathsConflict)
{
    // straight line graph
    constexpr std::size_t nx = 4, ny = 1;
    TestGraph graph = ConstructGridGraph(nx, ny);

    const auto costLambda = [&](TestVertexId, TestVertexId) -> double { return 1.; };

    const auto fw = FloydWarshall(graph, costLambda);

    EXPECT_FALSE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 0, 1, 2, 3)));
    EXPECT_FALSE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 1, 2, 3, 3)));
    EXPECT_FALSE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 2, 1, 3, 3)));

    EXPECT_TRUE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 1, 2, 2, 3)));
    EXPECT_TRUE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 2, 1, 3, 2)));

    EXPECT_FALSE((PolicyPathsConflict<TestVertexId, decltype(fw), true>(fw, 1, 2, 2, 3)));
    EXPECT_FALSE((PolicyPathsConflict<TestVertexId, decltype(fw), true>(fw, 2, 1, 3, 2)));

    EXPECT_TRUE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 1, 1, 3, 0)));
    EXPECT_TRUE((PolicyPathsConflict<TestVertexId, decltype(fw), false>(fw, 2, 2, 0, 3)));
}

TEST(Graph2, PathsConflict)
{
    // straight line graph
    constexpr std::size_t nx = 3, ny = 3;
    TestGraph graph = ConstructGridGraph(nx, ny);

    using PathType = std::vector<TestVertexId>;

    EXPECT_FALSE((PathsConflict<PathType, false>(PathType{0, 1}, PathType{2, 3})));
    EXPECT_FALSE((PathsConflict<PathType, false>(PathType{0, 1, 2}, PathType{2, 3})));

    EXPECT_FALSE((PathsConflict<PathType, true>(PathType{0, 1, 2}, PathType{1, 2, 3})));
    EXPECT_FALSE((PathsConflict<PathType, true>(PathType{3, 2, 1}, PathType{2, 1, 0})));

    EXPECT_TRUE((PathsConflict<PathType, false>(PathType{1, 1}, PathType{3, 2, 1, 0})));
    EXPECT_TRUE((PathsConflict<PathType, false>(PathType{1, 1}, PathType{3, 2, 1, 1})));
    EXPECT_TRUE((PathsConflict<PathType, false>(PathType{1, 1}, PathType{1, 0})));

    EXPECT_TRUE((PathsConflict<PathType, false>(PathType{3, 2, 1, 0}, PathType{1, 1})));
    EXPECT_TRUE((PathsConflict<PathType, false>(PathType{3, 2, 1, 1}, PathType{1, 1})));
    EXPECT_TRUE((PathsConflict<PathType, false>(PathType{1, 0}, PathType{1, 1})));
}

TEST(Graph2, MStar)
{
    constexpr std::size_t nx = 3, ny = 3;
    TestGraph graph = ConstructGridGraph(nx, ny);
    CartesianGraph cartesian{graph, 2};

    const auto costLambda = [&](TestVertexId, TestVertexId) -> double { return 1.; };

    const auto fw = FloydWarshall(graph, costLambda);
    //

    const auto mstarPath = MStar(cartesian, costLambda, fw, {}, {});
}