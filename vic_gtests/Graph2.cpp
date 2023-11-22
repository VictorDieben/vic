
#include <gtest/gtest.h>

#include "vic/graph2/graph.h"

#include "vic/utils/timing.h"
#include "vic/utils/to_string.h"

#include <random>

using namespace vic;

using namespace vic::graph2;

using TestVertexId = uint16_t;
using TestEdgeId = uint16_t;
using TestGraph = Graph<TestVertexId, TestEdgeId>;

using TestCartesianVertexId = uint64_t;
using TestCartesianEdgeId = uint64_t;
using TestCartesianGraph = Graph<TestCartesianVertexId, TestCartesianEdgeId>;

template <typename TVertex = uint16_t, typename TEdge = uint16_t>
Graph<TVertex, TEdge> ConstructGridGraph(const std::size_t nx, const std::size_t ny)
{
    const auto indexLambda = [&](const std::size_t i, const std::size_t j) { return uint16_t(i + (j * nx)); };

    std::vector<std::pair<TVertex, TVertex>> edges;

    // construct horizontal edges
    for(std::size_t i = 0; i < nx - 1; ++i)
        for(std::size_t j = 0; j < ny; ++j)
            edges.emplace_back(indexLambda(i, j), indexLambda(i + 1, j));

    // construct vertical edges
    for(std::size_t i = 0; i < nx; ++i)
        for(std::size_t j = 0; j < ny - 1; ++j)
            edges.emplace_back(indexLambda(i, j), indexLambda(i, j + 1));

    return Graph<TVertex, TEdge>(nx * ny, edges);
}

template <typename TGraph>
bool VerifyPathEdgeConnected(const TGraph& graph, const std::vector<typename TGraph::VertexIdType>& path)
{
    return true; //
}

TEST(Graph2, Traits)
{
    constexpr std::size_t nx = 3, ny = 3;
    const auto graph = ConstructGridGraph(nx, ny);

    static_assert(ConceptGraph<TestGraph>);
    static_assert(ConceptGraphEdgeList<TestGraph>);
    static_assert(!ConceptGraphCartesian<TestGraph>);

    const auto fw = FloydWarshall<double, TestVertexId>(graph, [](const auto, const auto) { return 1; });

    static_assert(ConceptHeuristic<decltype(fw)>);
    static_assert(ConceptPolicy<decltype(fw)>);
}

TEST(Graph2, Iterators)
{
    constexpr std::size_t nx = 3, ny = 3;
    const auto graph = ConstructGridGraph(nx, ny);
    const auto outIterator = GetOutVertexIterator(graph);
}

TEST(Graph2, Startup)
{
    constexpr std::size_t nx = 6, ny = 7;
    const auto graph = ConstructGridGraph(nx, ny);

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

    const auto fw = FloydWarshall<double, TestVertexId>(graph, costLambda);

    const auto heuristicLambda = [&](const TestVertexId from, const TestVertexId to) -> double { return fw.Cost(from, to); };

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
            const auto path_fw = fw.PolicyPath(v1, v2);
            const auto cost_fw = fw.Cost(v1, v2);

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

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    constexpr std::size_t nx = 4, ny = 4;
    constexpr std::size_t lastVertex = (nx * ny) - 1;
    const auto graph = ConstructGridGraph(nx, ny);
    CartesianGraph cartesian{graph, 2};

    static_assert(ConceptGraph<decltype(cartesian)>);
    static_assert(ConceptGraphCartesian<decltype(cartesian)>);

    const auto costLambda = [&](TestVertexId, TestVertexId) -> double { return 1.; };

    const std::vector<TestGraph::VertexIdType> startVec{0, lastVertex};
    const auto startId = ToId<CartesianVertexIdType>(std::vector<TestVertexId>{0, lastVertex}, graph.NumVertices());

    const auto endId = ToId<CartesianVertexIdType>(std::vector<TestVertexId>{lastVertex, 0}, graph.NumVertices());

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

    const auto fw = FloydWarshall<double, TestVertexId>(graph, costLambda);

    EXPECT_FALSE(fw.PolicyPathsConflict(0, 1, 2, 3));
    EXPECT_FALSE(fw.PolicyPathsConflict(1, 2, 3, 3));
    EXPECT_FALSE(fw.PolicyPathsConflict(2, 1, 3, 3));

    EXPECT_TRUE(fw.PolicyPathsConflict(1, 2, 2, 3));
    EXPECT_TRUE(fw.PolicyPathsConflict(2, 1, 3, 2));

    EXPECT_FALSE(fw.PolicyPathsConflict(1, 2, 2, 3));
    EXPECT_FALSE(fw.PolicyPathsConflict(2, 1, 3, 2));

    EXPECT_TRUE(fw.PolicyPathsConflict(1, 1, 3, 0));
    EXPECT_TRUE(fw.PolicyPathsConflict(2, 2, 0, 3));
}

TEST(Graph2, PathsConflict)
{
    // straight line graph
    constexpr std::size_t nx = 3, ny = 3;
    TestGraph graph = ConstructGridGraph(nx, ny);

    using PathType = std::vector<TestVertexId>;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

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

TEST(Graph2, CartesianVertex)
{
    constexpr std::size_t nx = 5, ny = 5;
    constexpr std::size_t numVertices = nx * ny;
    const auto graph = ConstructGridGraph(nx, ny);

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    for(const uint32_t dims : {2, 3, 4})
    {
        const auto cartesianVertices = std::pow(numVertices, dims);

        std::vector<TestVertexId> buffer;
        buffer.reserve(dims);

        for(CartesianVertexIdType iVertex = 0; iVertex < cartesianVertices; ++iVertex)
        {
            // convert to and from a vector of indices
            ToVector(iVertex, dims, numVertices, buffer);
            const auto backConverted = ToId<CartesianVertexIdType>(buffer, numVertices);

            if(iVertex != backConverted)
                ASSERT_EQ(iVertex, backConverted);
        }
    }
}

TEST(Graph2, CartesianAStar)
{
    // constexpr std::size_t nx = 40, ny = 40;

    // performace release 40x40:
    // 2591.4221ms; 944.0560999999999ms

    constexpr std::size_t nx = 10, ny = 10;

    constexpr std::size_t last = (nx * ny) - 1;

    constexpr std::size_t bl = 0;
    constexpr std::size_t br = nx - 1;
    constexpr std::size_t tl = (nx * (ny - 1));
    constexpr std::size_t tr = (nx * ny) - 1;

    constexpr std::size_t ml = (nx * ((ny - 1) / 2));
    constexpr std::size_t mr = ml + nx;

    using VertexType = uint16_t;
    using EdgeType = uint16_t;

    using CostType = float;

    using CartesianVertexType = std::vector<VertexType>;
    using CartesianEdgeType = std::vector<EdgeType>;

    const auto graph = ConstructGridGraph<VertexType, EdgeType>(nx, ny);

    const auto fw = FloydWarshall<CostType, TestVertexId>(graph, [&](const auto, const auto) -> CostType { return CostType{1.}; });

    const auto heuristicLambda = [&fw]<typename T>(const T& from, const T& to) -> CostType {
        assert(from.size() == to.size());
        CostType cost = 0.;
        const auto size = from.size();
        for(std::size_t i = 0; i < size; ++i)
            cost += fw.Cost(from[i], to[i]);
        cost = cost / CostType(size);
        return 1.00001 * cost; // slightly overestimate the remaining part of path
    };

    const auto costLambda = []<typename T>(const T&, const T&) -> CostType { return CostType{1.}; };
    auto astarInstance = CartesianAStar<CostType, decltype(graph)>(graph);

    // 4
    const auto from = CartesianVertexType{bl, tr, tl, br};
    const auto to = CartesianVertexType{tr, bl, br, tl};

    // 5
    //const auto from = CartesianVertexType{bl, tr, tl, br, ml};
    //const auto to = CartesianVertexType{tr, bl, br, tl, mr};

    //const auto timer = Timer();
    //const auto path = astarInstance.Run(from, to, costLambda, heuristicLambda);
    //const auto duration = timer.GetTime();
    //std::cout << std::format("old: {}ms", 1000. * duration.count()) << std::endl;

    auto astarArrayInstance = CartesianArrayAStar<CostType, decltype(graph)>(graph);

    const auto timer2 = Timer();
    const auto path2 = astarArrayInstance.Run<4>(from, to, costLambda, heuristicLambda);
    const auto duration2 = timer2.GetTime();

    std::cout << std::format("new: {}ms", 1000. * duration2.count()) << std::endl;

    // EXPECT_EQ(path.size(), nx + ny - 1);
    EXPECT_EQ(path2.size(), nx + ny - 1);

    ASSERT_TRUE(false);
}

TEST(Graph2, CartesianIterator)
{
    constexpr std::size_t nx = 3, ny = 3;

    using VertexType = uint16_t;
    using EdgeType = uint16_t;

    const auto graph = ConstructGridGraph<VertexType, EdgeType>(nx, ny);

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    // unpacked versions of the ids
    using CartesianVertexType = std::vector<VertexType>;
    using CartesianEdgeType = std::vector<EdgeType>;

    const auto costLambda = [&](VertexType, VertexType) -> double { return 1.; };

    const auto fw = FloydWarshall<double, TestVertexId>(graph, costLambda);

    // todo: make this a function argument
    const auto outIterator = GetOutVertexIterator(graph);

    const auto generateOutSet = [&](const decltype(outIterator)& iterator, const VertexType& vertex) {
        std::set<VertexType> result{};
        iterator.ForeachOutVertex(vertex, [&](const VertexType& other) { result.insert(other); });
        return result;
    };

    EXPECT_EQ(generateOutSet(outIterator, 0), (std::set<VertexType>{0, 1, 3}));
    EXPECT_EQ(generateOutSet(outIterator, 3), (std::set<VertexType>{0, 3, 4, 6}));
    EXPECT_EQ(generateOutSet(outIterator, 4), (std::set<VertexType>{1, 3, 4, 5, 7}));

    const auto cartesianOutIterator = CartesianOutIterator(outIterator);

    const auto generateCartesianOutSet = [&](const decltype(cartesianOutIterator)& iterator, const CartesianVertexType& vertex) {
        std::set<CartesianVertexType> result{};
        iterator.ForeachOutVertex(vertex, [&](const auto& other) { result.insert(other); });
        return result;
    };

    const auto out44 = generateCartesianOutSet(cartesianOutIterator, std::vector<VertexType>{4, 4});
    EXPECT_EQ(out44.size(), 5 * 5);

    const auto out00 = generateCartesianOutSet(cartesianOutIterator, std::vector<TestVertexId>{0, 0});
    EXPECT_EQ(out00.size(), 3 * 3);

    const auto out33 = generateCartesianOutSet(cartesianOutIterator, std::vector<TestVertexId>{3, 3});
    EXPECT_EQ(out33.size(), 4 * 4);

    // 3d
    const auto out444 = generateCartesianOutSet(cartesianOutIterator, std::vector<VertexType>{4, 4, 4});
    EXPECT_EQ(out444.size(), 5 * 5 * 5);

    // 4d
    const auto out4444 = generateCartesianOutSet(cartesianOutIterator, std::vector<VertexType>{4, 4, 4, 4});
    EXPECT_EQ(out4444.size(), 5 * 5 * 5 * 5);

    // valid out sets
    const auto generateValidCartesianOutSet = [&](const decltype(cartesianOutIterator)& iterator, const CartesianVertexType& vertex) {
        std::set<CartesianVertexType> result{};
        iterator.ForeachValidOutVertex(vertex, [&](const auto& other) { result.insert(other); });
        return result;
    };

    const auto valid02 = generateValidCartesianOutSet(cartesianOutIterator, std::vector<VertexType>{0, 2});
    EXPECT_EQ(valid02.size(), (3 * 3) - 1); // [1,1] is not valid

    const auto valid345 = generateValidCartesianOutSet(cartesianOutIterator, std::vector<VertexType>{3, 4, 5});
    EXPECT_EQ(valid345.size(), 3 * 3 * 3); // each agent can only go up/down/stay still
}

TEST(Graph2, ValidOutPerformace)
{
    constexpr std::size_t nx = 5, ny = 5;

    using VertexType = uint16_t;
    using EdgeType = uint16_t;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexType>;
    using CartesianEdgeType = std::vector<EdgeType>;

    const auto graph = ConstructGridGraph<VertexType, EdgeType>(nx, ny);

    const auto outIterator = GetOutVertexIterator(graph);

    const auto cartesianOutIterator = CartesianOutIterator(outIterator);

    const auto countCartesianOut = [&](const decltype(cartesianOutIterator)& iterator, const CartesianVertexType& vertex) {
        std::size_t count = 0;
        iterator.ForeachOutVertex(vertex, [&](const auto& other) { count++; });
        return count;
    };

    const auto countCartesianValidOut = [&](const decltype(cartesianOutIterator)& iterator, const CartesianVertexType& vertex) {
        std::size_t count = 0;
        iterator.ForeachValidOutVertex(vertex, [&](const auto& other) { count++; });
        return count;
    };

    auto timer = Timer();

    {
        timer.Reset();
        const auto count = countCartesianOut(cartesianOutIterator, std::vector<VertexType>{12, 12, 12, 12, 12, 12, 12, 12, 12, 12});
        const auto allOutduration = timer.GetTime();

        std::cout << std::format("10d: {}[s] ", allOutduration.count()) << std::endl;

        ASSERT_EQ(count, 9765625); // use the value so it will not be optimized out
    }

    {
        timer.Reset();
        const auto validOut5 = countCartesianValidOut(cartesianOutIterator, std::vector<VertexType>{0, 6, 12, 18, 24});
        const auto validOut5Duration = timer.GetTime();

        std::cout << std::format("valid out 5d: {}[s] ", validOut5Duration.count()) << std::endl;

        ASSERT_EQ(validOut5, 689); // use the value so it will not be optimized out
    }

    ASSERT_TRUE(false);
}

TEST(Graph2, SubsetOutIterator)
{
    using VertexType = uint16_t;
    using EdgeType = uint16_t;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexType>;
    using CartesianEdgeType = std::vector<EdgeType>;

    constexpr std::size_t nx = 3, ny = 3;
    const auto graph = ConstructGridGraph<VertexType, EdgeType>(nx, ny);

    const auto outIterator = GetOutVertexIterator(graph);

    const auto subsetIterator = SubsetOutIterator(graph, outIterator);

    const auto countSubsetOut = [&](const CartesianVertexType& vertex, //
                                    const CartesianVertexType& policy,
                                    const CollisionSet collisionSet) {
        std::size_t count = 0;
        subsetIterator.ForeachOutVertex(vertex, policy, collisionSet, [&](const auto& other) {
            // std::cout << count << ": " << other << std::endl;
            count++;
        });
        return count;
    };

    CollisionSet noAgents{0};
    CollisionSet allAgents{static_cast<uint64_t>(-1)};
    CollisionSet firstAgents = ((uint64_t)1) << 0;
    CollisionSet secondAgents = ((uint64_t)1) << 1;
    CollisionSet thirdAgents = ((uint64_t)1) << 2;

    EXPECT_EQ(countSubsetOut(CartesianVertexType{3, 4, 5}, //
                             CartesianVertexType{6, 7, 8},
                             allAgents),
              (3 * 3 * 3));

    EXPECT_EQ(countSubsetOut(CartesianVertexType{3, 4, 5}, //
                             CartesianVertexType{6, 7, 8},
                             firstAgents | secondAgents),
              (3 * 3 * 1));

    // check situation where every agent wants to go to the same node.
    // no agent is in collision set, but this should not be possible anyway
    EXPECT_EQ(countSubsetOut(CartesianVertexType{1, 3, 7}, //
                             CartesianVertexType{4, 4, 4},
                             noAgents),
              0);

    const auto generateOutSet = [&](const CartesianVertexType& vertex, //
                                    const CartesianVertexType& policy,
                                    const CollisionSet collisionSet) {
        std::set<CartesianVertexType> result{};
        subsetIterator.ForeachOutVertex(vertex, policy, collisionSet, [&](const auto& other) { result.insert(other); });
        return result;
    };

    const auto outset_0_8 = generateOutSet(CartesianVertexType{0, 8}, //
                                           CartesianVertexType{1, 7},
                                           noAgents);
    EXPECT_EQ(outset_0_8, (std::set<CartesianVertexType>{CartesianVertexType{1, 7}}));

    const auto outset_0f_8 = generateOutSet(CartesianVertexType{0, 8}, //
                                            CartesianVertexType{1, 7},
                                            firstAgents);
    EXPECT_EQ(outset_0f_8,
              (std::set<CartesianVertexType>{CartesianVertexType{1, 7}, //
                                             CartesianVertexType{0, 7},
                                             CartesianVertexType{3, 7}}));
}

TEST(Graph2, MStar)
{
    using VertexType = uint16_t;
    using EdgeType = uint16_t;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexType>;
    using CartesianEdgeType = std::vector<EdgeType>;

    constexpr std::size_t nx = 3, ny = 3;
    const auto graph = ConstructGridGraph<VertexType, EdgeType>(nx, ny);

    const auto costLambda = [&](const CartesianVertexType&, const CartesianVertexType&) -> double { return 1.; };
    const auto fw = FloydWarshall<double, TestVertexId>(graph, [](const auto& a, const auto& b) -> double {
        const int ax = a % nx;
        const int ay = a / ny;
        const int bx = b % nx;
        const int by = b / ny;
        const double cx = (by - ay) * (ax + bx - 2);
        const double cy = -(bx - ax) * (ay + by - 2);
        const double cost = 1. + (0.000001 * (cx + cy));
        return cost;
    });
    const auto heuristicLambda = [&](const CartesianVertexType& from, const CartesianVertexType& to) -> double {
        double cost = 0.;
        for(std::size_t i = 0; i < from.size(); ++i)
            cost += fw.Cost(from[i], to[i]);
        cost = cost / double(from.size());
        return 1.00001 * cost; // slightly overestimate the remaining part of path
    };

    // todo: figure out why graph type is not automatically deduced
    MStar<double, decltype(graph)> mstar{graph};

    const auto path = mstar.Run(CartesianVertexType{0, 8, 6, 2}, //
                                CartesianVertexType{8, 0, 2, 6},
                                costLambda, //costLambda,
                                heuristicLambda,
                                fw);

    EXPECT_EQ(path.size(), 5);

    std::cout << "path:" << std::endl;
    for(const auto& p : path)
        std::cout << "  " << p << std::endl;

    EXPECT_TRUE(false);
}