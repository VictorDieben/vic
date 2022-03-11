#pragma once

#include <array>
#include <vector>


#include "graph.h"
#include "iterators.h"

// TODO(vicdie): make subfolder for algorithms?


namespace vic
{
namespace graph
{
namespace algorithms
{

// Calculate the matrix of shortest distances and shortest paths
// Can be used by other algorithms for policy.
template <typename TGraph, typename TEdgeCostFunctor, bool directed = false >
class FloydWarshall
{
public:
    using CostType = double; // TODO: lambda return type
    using VertexIdType = typename TGraph::VertexIdType;

    FloydWarshall(TGraph& graph, TEdgeCostFunctor functor)
        : mGraph(graph)
        , mEdgeCostFunctor(functor) {}

    void Update()
    {
        const auto size = mGraph.GetNumVertices();
        mCostMatrix = InitializeEmpty(0., size, size);
    }
private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor;

    // TODO(vicdie): replace these with a matrix
    std::vector<std::vector<CostType>> mCostMatrix{};
    std::vector<std::vector<VertexIdType>> mPolicyMatrix{};
};


// example of simple dijkstras algorithm
template <typename TGraph, typename TEdgeCostFunctor>
class Dijkstra
{
public:
    using GraphType = typename TGraph::GraphType;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    Dijkstra(TGraph& graph, TEdgeCostFunctor functor)
        : mGraph(graph)
        , mEdgeCostFunctor(functor) {}

    void Update()
    {
        // todo
    }
private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor;
};

// example of A* implementation
template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
class AStar
{
public:
    using GraphType = typename TGraph::GraphType;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    AStar(TGraph& graph, TEdgeCostFunctor edgeCost, THeuristicFunctor heuristic)
        : mGraph(graph)
        , mEdgeCostFunctor(edgeCost)
        , mHeuristicFunctor(heuristic) {}

    void Update()
    {
        // todo
    }

    std::vector<VertexIdType> Calculate(const VertexIdType source, const VertexIdType sink)
    {
        // todo
        return {};
    }
private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor; // defines the exact cost of going over a certain edge
    THeuristicFunctor mHeuristicFunctor; // defines the expected cost of going from a to b
};


}
}
}