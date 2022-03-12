#pragma once

#include <array>
#include <vector>
#include <limits>

#include "vic/graph/graph.h"
#include "vic/graph/iterators.h"
#include "vic/utils.h"

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
    using VertexIdType = typename TGraph::VertexIdType;

    FloydWarshall(TGraph& graph, TEdgeCostFunctor functor)
        : mGraph(graph)
        , mEdgeCostFunctor(functor) {}

private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor;
    using CostType = decltype(mEdgeCostFunctor(0)); // return type of lambda

    // TODO(vicdie): replace these with a matrix
    std::vector<std::vector<CostType>> mCostMatrix{};
    std::vector<std::vector<VertexIdType>> mPolicyMatrix{};

public:
    void Update()
    {
        // TODO(vicdie): use Matrix from vic::linalg
        // initialize
        const auto n = mGraph.GetNumVertices();
        constexpr CostType maxval = std::numeric_limits<CostType>::max() / 4.;
        mCostMatrix = InitializeEmpty<CostType>(maxval, n, n);
        mPolicyMatrix = InitializeEmpty<VertexIdType>(n, n);

        // vertex to itself is zero
        for (const auto& vertex : VertexIterator(mGraph))
        {
            const auto id = vertex.Id();
            mCostMatrix[id][id] = 0.;
            mPolicyMatrix[id][id] = id;
        }

        // set value of the diagonal
        for (const auto& edge : EdgeIterator(mGraph))
        {
            mCostMatrix[edge.Source()][edge.Sink()] = mEdgeCostFunctor(edge.Id());
            mPolicyMatrix[edge.Source()][edge.Sink()] = edge.Source();
            if constexpr (!directed)
            {
                mCostMatrix[edge.Sink()][edge.Source()] = mEdgeCostFunctor(edge.Id());
                mPolicyMatrix[edge.Sink()][edge.Source()] = edge.Sink();
            }
        }

        // perform calculation
        for (const auto& vk : VertexIterator(mGraph))
        {
            for (const auto& vi : VertexIterator(mGraph))
            {
                for (const auto& vj : VertexIterator(mGraph))
                {
                    const double sum_ik_kj = mCostMatrix[vi.Id()][vk.Id()] + mCostMatrix[vk.Id()][vj.Id()];
                    if (mCostMatrix[vi.Id()][vj.Id()] > sum_ik_kj)
                    {
                        mCostMatrix[vi.Id()][vj.Id()] = sum_ik_kj;
                        mPolicyMatrix[vi.Id()][vj.Id()] = mPolicyMatrix[vi.Id()][vk.Id()];
                    }
                }
            }
        }
    }

    CostType Get(const VertexIdType source, const VertexIdType sink)
    {
        return mCostMatrix[source][sink]; 
    }

    VertexIdType Policy(const VertexIdType source, const VertexIdType sink)
    {
        return mPolicyMatrix[source][sink]; 
    }
};


// example of simple dijkstras algorithm
template <typename TGraph, typename TEdgeCostFunctor>
class Dijkstra
{
public:
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    Dijkstra(TGraph& graph, TEdgeCostFunctor functor)
        : mGraph(graph)
        , mEdgeCostFunctor(functor)
        , mOutIterator(graph) 
    {
        Update();
    }

private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor;
    OutIterator<TGraph> mOutIterator;

    using CostType = decltype(mEdgeCostFunctor(0)); // return type of lambda
    struct SHeapObject
    {
        VertexIdType mVertex{};
        VertexIdType mPrevous{};
        CostType mValue{};
    };

public:
    void Update() { mOutIterator.Update(); }

    std::vector<VertexIdType> Calculate(const VertexIdType source, const VertexIdType sink)
    {
        // todo
        std::vector<SHeapObject> items;
        std::vector<VertexIdType> heap;

        return {};
    }
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