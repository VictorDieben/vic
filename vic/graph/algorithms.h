#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

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
template <typename TGraph, typename TEdgeCostFunctor, bool directed = false>
class FloydWarshall
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;
    using EdgeType = typename TGraph::EdgeType;

    FloydWarshall(TGraph& graph, TEdgeCostFunctor functor)
        : mGraph(graph)
        , mEdgeCostFunctor(functor)
    { }

private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor;
    using CostType = decltype(mEdgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{})); // return type of lambda

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
        for(const auto& vertex : VertexIterator(mGraph))
        {
            const auto id = vertex.Id();
            mCostMatrix[id][id] = 0.;
            mPolicyMatrix[id][id] = id;
        }

        // TODO(vicdie): fix for when 2 edges go between the same vertices

        // set value of the direct edges
        for(const auto& edge : EdgeIterator(mGraph))
        {
            mCostMatrix[edge.Source()][edge.Sink()] = mEdgeCostFunctor(edge.Source(), edge.Id(), edge.Sink());
            mPolicyMatrix[edge.Source()][edge.Sink()] = edge.Source();
            if constexpr(!directed)
            {
                mCostMatrix[edge.Sink()][edge.Source()] = mEdgeCostFunctor(edge.Source(), edge.Id(), edge.Sink());
                mPolicyMatrix[edge.Sink()][edge.Source()] = edge.Sink();
            }
        }

        // perform calculation
        std::vector<VertexIdType> ids;
        for(const auto& vert : VertexIterator(mGraph))
            ids.push_back(vert.Id());

        for(const auto& vk : ids)
        {
            for(const auto& vi : ids)
            {
                for(const auto& vj : ids)
                {
                    const double sum_ik_kj = mCostMatrix[vi][vk] + mCostMatrix[vk][vj];
                    if(mCostMatrix[vi][vj] > sum_ik_kj)
                    {
                        mCostMatrix[vi][vj] = sum_ik_kj;
                        mPolicyMatrix[vi][vj] = mPolicyMatrix[vi][vk];
                    }
                }
            }
        }
    }

    CostType Get(const VertexIdType source, const VertexIdType sink) const { return mCostMatrix[source][sink]; }

    VertexIdType Policy(const VertexIdType source, const VertexIdType sink) const { return mPolicyMatrix[source][sink]; }
};

// example of simple dijkstras algorithm
template <typename TGraph, typename TEdgeCostFunctor>
class Dijkstra
{
public:
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;
    using EdgeType = typename GraphType::EdgeType;

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

    using CostType = decltype(mEdgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{})); // return type of lambda
    struct SHeapObject
    {
        VertexIdType mVertex{};
        // VertexIdType mPrevous{};
        CostType mValue{};
    };

public:
    void Update() { mOutIterator.Update(); }

    std::vector<VertexIdType> Calculate(const VertexIdType start, const VertexIdType target)
    {
        std::vector<SHeapObject> heap;

        // storing in a map, because we only want to store the actually visited vertices.
        // (we can have A LOT of vertices when using tensor graphs)
        std::map<VertexIdType, SHeapObject> bestAndPrevious{};

        const auto compare = [](const SHeapObject& h1, const SHeapObject& h2) { return h1.mValue > h2.mValue; };

        heap.push_back({start, 0.});
        bestAndPrevious[start] = {start, 0.};

        SHeapObject current;
        while(heap.size() > 0)
        {
            // pick the lowest value out of heap
            std::pop_heap(heap.begin(), heap.end(), compare);
            current = heap.back();
            heap.pop_back();

            if(current.mVertex == target)
                break;

            // iterate over neighbours, check with best value so far
            for(const auto& [edgeId, otherVertexId] : mOutIterator.OutEdgeVertices(current.mVertex))
            {
                const auto& edge = mGraph.GetEdge(edgeId);
                const auto edgeCost = mEdgeCostFunctor(current.mVertex, edgeId, otherVertexId);
                const auto newCost = current.mValue + edgeCost;
                auto it = bestAndPrevious.find(otherVertexId);
                if(it == bestAndPrevious.end() || newCost < it->second.mValue)
                {
                    bestAndPrevious[otherVertexId] = {current.mVertex, newCost};

                    // put the new vertex in the heap
                    heap.push_back({otherVertexId, newCost});
                    std::push_heap(heap.begin(), heap.end(), compare);
                }
            }
        }

        std::vector<VertexIdType> path;
        path.push_back(current.mVertex);
        while(true)
        {
            if(path.back() == start)
                break;
            auto next = bestAndPrevious[path.back()].mVertex;
            path.push_back(next);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    CostType GetCost(const std::vector<VertexIdType>& path)
    {
        CostType cost = 0.;
        for(std::size_t i = 0; i < path.size() - 1; ++i)
        {
            auto* edgePtr = mGraph.GetEdge(path.at(i), path.at(i + 1));
            assert(edgePtr != nullptr);
            cost = cost + mEdgeCostFunctor(edgePtr->Source(), edgePtr->Id(), edgePtr->Sink());
        }
        return cost;
    }
};

// example of A* implementation
template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
class AStar
{
public:
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    AStar(TGraph& graph, TEdgeCostFunctor edgeCost, THeuristicFunctor heuristic)
        : mGraph(graph)
        , mEdgeCostFunctor(edgeCost)
        , mHeuristicFunctor(heuristic)
        , mOutIterator(graph)
    { }

    void Update() { mOutIterator.Update(); }

private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor; // defines the exact cost of going over a certain edge
    THeuristicFunctor mHeuristicFunctor; // defines the expected cost of going from a to b
    OutIterator<TGraph> mOutIterator;

    // find out what the return type of the lambda is
    using CostType = decltype(mEdgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{}));

    struct ExploredObject
    {
        VertexIdType vertex{};
        CostType f{std::numeric_limits<CostType>::max()};
        CostType g{std::numeric_limits<CostType>::max()};
    };

public:
    std::vector<VertexIdType> Calculate(const VertexIdType start, const VertexIdType target)
    {
        std::map<VertexIdType, ExploredObject> exploredSet;
        std::vector<VertexIdType> heap;
        std::set<VertexIdType> closedSet;

        heap.push_back(start);
        exploredSet[start] = {start, 0., 0.};

        auto compareF = [&](const VertexIdType v1, const VertexIdType v2) {
            return exploredSet[v1].f > exploredSet[v2].f; //
        };

        VertexIdType current;
        while(heap.size() > 0)
        {
            // pick the lowest value out of heap
            std::pop_heap(heap.begin(), heap.end(), compareF);
            current = heap.back();
            heap.pop_back();

            if(current == target)
                break;
            if(closedSet.find(current) != closedSet.end())
                continue;
            closedSet.insert(current);

            // iterate over neighbours, check with best value so far
            for(const auto& [edgeId, otherVertexId] : mOutIterator.OutEdgeVertices(current))
            {
                const auto& edge = mGraph.GetEdge(edgeId);
                const auto edgeCost = mEdgeCostFunctor(current, edgeId, otherVertexId);
                const auto newGScore = exploredSet[current].g + edgeCost;

                auto& item = exploredSet[otherVertexId]; // adds item if it did not exist

                if(newGScore < item.g)
                {
                    const CostType hscore = mHeuristicFunctor(otherVertexId, target);
                    item = {current, newGScore + hscore, newGScore};

                    // put the new vertex in the heap

                    heap.push_back(otherVertexId);
                    std::push_heap(heap.begin(), heap.end(), compareF);
                }
            }
        }

        std::vector<VertexIdType> path;
        path.push_back(current);
        while(true)
        {
            if(path.back() == start)
                break;
            auto next = exploredSet[path.back()].vertex;
            path.push_back(next);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    CostType GetCost(const std::vector<VertexIdType>& path)
    {
        CostType cost = 0.;
        int i = 0;
        while(true)
        {
            if(i >= int(path.size()) - 1)
                break;

            auto* edgePtr = mGraph.GetEdge(path.at(i), path.at(i + 1));
            if(edgePtr == nullptr)
                break;
            cost = cost + mEdgeCostFunctor(edgePtr->Source(), edgePtr->Id(), edgePtr->Sink());
            ++i;
        }
        return cost;
    }
};

} // namespace algorithms
} // namespace graph
} // namespace vic