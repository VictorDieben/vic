#pragma once

#include "vic/graph/graph.h"
#include "vic/graph/traits.h"
#include "vic/utils.h"

#include <array>
#include <ranges>
#include <vector>
namespace vic
{
namespace graph
{

// todo(vicdie): a tensor graph is a wrapper around a normal graph, to support path planning with multiple robots
// with such a large graph, data storage becomes critical,
// so a lot of work is put into storing it as efficiently as possible

// A Tensor vertex is a vertex that represents the position of multiple agents
// at the same time. In order to store it as compactly as possible, the list of
// vertex ids (e.g. {1, 10, 50, 23}) is translated to a number with base equal
// to the number of vertices in the graph.

using TensorVertexId = Uint128;
using TensorEdgeId = std::pair<TensorVertexId, TensorVertexId>;

template <typename TVertex>
class TensorVertex
{
public:
    using VertexType = TVertex;
    using VertexIdType = typename TVertex::VertexIdType;

    TensorVertex() = default;

    template <typename TGraph>
    TensorVertex(const TGraph& graph, const TensorVertexId& id)
    {
        FromId(graph, id);
    }

    template <typename TGraph>
    TensorVertex(const TGraph& graph, const std::vector<VertexIdType>& verts)
        : mVertices(verts)
    { }

    template <typename TGraph>
    TensorVertexId FromVertexIds(const TGraph& graph, const std::vector<VertexIdType>& verts) const
    {
        // TODO(vicdie): verify
        mVertices = verts;
    }

    template <typename TGraph>
    TensorVertexId ToId(const TGraph& graph) const
    {
        return FromBase<TensorVertexId>(mVertices, graph.NumVertices());
    }

    template <typename TGraph>
    void FromId(const TGraph& graph, const TensorVertexId id)
    {
        ToBase<VertexIdType>(id, graph.NumVertices(), mVertices);
        mVertices.resize(graph.GetDimensions());
    }

    const std::vector<VertexIdType>& GetVertices() const { return mVertices; }

private:
    std::vector<VertexIdType> mVertices{};
};

template <typename TGraph>
requires ConceptGraph<TGraph>
class TensorGraph
{
public:
    using GraphType = TGraph;
    using VertexType = typename TGraph::VertexType;
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeType = typename TGraph::EdgeType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using TensorVertexType = TensorVertex<VertexType>;
    using TensorVertexIdType = TensorVertexId;
    using TensorEdgeIdType = void;

    TensorGraph(TGraph& graph)
        : mGraph(graph)
    { }

    GraphType& GetGraph() { return mGraph; }
    void SetDimensions(Uint dims) { mDimensions = dims; }
    Uint GetDimensions() const { return mDimensions; }
    std::size_t NumVertices() const { return mGraph.GetNumVertices(); }
    Uint128 NumTensorVertices() const { return Power<Uint128, std::size_t, Uint>(mGraph.GetNumVertices(), mDimensions); }

private:
    Uint mDimensions{1};
    GraphType& mGraph;
};

// example of A* implementation
template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
requires ConceptTensorGraph<TGraph>
class TensorAStar
{
public:
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    TensorAStar(TGraph& graph, TEdgeCostFunctor edgeCost, THeuristicFunctor heuristic)
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

} // namespace graph
} // namespace vic