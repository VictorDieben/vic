#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include "vic/memory/flat_set.h"

#include <algorithm>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

namespace vic
{
namespace graph2
{

using CollisionSet = uint64_t;

inline constexpr void SetNthBit(CollisionSet& collisionset, uint8_t index)
{
    collisionset |= ((uint64_t)1) << index; //
}

inline constexpr bool NthBitIsSet(const CollisionSet collisionset, const uint8_t index)
{
    const CollisionSet mask = ((uint64_t)1) << index;
    return collisionset & mask;
}

template <typename TGraph, typename TOutVertexIterator>
class SubsetOutIterator
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using Graph = TGraph;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexIdType>;
    using CartesianEdgeType = std::vector<EdgeIdType>;

private:
    const Graph& mGraph;
    const TOutVertexIterator mOutIterator; // todo: constrain with concept

    using OccupiedSetType = vic::memory::UnorderedFlatSet<VertexIdType>;

public:
    SubsetOutIterator(const TGraph& graph, const TOutVertexIterator& outIterator)
        : mGraph(graph)
        , mOutIterator(outIterator)
    { }

    template <typename TFunctor>
    void ForeachOutVertex(const CartesianVertexType& from, //
                          const CartesianVertexType& policy,
                          const CollisionSet collisionSet,
                          TFunctor functor) const
    {
        auto copy = from;
        OccupiedSetType occupiedSet(from.begin(), from.end());
        ForeachOutVertexRecursive(copy, occupiedSet, policy, collisionSet, 0, from.size(), functor);
    }

private:
    template <typename TFunctor>
    void ForeachOutVertexRecursive(CartesianVertexType& vertex, //
                                   OccupiedSetType& occupiedVertices,
                                   const CartesianVertexType& policy,
                                   const CollisionSet collisionSet,
                                   const std::size_t dim,
                                   const std::size_t dims,
                                   TFunctor functor) const
    {
        if(dim == dims)
        {
            functor(vertex);
            return;
        }
        const VertexIdType vertexAtDim = vertex.at(dim);

        if(!NthBitIsSet(collisionSet, dim)) // agent not in collision, step in policy direction
        {
            if(occupiedVertices.insert(policy.at(dim)).second)
            {
                vertex.at(dim) = policy.at(dim);
                ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);
                vertex.at(dim) = vertexAtDim;
                occupiedVertices.pop_back();
            }
        }
        else // agent in collision, fully expand this dimension
        {
            ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);

            for(const auto& outVert : mOutIterator.OutVertices(vertex.at(dim)))
            {
                if(occupiedVertices.insert(outVert).second)
                {
                    vertex.at(dim) = outVert;
                    ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);
                    occupiedVertices.pop_back();
                }
            }
            vertex.at(dim) = vertexAtDim;
        }
    }
};

template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
auto MStar(const TGraph& graph, //
           TEdgeCostFunctor edgeCostFunctor,
           THeuristicFunctor heuristicFunctor,
           const typename TGraph::VertexIdType start,
           const typename TGraph::VertexIdType target)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    using CostType = double;

    // todo: make this a function argument
    const auto outIterator = SubsetOutIterator(graph);

    const auto subsetIterator = SubsetOutIterator(graph, outIterator);

    struct ExploredObject
    {
        CollisionSet collisionSet{0};
        CostType cost{std::numeric_limits<CostType>::max()};
        VertexIdType previous{};
    };

    std::vector<VertexIdType> heap;
    heap.push_back(start);

    std::set<VertexIdType> closedSet;
    std::map<VertexIdType, ExploredObject> exploredMap;
    exploredMap[start] = ExploredObject{{}, 0, start};

    // note: > because default make_heap behaviour is max heap for operator<
    const auto compareCost = [&](const VertexIdType v1, const VertexIdType v2) { return exploredMap.at(v1).cost > exploredMap.at(v2).cost; };

    VertexIdType current = start;

    while(!heap.empty())
    {
        std::pop_heap(heap.begin(), heap.end(), compareCost);
        current = heap.back();
        heap.pop_back();

        if(current == target)
            break;
        if(closedSet.contains(current))
            continue;
        closedSet.insert(current);

        //subsetIterator.ForeachValidOut(current, [&](const VertexIdType other) {
        //    //
        //});
    }

    return 1; // todo
}

} // namespace graph2
} // namespace vic