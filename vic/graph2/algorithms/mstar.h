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

public:
    SubsetOutIterator(const TGraph& graph, const TOutVertexIterator& outIterator)
        : mGraph(graph)
        , mOutIterator(outIterator)
    { }

    template <typename TFunctor>
    void ForeachOutVertex(const CartesianVertexType& from, //
                          const CartesianVertexType& policy,
                          const CollisionSet collisionSet,
                          TFunctor lambda) const
    {
        //lambda(from);
        //for(const auto& to : mOutVertices.at(from))
        //    lambda(to);
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