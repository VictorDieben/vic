#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include <algorithm>
#include <map>
#include <set>

namespace vic
{
namespace graph2
{

namespace detail
{
// todo: put  implementation here, where iterator object is passed as argument
}

template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
auto AStar(const TGraph& graph, //
           TEdgeCostFunctor edgeCostFunctor,
           THeuristicFunctor heuristicFunctor,
           const typename TGraph::VertexIdType start,
           const typename TGraph::VertexIdType target)
{
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{}));
    using HeuristicType = decltype(edgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{}));
    static_assert(std::is_same_v<CostType, HeuristicType>);

    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    OutIterator iterator{graph};

    struct ExploredObject
    {
        VertexIdType vertex{};
        CostType f{std::numeric_limits<CostType>::max()};
        CostType g{std::numeric_limits<CostType>::max()};
    };

    std::vector<VertexIdType> heap;
    heap.push_back(start);

    std::set<VertexIdType> closedSet;
    std::map<VertexIdType, ExploredObject> exploredMap;
    exploredMap[start] = ExploredObject{start, 0., 0.};

    // note: > because default make_heap behaviour is max heap for operator<
    const auto compareF = [&](const VertexIdType v1, const VertexIdType v2) { return exploredMap.at(v1).f > exploredMap.at(v2).f; };

    VertexIdType current = start;
    while(!heap.empty())
    {
        std::pop_heap(heap.begin(), heap.end(), compareF);
        current = heap.back();
        heap.pop_back();

        if(current == target)
            break;
        if(closedSet.contains(current))
            continue;
        closedSet.insert(current);

        // iterate over neighbours, check with best value so far
        for(const auto& [edgeId, otherVertexId] : iterator.OutEdgeVertices(current))
        {
            const auto edgeCost = mEdgeCostFunctor(current, edgeId, otherVertexId);
            const auto newGScore = exploredMap[current].g + edgeCost;

            auto& item = exploredMap[otherVertexId]; // adds item if it did not exist

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

    if(current != target)
        return std::vector<VertexIdType>{};

    std::vector<VertexIdType> path;
    path.push_back(target);
    while(path.back() != start && path.size() < graph.NumVertices())
    {
        if(path.back() == start)
            break;
        auto next = exploredMap[path.back()].vertex;
        path.push_back(next);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

template <typename TGraph, typename TEdgeCostFunctor>
auto PathCost(const TGraph& graph, //
              TEdgeCostFunctor edgeCostFunctor,
              const std::vector<typename TGraph::VertexIdType>& path)
{ }

} // namespace graph2
} // namespace vic