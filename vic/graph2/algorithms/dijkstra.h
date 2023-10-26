#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include <algorithm>

namespace vic
{
namespace graph2
{

namespace detail
{
// todo: put Dijkstra implementation here, where iterator object is passed as argument
}

template <typename TGraph, typename TEdgeCostFunctor>
auto Dijkstra(const TGraph& graph, //
              TEdgeCostFunctor edgeCostFunctor,
              const typename TGraph::VertexIdType start,
              const typename TGraph::VertexIdType target)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, VertexIdType{})); // return type of lambda

    const auto outIterator = GetOutVertexIterator(graph);

    struct VertexData
    {
        CostType lowestCost{std::numeric_limits<CostType>::max()};
        bool visited{false};
        VertexIdType previous{0};
    };

    std::vector<VertexData> vertexData{graph.NumVertices(), VertexData{}};

    std::vector<VertexIdType> heap;

    // note: > because default make_heap behaviour is max heap for operator<
    const auto compare = [&](const VertexIdType& v1, const VertexIdType& v2) { return vertexData.at(v1).lowestCost > vertexData.at(v2).lowestCost; };

    // add the initial starting point
    heap.push_back(start);
    vertexData.at(start) = {0., false, start};

    VertexIdType current = start;
    while(!heap.empty())
    {
        // pick the lowest value out of heap
        std::pop_heap(heap.begin(), heap.end(), compare);
        current = heap.back();
        heap.pop_back();

        if(vertexData.at(current).visited)
            continue; // already explored this vertex

        if(current == target)
            break; // we arrived at the target vertex

        vertexData.at(current).visited = true;

        outIterator.ForeachOutVertex(current, [&](const VertexIdType other) {
            const auto edgeCost = edgeCostFunctor(current, other); // todo: remove edge id from cost functor
            const auto newCost = vertexData.at(current).lowestCost + edgeCost;

            if(newCost < vertexData.at(other).lowestCost)
            {
                vertexData.at(other) = {newCost, false, current};
                heap.push_back(other);
                std::push_heap(heap.begin(), heap.end(), compare);
            }
        });
    }

    if(current != target)
        return std::vector<VertexIdType>{};

    // backtrack
    std::vector<VertexIdType> path;
    path.push_back(target);
    while(path.back() != start && path.size() <= graph.NumVertices())
        path.push_back(vertexData.at(path.back()).previous);

    std::reverse(path.begin(), path.end());

    return path;
}

} // namespace graph2
} // namespace vic