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
    using EdgeType = typename GraphType::EdgeType;
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{})); // return type of lambda

    const OutIterator outIterator{graph};

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

        // iterate over neighbours, check with best value so far
        for(const auto& [edgeId, otherVertexId] : outIterator.OutEdgeVertices(current))
        {
            const auto edgeCost = edgeCostFunctor(current, edgeId, otherVertexId);
            const auto newCost = vertexData.at(current).lowestCost + edgeCost;

            if(newCost < vertexData.at(otherVertexId).lowestCost)
            {
                vertexData.at(otherVertexId) = {newCost, false, current};
                heap.push_back(otherVertexId);
                std::push_heap(heap.begin(), heap.end(), compare);
            }
        }
    }

    if(current != target)
        return std::vector<VertexIdType>{};

    // backtrack
    std::vector<VertexIdType> path;
    path.push_back(target);
    while(path.back() != start && path.size() < graph.NumVertices())
        path.push_back(vertexData.at(path.back()).previous);

    std::reverse(path.begin(), path.end());

    return path;
}

template <typename TGraph, typename TEdgeCostFunctor>
auto PathCost(const TGraph& graph, //
              TEdgeCostFunctor edgeCostFunctor,
              const std::vector<typename TGraph::VertexIdType>& path)
{
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{}));

    CostType cost = 0.;
    for(std::size_t i = 0; i < path.size() - 1; ++i)
        cost = cost + edgeCostFunctor(path.at(i), //
                                      graph.GetEdgeId(path.at(i), path.at(i + 1)),
                                      path.at(i + 1)); // todo: decide how we want to compute the edge cost (using edge id?)
    return cost;
}

} // namespace graph2
} // namespace vic