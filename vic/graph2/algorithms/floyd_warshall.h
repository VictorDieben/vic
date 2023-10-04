#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include <algorithm>
#include <vector>

namespace vic
{
namespace graph2
{

template <typename TGraph, typename TEdgeCostFunctor, bool directed = false>
auto FloydWarshall(const TGraph& graph, //
                   TEdgeCostFunctor edgeCostFunctor)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;
    using EdgeType = typename GraphType::EdgeType;
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, EdgeIdType{}, VertexIdType{})); // return type of lambda

    const auto n = graph.NumVertices();

    struct FWData
    {
        CostType cost{std::numeric_limits<CostType>::max()};
        VertexIdType next{0};
    };

    const std::vector<FWData> tmp{n, FWData{}};
    std::vector<std::vector<FWData>> matrix{n, tmp};

    const auto vertices = VertexIterator(graph);

    for(const auto i : vertices)
        matrix[i][i] = FWData{0., i};

    for(const auto& edge : EdgeIterator(graph))
    {
        const auto edgeId = graph.GetEdgeId(edge);
        const auto edgeCost = edgeCostFunctor(edge.first, edgeId, edge.second);
        matrix[edge.first][edge.second] = FWData{edgeCost, edge.second};
        if constexpr(!directed)
            matrix[edge.second][edge.first] = FWData{edgeCost, edge.first};
    }

    for(const auto& vk : vertices)
    {
        for(const auto& vi : vertices)
        {
            for(const auto& vj : vertices)
            {
                const double sum_ik_kj = matrix[vi][vk].cost + matrix[vk][vj].cost;
                if(matrix[vi][vj].cost > sum_ik_kj)
                {
                    matrix[vi][vj] = FWData{sum_ik_kj, matrix[vi][vk].next};
                }
            }
        }
    }

    return matrix;
}

template <typename TGraph, typename TData>
auto PolicyPath(const TGraph& graph, //
                const std::vector<std::vector<TData>>& fw,
                const typename TGraph::VertexIdType start,
                const typename TGraph::VertexIdType target)
{
    std::vector<typename TGraph::VertexIdType> path;

    path.push_back(start);
    while(path.back() != target && path.size() < graph.NumVertices())
        path.push_back(fw[path.back()][target].next);

    return path;
}

} // namespace graph2
} // namespace vic