#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include <algorithm>
#include <vector>

namespace vic
{
namespace graph2
{
template <typename CostType, typename VertexIdType>
struct FWData
{
    CostType cost{std::numeric_limits<CostType>::max()};
    VertexIdType next{0};
};

template <typename CostType, typename VertexIdType>
using FWMatrix = std::vector<std::vector<FWData<CostType, VertexIdType>>>;

template <typename TGraph, typename TEdgeCostFunctor, bool directed = false>
auto FloydWarshall(const TGraph& graph, //
                   TEdgeCostFunctor edgeCostFunctor)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, VertexIdType{}));

    using Data = FWData<CostType, VertexIdType>;

    const auto n = graph.NumVertices();

    const std::vector<Data> tmp{n, Data{}};
    FWMatrix<CostType, VertexIdType> matrix{n, tmp};

    const auto vertices = VertexIterator(graph);

    for(const auto i : vertices)
        matrix[i][i] = Data{0., i};

    for(const auto& edge : EdgeIterator(graph))
    {
        const auto edgeCost = edgeCostFunctor(edge.first, edge.second);
        matrix[edge.first][edge.second] = Data{edgeCost, edge.second};
        if constexpr(!directed)
            matrix[edge.second][edge.first] = Data{edgeCost, edge.first};
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
                    matrix[vi][vj] = Data{sum_ik_kj, matrix[vi][vk].next};
                }
            }
        }
    }

    return matrix;
}

// todo: higher order FW, where we store a policy for two robots driving their own path

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

template <typename TVertexId, typename FWData, bool shareHeadTail = false>
bool PolicyPathsConflict(const FWData& fw, //
                         const TVertexId start1,
                         const TVertexId target1,
                         const TVertexId start2,
                         const TVertexId target2)
{
    TVertexId current1 = start1;
    TVertexId current2 = start2;

    if(current1 == current2)
        return true;

    while(current1 != target1 || current2 != target2)
    {
        const TVertexId next1 = fw[current1][target1].next;
        const TVertexId next2 = fw[current2][target2].next;

        // check for conflict, if found, return true
        if constexpr(shareHeadTail)
        {
            if(next1 == next2 || (next1 == current2 && //
                                  next2 == current1))
                return true;
        }
        else
        {
            if(next1 == current2 || (next2 == current1 || //
                                     next1 == next2))
                return true;
        }

        // increase current values
        current1 = next1;
        current2 = next2;
    }

    return false; // todo
}

template <typename TPath, bool shareHeadTail = false>
bool PathsConflict(const TPath& path1, const TPath& path2)
{
    auto it1 = path1.begin();
    auto it2 = path2.begin();

    while(true)
    {
        auto next1 = std::next(it1);
        auto next2 = std::next(it2);

        if(next1 == path1.end() && next2 == path2.end())
            break;

        if(next1 == path1.end())
            next1 = it1;
        else if(next2 == path2.end())
            next2 = it2;

        if constexpr(shareHeadTail)
        {
            if(*next1 == *next2 || (*next1 == *it2 && //
                                    *next2 == *it1))
                return true;
        }
        else
        {
            if(*next1 == *it2 || (*next2 == *it1 || //
                                  *next1 == *next2))
                return true;
        }

        it1 = next1;
        it2 = next2;
    }
    return false;
}

} // namespace graph2
} // namespace vic