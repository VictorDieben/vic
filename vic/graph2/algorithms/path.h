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

template <typename TGraph, typename TEdgeCostFunctor>
auto PathCost(const TGraph& graph, //
              TEdgeCostFunctor edgeCostFunctor,
              const std::vector<typename TGraph::VertexIdType>& path)
{
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;
    using CostType = decltype(edgeCostFunctor(VertexIdType{}, VertexIdType{}));

    if(path.empty())
        return CostType{0.};

    CostType cost = 0.;
    for(std::size_t i = 0; i < path.size() - 1; ++i)
        cost = cost + edgeCostFunctor(path.at(i), //
                                      path.at(i + 1)); // todo: decide how we want to compute the edge cost (using edge id?)
    return cost;
}

} // namespace graph2
} // namespace vic