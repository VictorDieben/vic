#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

namespace vic
{
namespace graph
{
namespace algorithms
{

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
};

} // namespace algorithms
} // namespace graph
} // namespace vic