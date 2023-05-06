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

// Calculate the matrix of shortest distances and shortest paths
// Can be used by other algorithms for policy.
template <typename TGraph, typename TEdgeCostFunctor, bool directed = false>
class FloydWarshall
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;
    using EdgeType = typename TGraph::EdgeType;

    FloydWarshall(TGraph& graph, TEdgeCostFunctor functor)
        : mGraph(graph)
        , mEdgeCostFunctor(functor)
    { }
};

} // namespace algorithms
} // namespace graph
} // namespace vic