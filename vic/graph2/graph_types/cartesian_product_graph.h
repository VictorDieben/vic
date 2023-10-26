#pragma once

#include "vic/graph2/graph_types/base_graph.h"

#include "vic/utils.h"

#include <vector>

namespace vic
{
namespace graph2
{

template <typename TBaseId, typename TCartesianId>
void ToVector(const TCartesianId cartesianId, //
              const uint32_t dimensions,
              const uint32_t size,
              std::vector<TBaseId>& buffer)
{
    buffer.clear();
    // todo: convert cartesianId to "dimension" of numbers in base "size"
    ToBase<TBaseId>(cartesianId, size, buffer);
    buffer.resize(dimensions);
}

template <typename TBaseId, typename TCartesianId>
std::vector<TBaseId> ToVector(const TCartesianId cartesianId, //
                              const uint32_t dimensions,
                              const uint32_t size)
{
    std::vector<TBaseId> buffer{dimensions, 0};
    ToVector(cartesianId, dimensions, size, buffer);
    return buffer;
}

template <typename TCartesianId, typename TBaseId>
TCartesianId ToId(const std::vector<TBaseId>& buffer, //
                  const uint32_t size)
{
    // todo: rewrite FromBase to do a fixed number of iterations, equal to size
    return FromBase<TCartesianId>(buffer, size);
}

template <typename TGraph>
class CartesianGraph
{
public:
    using BaseGraphType = TGraph;
    using VertexIdType = uint64_t;
    using EdgeIdType = uint64_t;

    CartesianGraph(const TGraph& graph, uint32_t dimensions)
        : mGraph(graph)
        , mDimensions(dimensions)
    { }

    std::size_t NumVertices() const { return vic::Power<std::size_t>(mGraph.NumVertices(), mDimensions); }
    std::size_t NumEdges() const { return vic::Power<std::size_t>(mGraph.NumEdges(), mDimensions); }

    uint32_t NumDimensions() const { return mDimensions; }

    const TGraph& GetBaseGraph() const { return mGraph; }

private:
    const TGraph& mGraph;
    uint32_t mDimensions;
};
} // namespace graph2
} // namespace vic