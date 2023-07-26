#pragma once

#include "vic/graph/definitions.h"
#include "vic/graph/traits.h"
#include <cstdint>

namespace vic
{
namespace graph
{

template <typename TVertex>
struct BaseVertex
{
    using VertexIdType = vertex_index_t<TVertex>;
};

template <typename TVertex, typename TEdge>
struct BaseEdge
{
    using VertexIdType = vertex_index_t<TVertex>;
    using EdgeIdType = edge_index_t<TEdge>;
};

} // namespace graph
} // namespace vic