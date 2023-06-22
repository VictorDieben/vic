#pragma once

#include "vic/graph/graph_types/base.h"
#include "vic/graph/traits.h"

#include <vector>

namespace vic
{
namespace graph
{

template <typename TVertex, typename TEdge>
    requires ConceptVertex<TVertex> && ConceptEdge<TEdge>
class AdjecencyGraph
{
public:
    using VertexType = TVertex;
    using VertexIdType = typename TVertex::VertexIdType;
    using EdgeType = TEdge;
    using EdgeIdType = typename TEdge::EdgeIdType;

    static_assert(std::is_same_v<VertexType::VertexIdType, EdgeType::VertexIdType>);

    AdjecencyGraph() = default;

    std::size_t GetNumVertices() const { return mVertices.size(); }
    std::size_t GetNumEdges() const { return mEdges.size(); }

    VertexType& GetVertex(const VertexIdType id) { return mVertices.at(id); }
    EdgeType& GetEdge(const EdgeIdType id) { return mEdges.at(id); }
    EdgeType& GetEdge(const VertexIdType id1, const VertexIdType id2) { return mEdges.at(0); }

    // non-ConceptGraph

private:
    std::vector<VertexType> mVertices;
    std::vector<EdgeType> mEdges;
};

} // namespace graph
} // namespace vic