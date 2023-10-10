#pragma once

#include <vector>

namespace vic
{
namespace graph2
{

// barebones graph type, for functional stype algorithms
template <typename TVertexId, typename TEdgeId>
class Graph
{
public:
    using VertexIdType = typename TVertexId;
    using EdgeIdType = typename TEdgeId;

    using EdgeType = std::pair<VertexIdType, VertexIdType>;

    Graph(const std::size_t numVertices, const std::vector<EdgeType>& edges)
        : mNumVertices(numVertices)
        , mEdges(edges)
    {
        for(const auto& edge : mEdges)
            if(edge.first >= mNumVertices || edge.second >= mNumVertices)
                throw std::runtime_error("invalid edge verterx!");
    }

    std::size_t NumVertices() const { return mNumVertices; }
    std::size_t NumEdges() const { return mEdges.size(); }

    std::vector<EdgeType>& Edges() { return mEdges; }
    const std::vector<EdgeType>& Edges() const { return mEdges; }

    EdgeIdType GetEdgeId(const EdgeType& edge) const { return (EdgeIdType)(&edge - mEdges.data()); }
    EdgeIdType GetEdgeId(const VertexIdType from, const VertexIdType to) const
    {
        for(EdgeIdType id = 0; id < mEdges.size(); ++id)
            if(mEdges.at(id) == std::pair(from, to) || mEdges.at(id) == std::pair(to, from))
                return id;
        throw std::runtime_error("could not find edge!");
    }

    EdgeType& GetEdge(const VertexIdType from, const VertexIdType to)
    {
        for(auto& edge : mEdges)
            if(edge == std::pair(from, to) || edge == std::pair(to, from))
                return edge;
        throw std::runtime_error("could not find edge!");
    }
    EdgeType& GetEdge(const EdgeIdType id) { return mEdges.at(id); }

private:
    std::size_t mNumVertices;
    std::vector<EdgeType> mEdges;
};

} // namespace graph2
} // namespace vic