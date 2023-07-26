#pragma once

#include "vic/utils.h"
#include <array>
#include <vector>

#include "definitions.h"

namespace std
{
// hash function overload
template <>
struct ::std::hash<uint64_t>
{
    inline uint64_t operator()(const uint64_t& x) const noexcept { return static_cast<uint64_t>(x); }
};
} // namespace std

namespace vic
{
namespace graph
{

struct EmptyVertexDataType
{
    using VertexIdType = uint16_t;
};

struct EmptyEdgeDataType
{
    using VertexIdType = uint16_t;
    using EdgeIdType = uint16_t;
};

template <typename TData = EmptyVertexDataType>
struct Vertex
{
    using VertexDataType = TData;
    using VertexIdType = typename TData::VertexIdType;
    constexpr Vertex() = default;
    constexpr Vertex(const VertexIdType id)
        : mId(id)
    { }
    constexpr Vertex(const VertexIdType id, const VertexDataType& data)
        : mId(id)
        , mData(data)
    { }
    const VertexIdType Id() const { return mId; }
    VertexDataType& Data() { return mData; }
    VertexIdType mId{};
    VertexDataType mData{};
};

template <typename TData = EmptyEdgeDataType>
struct Edge
{
    using EdgeDataType = TData;
    using EdgeIdType = typename EmptyEdgeDataType::EdgeIdType;
    using VertexIdType = EdgeIdType; // todo: decide if we want to just store the same type

    Edge() = default;
    Edge(const VertexIdType source, const VertexIdType sink, const EdgeIdType id)
        : mSource(source)
        , mSink(sink)
        , mId(id)
    { }
    Edge(const VertexIdType source, const VertexIdType sink, const EdgeIdType id, const EdgeDataType& data)
        : mSource(source)
        , mSink(sink)
        , mId(id)
        , mData(data)
    { }

    const EdgeIdType Id() const { return mId; }
    EdgeDataType& Data() { return mData; }
    VertexIdType Source() const { return mSource; }
    VertexIdType Sink() const { return mSink; }

    VertexIdType mSource{};
    VertexIdType mSink{};
    EdgeIdType mId{};
    EdgeDataType mData{};
};

// todo: make a Graph concept (c++20)
template <typename TVertex, typename TEdge>
class BaseGraph
{
public:
    using VertexType = TVertex;
    using EdgeType = TEdge;
    using VertexIdType = typename TVertex::VertexIdType;
    using EdgeIdType = typename TEdge::EdgeIdType;

    BaseGraph() = default;

    VertexType& AddVertex()
    {
        mVertices.emplace_back(mVertexIdCounter);
        mVertexIdCounter++;
        return mVertices.back();
    }

    VertexType& AddVertex(const typename VertexType::VertexDataType& data)
    {
        mVertices.emplace_back(mVertexIdCounter);
        mVertices.back().mData = data;
        mVertexIdCounter++;
        return mVertices.back();
    }

    EdgeType& AddEdge(VertexIdType source, VertexIdType sink)
    {
        mEdges.emplace_back(source, sink, mEdgeIdCounter);
        mEdgeIdCounter++;
        return mEdges.back();
    }
    VertexType& GetVertex(const VertexIdType id) { return mVertices.at(id); } // id is also index
    EdgeType& GetEdge(const EdgeIdType id) { return mEdges.at(id); } // id is also index
    EdgeType* GetEdge(const VertexIdType source, const VertexIdType sink)
    {
        for(auto& edge : mEdges)
        {
            if(edge.Source() == source && edge.Sink() == sink)
                return &edge;
            // todo: check if graph is directed
            if(edge.Source() == sink && edge.Sink() == source)
                return &edge;
        }
        return nullptr;
    }

    std::size_t GetNumVertices() const { return mVertices.size(); }
    std::size_t GetNumEdges() const { return mEdges.size(); }

    // todo: iterators
    std::vector<VertexType>& Vertices() { return mVertices; }
    std::vector<EdgeType>& Edges() { return mEdges; }

private:
    std::vector<VertexType> mVertices{};
    std::vector<EdgeType> mEdges{};

    VertexIdType mVertexIdCounter{0};
    EdgeIdType mEdgeIdCounter{0};
};

} // namespace graph
} // namespace vic