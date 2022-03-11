#pragma once

#include <array>
#include <vector>
#include "vic/utils.h"

namespace std
{
// hash function overload
template <>
struct ::std::hash<uint64_t>
{
    inline uint64_t operator()(const uint64_t& x) const noexcept
    {
        return static_cast<uint64_t>(x);
    }
};
}

namespace vic
{
namespace graph
{

using Uint = unsigned int;
using Uint128 = uint64_t; // TODO(vicdie): enable in vs
using Uint256 = uint64_t;
using Uint512 = uint64_t;
using Uint1024 = uint64_t;

struct EmptyVertexDataType
{
    using VertexIdType = uint16_t;
};

struct EmptyEdgeDataType
{
    using EdgeIdType = uint16_t;
};

template <typename TData = EmptyVertexDataType>
struct Vertex
{
    using VertexDataType = TData;
    using VertexIdType = typename TData::VertexIdType;
    constexpr Vertex() = default;
    constexpr Vertex(const VertexIdType id) : mId(id) {}
    constexpr Vertex(const VertexIdType id, const VertexDataType& data)
        : mId(id)
        , mData(data) {}
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
    using VertexIdType = EdgeIdType; // TODO(vicdie): decide if we want to just store the same type

    Edge() = default;
    Edge(const VertexIdType source, const VertexIdType sink, const EdgeIdType id)
        : mSource(source)
        , mSink(sink)
        , mId(id) {}
    Edge(const VertexIdType source, const VertexIdType sink, const EdgeIdType id, const EdgeDataType& data)
        : mSource(source)
        , mSink(sink)
        , mId(id)
        , mData(data) {}

    const EdgeIdType Id() const { return mId; }
    EdgeDataType& Data() { return mData; }
    VertexIdType Source() const { return mSource;  }
    VertexIdType Sink() const { return mSink; }

    VertexIdType mSource{};
    VertexIdType mSink{};
    EdgeIdType mId{};
    EdgeDataType mData{};
};

// TODO(vicdie): make a Graph concept (c++20)
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
    VertexType& GetVertex(const VertexIdType id) { return mVertices.at(id); }// id is also index
    EdgeType& GetEdge(const EdgeIdType id) { return mEdges.at(id); }// id is also index

    std::size_t GetNumVertices() const { return mVertices.size(); }
    std::size_t GetNumEdges() const { return mEdges.size(); }

    // todo: iterators
    std::vector<VertexType>& Vertices() { return mVertices; }
    std::vector<EdgeType>& Edges() { return mEdges; }
private:

    std::vector<VertexType> mVertices{};
    std::vector<EdgeType> mEdges{};

    VertexIdType mVertexIdCounter{ 0 };
    EdgeIdType mEdgeIdCounter{ 0 };
};


// A Tensor vertex is a vertex that represents the position of multiple agents at the same time.
// In order to store it as compactly as possible, the list of vertex ids (e.g. {1, 10, 50, 23}) 
// is translated to a number with base equal to the number of vertices in the graph.

template <typename TVertex>
class TensorVertex; // forward declare

class TensorVertexId
{
public:
    using TensorVertexIdType = Uint128;
    TensorVertexId() = default;
    TensorVertexId(const TensorVertexIdType id)
        : mId(id) {}

private:
    TensorVertexIdType mId{};
};

template <typename TVertex>
class TensorVertex
{
public:
    using VertexType = TVertex;
    using VertexIdType = typename TVertex::VertexIdType;
    using TensorVertexIdType = TensorVertexId; // TODO: fix

    TensorVertex() = default;
    TensorVertex(const TensorVertexId& id) { FromId(id); }

    TensorVertexId ToId() const { return {}; }
    void FromId(const TensorVertexId id)
    {
        // TODO: implement 
    }
private:

};

template <typename TGraph>
class TensorGraph
{
public:
    using GraphType = TGraph;
    using VertexType = typename TGraph::VertexType;
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeType = typename TGraph::EdgeType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    TensorGraph(TGraph& graph) : mGraph(graph) {}

    GraphType& GetGraph() { return mGraph; }
    void SetDimensions(Uint dims) { mDimensions = dims; }
    Uint GetDimensions() const { return mDimensions; }
    void NumTensorVertices() const { return 0; }

private:
    Uint mDimensions{ 1 };
    GraphType& mGraph;
};




}
}