#pragma once

#include "graph.h"
#include <vector>

namespace vic
{
namespace graph
{

// wrapper for begin and end iterators
template <typename TIter>
class Iterable
{
public:
    Iterable(TIter begin, TIter end)
        : mBegin(begin)
        , mEnd(end)
    { }
    template <typename T>
    Iterable(T iterable)
        : mBegin(iterable.begin())
        , mEnd(iterable.end())
    { }
    auto begin() { return mBegin; }
    auto end() { return mEnd; }

private:
    TIter mBegin;
    TIter mEnd;
};

// iterate over all vertices.
// This is a separate object, because a tensor graph will not be as trivial
template <typename TGraph>
class VertexIterator
{
public:
    VertexIterator(const TGraph& graph)
        : mGraph(graph)
    { }
    // TODO: true iterator
    auto begin() { return mGraph.Vertices().begin(); }
    auto end() { return mGraph.Vertices().end(); }

private:
    const TGraph& mGraph;
};

// iterate over all vertices
template <typename TGraph>
class EdgeIterator
{
public:
    EdgeIterator(const TGraph& graph)
        : mGraph(graph)
    { }
    auto begin() { return mGraph.Edges().begin(); }
    auto end() { return mGraph.Edges().end(); }

private:
    const TGraph& mGraph;
};

// iterate over all out edges of a certain vertex
template <typename TGraph, bool directed = false>
class OutIterator
{
public:
    OutIterator(const TGraph& graph)
        : mGraph(graph)
    { }

    using EdgeIdType = TGraph::EdgeIdType;
    using VertexIdType = TGraph::VertexIdType;

    void Update()
    {
        mOutEdges.clear();
        mOutVertices.clear();
        mOutEdges.resize(mGraph.GetNumVertices());
        mOutVertices.resize(mGraph.GetNumVertices());
        mOutEdgeVertices.resize(mGraph.GetNumVertices());

        for(const auto& edge : EdgeIterator(mGraph))
        {
            mOutEdges[edge.Source()].emplace_back(edge.Id());
            mOutVertices[edge.Source()].emplace_back(edge.Sink());
            mOutEdgeVertices[edge.Source()].emplace_back(edge.Id(), edge.Sink());
            if constexpr(!directed)
            {
                mOutEdges[edge.Sink()].emplace_back(edge.Id());
                mOutVertices[edge.Sink()].emplace_back(edge.Source());
                mOutEdgeVertices[edge.Sink()].emplace_back(edge.Id(), edge.Source());
            }
        }
    }

    // todo: maybe we don't want to return a reference to the vector itself
    const std::vector<EdgeIdType>& OutEdges(const VertexIdType id) const
    {
        assert(mGraph.GetNumVertices() == mOutEdges.size());
        return mOutEdges.at(id);
    }

    const std::vector<VertexIdType>& OutVertices(const VertexIdType id) const
    {
        assert(mGraph.GetNumVertices() == mOutVertices.size());
        return mOutVertices.at(id);
    }

    const std::vector<std::pair<EdgeIdType, VertexIdType>>& OutEdgeVertices(const VertexIdType id) const
    {
        assert(mGraph.GetNumVertices() == mOutEdgeVertices.size());
        return mOutEdgeVertices.at(id);
    }

private:
    const TGraph& mGraph;
    std::vector<std::vector<EdgeIdType>> mOutEdges{};
    std::vector<std::vector<VertexIdType>> mOutVertices{};
    std::vector<std::vector<std::pair<EdgeIdType, VertexIdType>>> mOutEdgeVertices{};
};

//// iterate over all out edges, for tensor vertices
//template <typename TGraph, bool directed = false>
//class TensorOutIterator
//{ };
//
//// iterate over all out edges, for tensor vertices,
//// such that no two agents are at the same spot,
//// and the transition is valid
//template <typename TGraph, bool directed = false>
//class TensorUniqueOutIterator
//{ };

} // namespace graph
} // namespace vic