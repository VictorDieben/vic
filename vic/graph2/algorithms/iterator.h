#pragma once

#include <tuple>
#include <vector>

namespace vic
{
namespace graph2
{
// note: some types of graphs will need specializations of these types

template <typename TGraph>
class VertexIterator
{
public:
    using VertexIdType = TGraph::VertexIdType;

    VertexIterator(const TGraph& graph)
        : mGraph(graph)
    {
        for(VertexIdType id = 0; id < graph.NumVertices(); ++id)
            mVertices.push_back(id);
    }

    auto begin() { return mVertices.begin(); }
    auto end() { return mVertices.end(); }

    auto begin() const { return mVertices.begin(); }
    auto end() const { return mVertices.end(); }

    auto cbegin() const { return mVertices.cbegin(); }
    auto cend() const { return mVertices.cend(); }

private:
    const TGraph& mGraph;
    std::vector<VertexIdType> mVertices;
};

template <typename TGraph>
class EdgeIterator
{
public:
    EdgeIterator(const TGraph& graph)
        : mGraph(graph)
    { }

    auto begin() const { return mGraph.Edges().begin(); }
    auto end() const { return mGraph.Edges().end(); }

    auto cbegin() const { return mGraph.Edges().cbegin(); }
    auto cend() const { return mGraph.Edges().cend(); }

private:
    const TGraph& mGraph;
};

template <typename TGraph, bool directed = false>
class OutIterator
{
public:
    using EdgeIdType = TGraph::EdgeIdType;
    using VertexIdType = TGraph::VertexIdType;

    OutIterator(const TGraph& graph)
        : mGraph(graph)
    {
        Update();
    }

    void Update()
    {
        mOutEdgeData.clear();
        mOutEdgeData.resize(mGraph.NumVertices());
        for(const auto& edge : mGraph.Edges())
        {
            mOutEdgeData[edge.first].emplace_back(mGraph.GetEdgeId(edge), edge.second);
            if constexpr(!directed)
                mOutEdgeData[edge.second].emplace_back(mGraph.GetEdgeId(edge), edge.first);
        }
    }

    const std::vector<std::pair<EdgeIdType, VertexIdType>>& OutEdgeVertices(const VertexIdType id) const
    {
        assert(mGraph.NumVertices() == mOutEdgeData.size());
        return mOutEdgeData.at(id);
    }

private:
    const TGraph& mGraph;
    std::vector<std::vector<std::pair<EdgeIdType, VertexIdType>>> mOutEdgeData{};
};
} // namespace graph2
} // namespace vic