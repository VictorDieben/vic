#pragma once

#include <tuple>
#include <vector>

#include "vic/graph2/traits.h"
#include "vic/memory/flat_set.h"

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

template <typename TGraph>
class BaseOutVertexIterator
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    BaseOutVertexIterator(const TGraph& graph) { Update(graph); }

    void Update(const TGraph& graph)
    {
        mOutVertices.clear();
        mOutVertices.resize(graph.NumVertices());
        for(const auto& edge : graph.Edges())
        {
            mOutVertices[edge.first].emplace_back(edge.second);
            // if constexpr(!directed)
            mOutVertices[edge.second].emplace_back(edge.first);
        }
    }

    const std::vector<VertexIdType>& OutVertices(const VertexIdType id) const { return mOutVertices.at(id); }

    template <typename TFunctor>
    void ForeachOutVertex(const VertexIdType from, TFunctor lambda) const
    {
        lambda(from);
        for(const auto& to : mOutVertices.at(from))
            lambda(to);
    }

private:
    std::vector<std::vector<VertexIdType>> mOutVertices{};
};

template <typename TGraph>
class CartesianOutVertexIterator
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    CartesianOutVertexIterator(const TGraph& graph)
        : mGraph(graph)
    { }

    template <typename TFunctor>
    void ForeachOutVertex(const VertexIdType from, TFunctor lambda) const
    {
        //for(const auto& to : mOutVertices.at(from))
        //    lambda(to);
    }

private:
    const TGraph& mGraph;
};

template <typename TGraph>
auto GetOutVertexIterator(const TGraph& graph)
{
    if constexpr(ConceptGraphCartesian<TGraph>)
        return CartesianOutVertexIterator(graph);
    else if constexpr(ConceptGraph<TGraph>)
        return BaseOutVertexIterator(graph);
    else
        throw std::runtime_error("No out vertex iterator available for this type of graph!");
}

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

template <typename TOutVertexIterator>
class CartesianOutIterator
{
public:
    using VertexIdType = uint16_t;
    using EdgeIdType = uint16_t;

    // todo: this might need to be input
    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexIdType>;
    using CartesianEdgeType = std::vector<EdgeIdType>;

private:
    const TOutVertexIterator& mOutIterator; // todo: constrain with concept

public:
    CartesianOutIterator(const TOutVertexIterator& outIterator)
        : mOutIterator(outIterator)
    { }

    template <typename TFunctor>
    void ForeachOutVertex(const CartesianVertexType& vert, TFunctor functor) const
    {
        auto buffer = vert;
        ForeachOutRecursive(buffer, functor, 0, vert.size());
    }

    template <typename TFunctor>
    void ForeachValidOutVertex(const CartesianVertexType& vert, TFunctor functor) const
    {
        auto buffer = vert;
        vic::memory::UnorderedFlatSet<VertexIdType> occupiedVertices(vert.begin(), vert.end());
        if(occupiedVertices.size() < vert.size())
            return; // start vertex is already in collision
        ForeachValidOutVertexRecursive(buffer, functor, 0, vert.size(), occupiedVertices);
    }

private:
    template <typename TFunctor>
    void ForeachOutRecursive(CartesianVertexType& vertex,
                             TFunctor functor, //
                             const std::size_t dim,
                             const std::size_t dims) const
    {
        if(dim == dims)
        {
            functor(vertex);
            return;
        }
        const VertexIdType vertexAtDim = vertex[dim];

        // loop over case where this dimension is constant
        ForeachOutRecursive(vertex, functor, dim + 1, dims);

        // todo: loop over all out vertices for this dimension
        const auto& outVerts = mOutIterator.OutVertices(vertexAtDim);
        for(const auto& outVert : outVerts)
        {
            vertex[dim] = outVert;
            ForeachOutRecursive(vertex, functor, dim + 1, dims);
        }
        vertex[dim] = vertexAtDim;
    }

    template <typename TFunctor>
    void ForeachValidOutVertexRecursive(CartesianVertexType& vertex,
                                        TFunctor functor, //
                                        const std::size_t dim,
                                        const std::size_t dims,
                                        vic::memory::UnorderedFlatSet<VertexIdType>& occupiedVertices) const
    {
        if(dim == dims)
        {
            functor(vertex);
            return;
        }
        const VertexIdType vertexAtDim = vertex[dim];

        // loop over case where this dimension is constant
        ForeachValidOutVertexRecursive(vertex, functor, dim + 1, dims, occupiedVertices);

        // loop over all out vertices for this dimension
        const auto& outVerts = mOutIterator.OutVertices(vertex.at(dim));
        for(const auto& outVert : outVerts)
        {
            if(occupiedVertices.insert(outVert).second)
            {
                vertex[dim] = outVert;
                ForeachValidOutVertexRecursive(vertex, functor, dim + 1, dims, occupiedVertices);
                occupiedVertices.pop_back();
                // occupiedVertices.erase(outVert); // for normal set
            }
        }
        vertex[dim] = vertexAtDim;
    }
};

} // namespace graph2
} // namespace vic