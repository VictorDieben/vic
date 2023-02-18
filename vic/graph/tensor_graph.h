#pragma once

#include "vic/graph/graph.h"
#include "vic/graph/traits.h"
#include "vic/utils.h"

#include <array>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vic
{
namespace graph
{

// todo: a tensor graph is a wrapper around a normal graph, to support path planning with multiple robots
// with such a large graph, data storage becomes critical,
// so a lot of work is put into storing it as efficiently as possible

// A Tensor vertex is a vertex that represents the position of multiple agents
// at the same time. In order to store it as compactly as possible, the list of
// vertex ids (e.g. {1, 10, 50, 23}) is translated to a number with base equal
// to the number of vertices in the graph.

using TensorVertexId = Uint128;
using TensorEdgeId = std::pair<TensorVertexId, TensorVertexId>;

template <typename TVertex>
class TensorVertex
{
public:
    using VertexType = TVertex;
    using VertexIdType = typename TVertex::VertexIdType;

    TensorVertex() = default;

    template <typename TGraph>
    TensorVertex(const TGraph& graph, const TensorVertexId& id)
    {
        FromId(graph, id);
    }

    template <typename TGraph>
    TensorVertex(const TGraph& graph, const std::vector<VertexIdType>& verts)
        : mVertices(verts)
    { }

    template <typename TGraph>
    TensorVertexId FromVertexIds(const TGraph& graph, const std::vector<VertexIdType>& verts) const
    {
        mVertices = verts;
    }

    template <typename TGraph>
    TensorVertexId ToId(const TGraph& graph) const
    {
        return FromBase<TensorVertexId>(mVertices, graph.NumVertices());
    }

    template <typename TGraph>
    void FromId(const TGraph& graph, const TensorVertexId id)
    {
        ToBase<VertexIdType>(id, graph.NumVertices(), mVertices);
        mVertices.resize(graph.GetDimensions());
    }

    const std::vector<VertexIdType>& GetVertices() const { return mVertices; }

    VertexIdType At(const std::size_t dim) const { return mVertices.at(dim); }
    void Set(const std::size_t dim, const VertexIdType value) { mVertices[dim] = value; }

private:
    std::vector<VertexIdType> mVertices{};
};

template <typename TGraph>
// requires ConceptGraph<TGraph>
class TensorGraph
{
public:
    using GraphType = TGraph;
    using VertexType = typename TGraph::VertexType;
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeType = typename TGraph::EdgeType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using TensorVertexType = TensorVertex<VertexType>;
    using TensorVertexIdType = TensorVertexId;
    using TensorEdgeIdType = void;

    TensorGraph(TGraph& graph)
        : mGraph(graph)
    { }

    GraphType& GetGraph() { return mGraph; }
    void SetDimensions(Uint dims) { mDimensions = dims; }
    Uint GetDimensions() const { return mDimensions; }
    std::size_t NumVertices() const { return mGraph.GetNumVertices(); }
    Uint128 NumTensorVertices() const { return Power<Uint128, std::size_t, Uint>(mGraph.GetNumVertices(), mDimensions); }

private:
    Uint mDimensions{1};
    GraphType& mGraph;
};

// iterate over all vertices.
// This is a separate object, because a tensor graph will not be as trivial
template <typename TTensorGraph>
class TensorOutIterator
{
public:
    using VertexType = typename TTensorGraph::VertexType;
    using EdgeType = typename TTensorGraph::EdgeType;
    using VertexIdType = typename TTensorGraph::VertexIdType;
    using EdgeIdType = typename TTensorGraph::EdgeIdType;

    using TensorVertexType = TensorVertex<VertexType>;

    using TensorGraphType = typename TTensorGraph;
    using GraphType = typename TTensorGraph::GraphType;

private:
    TTensorGraph& mGraph;
    OutIterator<GraphType> mOutIterator;

public:
    TensorOutIterator(TTensorGraph& graph)
        : mGraph(graph)
        , mOutIterator(graph.GetGraph())
    { }

    void Update()
    {
        //
        mOutIterator.Update();
    }

    template <typename TFunctor>
    void ForeachOut(const TensorVertexId id, TFunctor functor) const
    {
        ForeachOut(TensorVertexType(mGraph, id), functor);
    }

    template <typename TFunctor>
    void ForeachOut(const TensorVertexType& vert, TFunctor functor) const
    {
        TensorVertexType copy = vert;
        ForeachOutRecursive(copy, functor, 0, mGraph.GetDimensions());
    }

    template <typename TFunctor>
    void ForeachValidOut(const TensorVertexId id, TFunctor functor) const
    {
        ForeachValidOut(TensorVertexType(mGraph, id), functor);
    }

    template <typename TFunctor>
    void ForeachValidOut(const TensorVertexType& vert, TFunctor functor) const
    {
        TensorVertexType copy = vert;
        std::unordered_set<VertexIdType> occupiedVertices(copy.GetVertices().begin(), copy.GetVertices().end());
        if(occupiedVertices.size() < copy.GetVertices().size())
            return;
        ForeachValidOutRecursive(copy, functor, 0, mGraph.GetDimensions(), occupiedVertices);
    }

private:
    template <typename TFunctor>
    void ForeachOutRecursive(TensorVertexType& vertex,
                             TFunctor functor, //
                             const std::size_t dim,
                             const std::size_t dims) const
    {
        if(dim == dims)
        {
            functor(vertex);
            return;
        }
        const VertexIdType vertexAtDim = vertex.At(dim);

        // loop over case where this dimension is constant
        ForeachOutRecursive(vertex, functor, dim + 1, dims);

        // todo: loop over all out vertices for this dimension
        const auto& outVerts = mOutIterator.OutVertices(vertex.At(dim));
        for(const auto& outVert : outVerts)
        {
            vertex.Set(dim, outVert);
            ForeachOutRecursive(vertex, functor, dim + 1, dims);
        }
        vertex.Set(dim, vertexAtDim);
    }

    template <typename TFunctor>
    void ForeachValidOutRecursive(TensorVertexType& vertex,
                                  TFunctor functor, //
                                  const std::size_t dim,
                                  const std::size_t dims,
                                  std::unordered_set<VertexIdType>& occupiedVertices) const
    {
        if(dim == dims)
        {
            functor(vertex);
            return;
        }
        const VertexIdType vertexAtDim = vertex.At(dim);

        // loop over case where this dimension is constant
        ForeachValidOutRecursive(vertex, functor, dim + 1, dims, occupiedVertices);

        // loop over all out vertices for this dimension
        const auto& outVerts = mOutIterator.OutVertices(vertex.At(dim));
        for(const auto& outVert : outVerts)
        {
            if(occupiedVertices.insert(outVert).second)
            {
                vertex.Set(dim, outVert);
                ForeachValidOutRecursive(vertex, functor, dim + 1, dims, occupiedVertices);
                occupiedVertices.erase(outVert);
            }
        }
        vertex.Set(dim, vertexAtDim);
    }
};

// example of A* implementation
template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
// requires ConceptTensorGraph<TGraph>
class TensorAStar
{
public:
    using GraphType = TGraph;
    using VertexType = typename GraphType::VertexType;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    using TensorVertexType = TensorVertex<VertexType>;

    TensorAStar(TGraph& graph, TEdgeCostFunctor edgeCost, THeuristicFunctor heuristic)
        : mGraph(graph)
        , mEdgeCostFunctor(edgeCost)
        , mHeuristicFunctor(heuristic)
        , mOutIterator(graph)
    { }

    void Update() { mOutIterator.Update(); }

private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor; // defines the exact cost of going over a certain edge
    THeuristicFunctor mHeuristicFunctor; // defines the expected cost of going from a to b

    TensorOutIterator<TGraph> mOutIterator;

    // find out what the return type of the lambda is
    using CostType = decltype(mEdgeCostFunctor(TensorVertexType{}, TensorVertexType{}));
    struct HeapObject
    {
        TensorVertexId vertex;
        CostType f{std::numeric_limits<CostType>::max()};
    };
    struct ExploredObject
    {
        TensorVertexId vertex{};
        CostType f{std::numeric_limits<CostType>::max()}; // default init to max value
        CostType g{std::numeric_limits<CostType>::max()};
    };

public:
    std::vector<TensorVertexId> Calculate(const TensorVertexId start, const TensorVertexId target) const
    {
        const TensorVertexType tensorTarget(mGraph, target);

        // todo: unordered_map cleanup seems very slow
        std::map<TensorVertexId, ExploredObject> exploredSet;

        std::vector<HeapObject> heap;

        heap.push_back({start, 0.});
        exploredSet[start] = {start, 0., 0.};

        const auto compareF = [&](const auto& item1, const auto& item2) {
            return item1.f > item2.f; //
        };

        HeapObject current;
        TensorVertexType currentTensorVertex;

        while(heap.size() > 0)
        {
            // pick the lowest value out of heap
            std::pop_heap(heap.begin(), heap.end(), compareF);
            current = heap.back();
            heap.pop_back();

            if(current.vertex == target)
                break;
            if(exploredSet[current.vertex].f < current.f)
                continue;

            currentTensorVertex.FromId(mGraph, current.vertex);

            mOutIterator.ForeachValidOut(currentTensorVertex, [&](const TensorVertexType& out) {
                const auto edgeCost = mEdgeCostFunctor(currentTensorVertex, out);
                const auto newGScore = exploredSet[current.vertex].g + edgeCost;

                const auto outId = out.ToId(mGraph);
                auto& item = exploredSet[outId]; // NOTE: also initializes to max if not there yet

                if(newGScore < item.g)
                {
                    const CostType hscore = mHeuristicFunctor(out, tensorTarget);
                    item = {current.vertex, newGScore + hscore, newGScore};
                    heap.push_back({outId, newGScore + hscore});
                    std::push_heap(heap.begin(), heap.end(), compareF);
                }
            });
        }

        std::vector<TensorVertexId> path;
        path.push_back(current.vertex);
        while(true)
        {
            if(path.back() == start)
                break;
            auto next = exploredSet[path.back()].vertex;
            path.push_back(next);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    CostType GetCost(const std::vector<TensorVertexId>& path)
    {
        CostType cost = 0.;
        int i = 0;
        while(true)
        {
            if(i >= int(path.size()) - 1)
                break;

            auto* edgePtr = mGraph.GetEdge(path.at(i), path.at(i + 1));
            if(edgePtr == nullptr)
                break;
            // cost = cost + mEdgeCostFunctor(edgePtr->Source(), edgePtr->Id(), edgePtr->Sink());
            ++i;
        }
        return cost;
    }
};

// example of M* implementation
template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
class TensorMStar
{
public:
    using GraphType = TGraph;
    using VertexType = typename GraphType::VertexType;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    using TensorVertexType = TensorVertex<VertexType>;

    TensorMStar(TGraph& graph, TEdgeCostFunctor edgeCost, THeuristicFunctor heuristic)
        : mGraph(graph)
        , mEdgeCostFunctor(edgeCost)
        , mHeuristicFunctor(heuristic)
        , mOutIterator(graph)
    { }

    void Update() { mOutIterator.Update(); }

private:
    TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor; // defines the exact cost of going over a certain edge
    THeuristicFunctor mHeuristicFunctor; // defines the expected cost of going from a to b

    TensorOutIterator<TGraph> mOutIterator;

    // find out what the return type of the lambda is
    using CostType = decltype(mEdgeCostFunctor(TensorVertexType{}, TensorVertexType{}));

    struct ExploredObject
    {
        TensorVertexId vertex{};
        CostType f{std::numeric_limits<CostType>::max()}; // default init to max value
        CostType g{std::numeric_limits<CostType>::max()};
    };

public:
    std::vector<TensorVertexId> Calculate(const TensorVertexId start, const TensorVertexId target) const
    {
        const TensorVertexType tensorTarget(mGraph, target);
        std::unordered_map<TensorVertexId, ExploredObject> exploredSet;
        std::vector<TensorVertexId> heap;
        std::unordered_set<TensorVertexId> closedSet;

        heap.push_back(start);
        exploredSet[start] = {start, 0., 0.};

        const auto compareF = [&](const TensorVertexId v1, const TensorVertexId v2) {
            return exploredSet[v1].f > exploredSet[v2].f; //
        };

        TensorVertexId current;
        TensorVertexType currentTensorVertex;

        while(heap.size() > 0)
        {
            // pick the lowest value out of heap
            std::pop_heap(heap.begin(), heap.end(), compareF);
            current = heap.back();
            heap.pop_back();

            if(current == target)
                break;
            if(closedSet.find(current) != closedSet.end())
                continue;
            closedSet.insert(current);

            currentTensorVertex.FromId(mGraph, current);

            mOutIterator.ForeachValidOut(currentTensorVertex, [&](const TensorVertexType& out) {
                const auto edgeCost = mEdgeCostFunctor(currentTensorVertex, out);
                const auto newGScore = exploredSet[current].g + edgeCost;

                const auto outId = out.ToId(mGraph);
                auto& item = exploredSet[outId]; // NOTE: also initializes to max if not there yet

                if(newGScore < item.g)
                {
                    const CostType hscore = mHeuristicFunctor(out, tensorTarget);
                    item = {current, newGScore + hscore, newGScore};
                    heap.push_back(outId);
                    std::push_heap(heap.begin(), heap.end(), compareF);
                }
            });
        }

        std::vector<TensorVertexId> path;
        path.push_back(current);
        while(true)
        {
            if(path.back() == start)
                break;
            auto next = exploredSet[path.back()].vertex;
            path.push_back(next);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};

} // namespace graph
} // namespace vic