#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include <algorithm>
#include <map>
#include <set>

namespace vic
{
namespace graph2
{

using CollisionSet = uint64_t;

inline constexpr void SetNthBit(CollisionSet& collisionset, uint8_t index)
{
    collisionset |= ((uint64_t)1) << index; //
}

inline constexpr bool NthBitIsSet(const CollisionSet collisionset, const uint8_t index)
{
    const CollisionSet mask = ((uint64_t)1) << index;
    return collisionset & mask;
}
//
//template <typename TGraph>
//class CartesianOutIterator
//{
//public:
//    using VertexIdType = typename TGraph::VertexIdType;
//
//    using TensorGraphType = typename TTensorGraph;
//    using GraphType = typename TTensorGraph::GraphType;
//
//private:
//    CartesianOutIterator& mGraph;
//    OutVertexIterator<GraphType> mOutIterator;
//
//public:
//    CartesianOutIterator(TTensorGraph& graph)
//        : mGraph(graph)
//        , mOutIterator(graph.GetGraph())
//    { }
//
//    void Update()
//    {
//        mOutIterator.Update(); //
//    }
//
//    template <typename TFunctor>
//    void ForeachOut(const VertexIdType id, TFunctor functor) const
//    {
//        ForeachOut(TensorVertexType(mGraph, id), functor);
//    }
//
//    template <typename TFunctor>
//    void ForeachOut(const TensorVertexType& vert, TFunctor functor) const
//    {
//        TensorVertexType copy = vert;
//        ForeachOutRecursive(copy, functor, 0, mGraph.GetDimensions());
//    }
//
//    template <typename TFunctor>
//    void ForeachValidOut(const TensorVertexId id, TFunctor functor) const
//    {
//        ForeachValidOut(TensorVertexType(mGraph, id), functor);
//    }
//
//    template <typename TFunctor>
//    void ForeachValidOut(const TensorVertexType& vert, TFunctor functor) const
//    {
//        TensorVertexType copy = vert;
//        std::unordered_set<VertexIdType> occupiedVertices(copy.GetVertices().begin(), copy.GetVertices().end());
//        if(occupiedVertices.size() < copy.GetVertices().size())
//            return;
//        ForeachValidOutRecursive(copy, functor, 0, mGraph.GetDimensions(), occupiedVertices);
//    }
//
//private:
//    template <typename TFunctor>
//    void ForeachOutRecursive(TensorVertexType& vertex,
//                             TFunctor functor, //
//                             const std::size_t dim,
//                             const std::size_t dims) const
//    {
//        if(dim == dims)
//        {
//            functor(vertex);
//            return;
//        }
//        const VertexIdType vertexAtDim = vertex.At(dim);
//
//        // loop over case where this dimension is constant
//        ForeachOutRecursive(vertex, functor, dim + 1, dims);
//
//        // todo: loop over all out vertices for this dimension
//        const auto& outVerts = mOutIterator.OutVertices(vertex.At(dim));
//        for(const auto& outVert : outVerts)
//        {
//            vertex.Set(dim, outVert);
//            ForeachOutRecursive(vertex, functor, dim + 1, dims);
//        }
//        vertex.Set(dim, vertexAtDim);
//    }
//
//    template <typename TFunctor>
//    void ForeachValidOutRecursive(TensorVertexType& vertex,
//                                  TFunctor functor, //
//                                  const std::size_t dim,
//                                  const std::size_t dims,
//                                  std::unordered_set<VertexIdType>& occupiedVertices) const
//    {
//        if(dim == dims)
//        {
//            functor(vertex);
//            return;
//        }
//        const VertexIdType vertexAtDim = vertex.At(dim);
//
//        // loop over case where this dimension is constant
//        ForeachValidOutRecursive(vertex, functor, dim + 1, dims, occupiedVertices);
//
//        // loop over all out vertices for this dimension
//        const auto& outVerts = mOutIterator.OutVertices(vertex.At(dim));
//        for(const auto& outVert : outVerts)
//        {
//            if(occupiedVertices.insert(outVert).second)
//            {
//                vertex.Set(dim, outVert);
//                ForeachValidOutRecursive(vertex, functor, dim + 1, dims, occupiedVertices);
//                occupiedVertices.erase(outVert);
//            }
//        }
//        vertex.Set(dim, vertexAtDim);
//    }
//};

template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
auto MStar(const TGraph& graph, //
           TEdgeCostFunctor edgeCostFunctor,
           THeuristicFunctor heuristicFunctor,
           const typename TGraph::VertexIdType start,
           const typename TGraph::VertexIdType target)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    using CostType = double;

    // todo

    struct ExploredObject
    {
        CollisionSet collisionSet{0};
        CostType cost{std::numeric_limits<CostType>::max()};
        VertexIdType previous{};
    };

    std::vector<VertexIdType> heap;
    heap.push_back(start);

    std::set<VertexIdType> closedSet;
    std::map<VertexIdType, ExploredObject> exploredMap;
    exploredMap[start] = ExploredObject{{}, 0, start};

    // note: > because default make_heap behaviour is max heap for operator<
    const auto compareCost = [&](const VertexIdType v1, const VertexIdType v2) { return exploredMap.at(v1).cost > exploredMap.at(v2).cost; };

    VertexIdType current = start;

    while(!heap.empty())
    {
        //
        std::pop_heap(heap.begin(), heap.end(), compareCost);
        current = heap.back();
        heap.pop_back();

        if(current == target)
            break;
    }

    return 1; // todo
}

} // namespace graph2
} // namespace vic