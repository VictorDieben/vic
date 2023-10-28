#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include "vic/memory/flat_set.h"

#include <algorithm>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

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

template <typename TGraph, typename TOutVertexIterator>
class SubsetOutIterator
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using Graph = TGraph;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexIdType>;
    using CartesianEdgeType = std::vector<EdgeIdType>;

    SubsetOutIterator(const TGraph& graph, const TOutVertexIterator& outIterator)
        : mGraph(graph)
        , mOutIterator(outIterator)
    { }

    template <typename TFunctor>
    void ForeachOutVertex(const CartesianVertexType& from, //
                          const CartesianVertexType& policy,
                          const CollisionSet collisionSet,
                          TFunctor functor) const
    {
        auto copy = from;
        OccupiedSetType occupiedSet(from.begin(), from.end());
        ForeachOutVertexRecursive(copy, occupiedSet, policy, collisionSet, 0, from.size(), functor);
    }

private:
    const Graph& mGraph;
    const TOutVertexIterator& mOutIterator; // todo: constrain with concept

    using OccupiedSetType = vic::memory::UnorderedFlatSet<VertexIdType>;

    template <typename TFunctor>
    void ForeachOutVertexRecursive(CartesianVertexType& vertex, //
                                   OccupiedSetType& occupiedVertices,
                                   const CartesianVertexType& policy,
                                   const CollisionSet collisionSet,
                                   const std::size_t dim,
                                   const std::size_t dims,
                                   TFunctor functor) const
    {
        if(dim == dims)
        {
            functor(vertex);
            return;
        }
        const VertexIdType vertexAtDim = vertex.at(dim);

        if(!NthBitIsSet(collisionSet, dim)) // agent not in collision, step in policy direction
        {
            if(occupiedVertices.insert(policy.at(dim)).second)
            {
                vertex.at(dim) = policy.at(dim);
                ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);
                vertex.at(dim) = vertexAtDim;
                occupiedVertices.pop_back();
            }
        }
        else // agent in collision, fully expand this dimension
        {
            ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);

            for(const auto& outVert : mOutIterator.OutVertices(vertex.at(dim)))
            {
                if(occupiedVertices.insert(outVert).second)
                {
                    vertex.at(dim) = outVert;
                    ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);
                    occupiedVertices.pop_back();
                }
            }
            vertex.at(dim) = vertexAtDim;
        }
    }
};

template <typename TGraph>
void Backprop(const TGraph& graph, //
              const std::vector<typename TGraph::VertexIdType>& vertex,
              const CollisionSet collisionSet,
              std::vector<uint64_t>& openList)
{
    //
}

// paper:
// https://citeseerx.ist.psu.edu/viewdoc/download?rep=rep1&type=pdf&doi=10.1.1.221.1909
template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
auto MStar(const TGraph& graph, //
           TEdgeCostFunctor edgeCostFunctor,
           THeuristicFunctor heuristicFunctor,
           const std::vector<typename TGraph::VertexIdType>& start,
           const std::vector<typename TGraph::VertexIdType>& target)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    using CartesianVertexIdType = uint64_t;
    using CartesianEdgeIdType = uint64_t;

    using CartesianVertexType = std::vector<VertexIdType>;
    using CartesianEdgeType = std::vector<EdgeIdType>;

    using CostType = double;
    using HeuristicType = decltype(heuristicFunctor(CartesianVertexType{}, CartesianVertexType{}));
    static_assert(std::is_same_v<CostType, HeuristicType>);
    assert(start.size() == target.size());

    const auto numVertices = graph.NumVertices();
    const auto dims = start.size();

    // todo: make this a function argument
    const auto outIterator = GetOutVertexIterator(graph);

    const auto subsetIterator = SubsetOutIterator(graph, outIterator);

    struct ExploredObject
    {
        CartesianVertexIdType previous{}; // back_ptr
        CollisionSet collisionSet{0};
        CostType cost{std::numeric_limits<CostType>::max()};
        CostType heuristic{std::numeric_limits<CostType>::max()};

        std::set<CartesianVertexIdType> back_set{};
    };

    std::vector<CartesianVertexIdType> heap;

    const auto startId = ToId<CartesianVertexIdType>(start, numVertices);
    const auto targetId = ToId<CartesianVertexIdType>(target, numVertices);
    heap.push_back(startId);

    std::set<CartesianVertexIdType> closedSet;
    std::map<CartesianVertexIdType, ExploredObject> exploredMap;
    exploredMap[startId] = ExploredObject{startId, {}, 0.};

    // note: > because default make_heap behaviour is max heap for operator<
    const auto compareCost = [&](const CartesianVertexIdType v1, const CartesianVertexIdType v2) { return exploredMap.at(v1).cost > exploredMap.at(v2).cost; };

    CartesianVertexIdType currentId = startId;
    CartesianVertexType current = start;

    while(!heap.empty())
    {
        std::pop_heap(heap.begin(), heap.end(), compareCost);
        currentId = heap.back();
        heap.pop_back();

        if(currentId == targetId)
            break;
        if(closedSet.contains(currentId))
            continue;
        closedSet.insert(currentId);

        //subsetIterator.ForeachOutVertex(current, [&](const CartesianVertexType& other) {
        //    const auto otherId = ToId<CartesianVertexIdType>(other, numVertices);

        //    const auto edgeCost = edgeCostFunctor(current, other);
        //    const auto newCost = exploredMap[currentId].cost + edgeCost;

        //    auto& item = exploredMap[otherId]; // adds item if it did not exist

        //    if(newCost < item.g)
        //    {
        //        const CostType hscore = heuristicFunctor(other, target);
        //        item = {current, newCost + hscore, newCost};

        //        heap.push_back(otherId);
        //        std::push_heap(heap.begin(), heap.end(), compareCost);
        //    }
        //});
    }

    if(currentId != targetId)
        return std::vector<CartesianVertexType>{};

    std::vector<CartesianVertexType> path;
    path.push_back(target);

    //currentId = targetId;
    //while(path.size() < numVertices * 4) // note: for multi-robot, we cannot assume than number of vertices is really the upper limit, 4x should be enough
    //{
    //    if(currentId == startId)
    //        break;
    //    auto& node = exploredMap[currentId].previous;
    //    current = node;

    //    path.push_back(ToVector<VertexIdType>(currentId, dims, numVertices));
    //}

    //std::reverse(path.begin(), path.end());

    return path;
}

} // namespace graph2
} // namespace vic