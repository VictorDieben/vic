#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include "vic/memory/flat_set.h"
#include "vic/memory/merge_sort.h"

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

inline constexpr void SetNthBit(CollisionSet& collisionset, const uint8_t index)
{
    collisionset |= ((uint64_t)1) << index; //
}

inline constexpr bool NthBitIsSet(const CollisionSet collisionset, const uint8_t index)
{
    const CollisionSet mask = ((uint64_t)1) << index;
    return collisionset & mask;
}

inline constexpr bool CollisionSetContains(const CollisionSet collisionSet, const CollisionSet possibleSubset)
{
    return (collisionSet & possibleSubset) == possibleSubset; //
}

inline constexpr bool MergeCollisionSets(const CollisionSet a, const CollisionSet b)
{
    return a | b; //
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
        : mOutIterator(outIterator)
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

        if(!NthBitIsSet(collisionSet, (uint8_t)dim)) // agent not in collision, step in policy direction
        {
            const bool atDestination = (vertex[dim] == policy[dim]);
            const bool inserted = atDestination ? false : occupiedVertices.insert(policy[dim]).second;

            if(atDestination || inserted)
            {
                vertex[dim] = policy[dim];
                ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);
                vertex[dim] = vertexAtDim;
                if(inserted)
                    occupiedVertices.pop_back();
            }
        }
        else // agent in collision, fully expand this dimension
        {
            ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);

            for(const auto& outVert : mOutIterator.OutVertices(vertex[dim]))
            {
                if(occupiedVertices.insert(outVert).second)
                {
                    vertex[dim] = outVert;
                    ForeachOutVertexRecursive(vertex, occupiedVertices, policy, collisionSet, dim + 1, dims, functor);
                    occupiedVertices.pop_back();
                }
            }
            vertex[dim] = vertexAtDim;
        }
    }
};

//template <typename TGraph, typename TCost>
//void Backprop(const TGraph& graph, //
//              const std::vector<typename TGraph::VertexIdType>& vertex,
//              const CollisionSet collisionSet,
//              std::vector<uint64_t>& openList)
//{
//    //
//}

template <typename CartesianVertexType, typename TPolicy>
void ConstructPolicy(const TPolicy& policy,
                     const CartesianVertexType& position, //
                     const CartesianVertexType& target,
                     CartesianVertexType& buffer)
{
    const auto numDims = position.size();
    buffer.clear();
    for(std::size_t i = 0; i < numDims; ++i)
        buffer.push_back(policy.Policy(position[i], target[i]));
}

template <typename CartesianVertexType, typename TPolicy>
CartesianVertexType ConstructPolicy(const TPolicy& policy,
                                    const CartesianVertexType& position, //
                                    const CartesianVertexType& target)
{
    CartesianVertexType buffer;
    ConstructPolicy(policy, position, target, buffer);
    return buffer;
}

template <typename TPolicy, typename CartesianVertexType>
    requires ConceptPolicy<TPolicy>
CollisionSet ConstructCollisionSet(const TPolicy& policy, //
                                   const CartesianVertexType& start,
                                   const CartesianVertexType& target)
{
    const auto numAgents = start.size();

    CartesianVertexType current = start;
    CartesianVertexType next;

    CollisionSet collisionSet{};

    // todo: this can probably be sped up by skipping pairs of agents that are already in collision
    while(current != target)
    {
        ConstructPolicy(policy, current, target, next);

        // for each pair of agents, check if their next step will conflict
        for(std::uint8_t a = 0; a < numAgents - 1; ++a)
        {
            for(std::uint8_t b = a + 1; b < numAgents; ++b)
            {
                if((current.at(a) == next.at(b)) || //
                   (next.at(a) == current.at(b)) || //
                   (next.at(a) == next.at(b)))
                {
                    SetNthBit(collisionSet, a);
                    SetNthBit(collisionSet, b);
                }
            }
        }
        std::swap(current, next);
    }

    return collisionSet;
}

template <typename CartesianVertexType>
CollisionSet FindCollisions(const CartesianVertexType& from, const CartesianVertexType& to)
{
    const auto numAgents = (uint8_t)from.size();

    CollisionSet collisionSet{};

    for(std::uint8_t a = 0; a < numAgents - 1; ++a)
    {
        for(std::uint8_t b = a + 1; b < numAgents; ++b)
        {
            if((from.at(a) == to.at(b)) || //
               (to.at(a) == from.at(b)) || //
               (to.at(a) == to.at(b)))
            {
                SetNthBit(collisionSet, a);
                SetNthBit(collisionSet, b);
            }
        }
    }

    return collisionSet;
}

template <typename TVertex>
using MStarVertex = std::vector<TVertex>;

template <typename TCost, typename TVertex>
struct MStarExploredObject
{
    MStarVertex<TVertex> previous{};
    TCost g{std::numeric_limits<TCost>::max()};
    CollisionSet collisionSet;
    std::set<MStarVertex<TVertex>> backpropSet; // todo: (unordered) flat map?
};

template <typename TCost, typename TVertex>
struct MStarHeapObject
{
    MStarVertex<TVertex> vertex{};
    TCost f{std::numeric_limits<TCost>::max()};
};

template <typename TCost>
struct MStarClosedMapItem
{
    TCost f;
    CollisionSet collisionSet;
};

template <typename TCost, typename TGraph>
struct MStar
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using CartesianVertexType = std::vector<VertexIdType>;

    using CostType = TCost;

    MStar(const TGraph& graph)
        : mOutIterator(graph)
        , mSubsetIterator(graph, mOutIterator)
    { }

private:
    BaseOutVertexIterator<TGraph> mOutIterator;
    SubsetOutIterator<TGraph, BaseOutVertexIterator<TGraph>> mSubsetIterator;

    using VertexType = MStarVertex<VertexIdType>;
    using ExploredObject = MStarExploredObject<CostType, VertexIdType>;
    using HeapObject = MStarHeapObject<CostType, VertexIdType>;
    using ClosedObject = MStarClosedMapItem<CostType>;

    // working data
    std::map<VertexType, ExploredObject> mExploredMap;
    std::vector<HeapObject> mHeap;
    std::vector<HeapObject> mHeapBuffer; // placeholder vec that will contain the sorted elements each iteration
    std::map<VertexType, ClosedObject> mClosedMap;

public:
    template <typename TEdgeCostFunctor, typename THeuristicFunctor, typename TPolicy>
        requires ConceptEdgeCost<TEdgeCostFunctor> && ConceptEdgeCost<THeuristicFunctor>
    auto Run(const CartesianVertexType& start, //
             const CartesianVertexType& target,
             const TEdgeCostFunctor& edgeCostFunctor,
             const THeuristicFunctor& heuristicFunctor,
             const TPolicy& policy)
    {

        //std::map<VertexType, ExploredObject> exploredMap;
        //std::vector<HeapObject> heap;
        //std::vector<HeapObject> heapBuffer; // placeholder vec that will contain the sorted elements each iteration
        //std::map<VertexType, ClosedObject> closedMap;

        mExploredMap.clear();
        mHeap.clear();
        mHeapBuffer.clear();
        mClosedMap.clear();

        mExploredMap[start] = ExploredObject(start, 0., CollisionSet{}, {});

        mHeap.push_back(HeapObject{start, 0.});

        // SubsetOutIterator subsetIterator(mGraph, mOutIterator);

        // note: > because default make_heap behaviour is max heap for operator<
        const auto compareF = [&](const auto& v1, const auto& v2) { return v1.f < v2.f; };

        HeapObject current;
        VertexType policyDirection;

        const auto backprop = [&](const CartesianVertexType& vertex, //
                                  const CollisionSet collisionSet) {
            // make a recursive lambda, by passing a reference to itself
            const auto backprop_recursive = [&](const CartesianVertexType& vertex, //
                                                const CollisionSet collisionSet,
                                                auto& self) {
                auto& currentMapEntry = mExploredMap[vertex];
                // if the current collision set fully contains the new collision set, ignore
                if(CollisionSetContains(currentMapEntry.collisionSet, collisionSet))
                    return;

                currentMapEntry.collisionSet |= collisionSet;

                // if we already closed this vertex, remove from closed set and re-open
                if(mClosedMap.contains(vertex))
                {
                    mHeap.push_back(HeapObject{vertex, mClosedMap.at(vertex).f});
                    mClosedMap.erase(vertex);
                }
                //else
                //{
                //}

                // recurse
                for(const auto& parent : mExploredMap[vertex].backpropSet)
                    self(parent, collisionSet, self);
            };

            return backprop_recursive(vertex, collisionSet, backprop_recursive);
        };

        while(true)
        {
            if(mHeap.empty())
                break;

            current = std::move(mHeap.front());

            if(current.vertex == target)
                break;

            if(mClosedMap.contains(current.vertex))
            {
                mHeap.erase(mHeap.begin());
                continue;
            }

            const auto& currentExploredItem = mExploredMap[current.vertex];
            const auto currentGScore = currentExploredItem.g;

            ConstructPolicy(policy, current.vertex, target, policyDirection);

            const CollisionSet policyCollisionSet = currentExploredItem.collisionSet | FindCollisions(current.vertex, policyDirection);

            auto heapSize = mHeap.size();

            bool updatedHeap = false;

            // todo: if this item was previously expanded already, only expand in a subset

            mSubsetIterator.ForeachOutVertex(current.vertex, policyDirection, policyCollisionSet, [&](const CartesianVertexType& other) {
                // append
                ExploredObject& item = mExploredMap[other]; // inserts if it does not exist

                const bool itemIsNew = item.backpropSet.empty(); // NOTE: it would be WAY better if we can detect directly if the operator[] added a new item, this is a hack

                item.backpropSet.insert(current.vertex);
                item.collisionSet |= policyCollisionSet;

                const auto edgeCost = edgeCostFunctor(current.vertex, other);
                const auto newGScore = currentGScore + edgeCost;

                backprop(other, item.collisionSet);

                // const auto collisionSet = ConstructCollisionSet();
                if(newGScore < item.g)
                {
                    // set neighbour data to new values
                    const CostType hscore = heuristicFunctor(other, target);
                    const CostType newFScore = newGScore + hscore;

                    item.previous = current.vertex;
                    item.g = newGScore;

                    if(itemIsNew)
                    {
                        mHeap.push_back(HeapObject{other, newFScore});
                    }
                    else if(mClosedMap.contains(other))
                    {
                        mHeap.push_back(HeapObject{other, newFScore});
                        mClosedMap.erase(other);
                    }
                    else
                    {
                        // item is not new and not closed, so it is already in the heap.
                        // the old f score is no longer valid
                        // todo:
                        auto it = std::find_if(mHeap.begin(), mHeap.begin() + heapSize, [&](const auto& heapItem) { return heapItem.vertex == other; });
                        if(it == mHeap.begin() + heapSize)
                            return; // error

                        it->f = newFScore; // note: store that we updated items in the existing heap?
                        updatedHeap = true;
                    }
                }
            });

            // sort new items
            auto heapNewSize = mHeap.size();
            if(updatedHeap) // we might have updated some values, skip first because we need to remove it
                std::sort(mHeap.begin() + 1, mHeap.begin() + heapSize, compareF);
            std::sort(mHeap.begin() + heapSize, mHeap.begin() + heapNewSize, compareF); // these are completely unsorted

            // merge the old heap and the newly added items, write result to buffer heap, the swap
            mHeapBuffer.clear();
            mHeapBuffer.reserve(mHeap.size());

            // todo: analyze if this makes any difference
            //
            //std::merge(heap.begin() + 1, // skip first, it was the current node this iteration
            //           heap.begin() + heapSize,
            //           heap.begin() + heapSize,
            //           heap.begin() + heapNewSize,
            //           std::back_inserter(heapBuffer),
            //           compareF);

            vic::sorting::move_merge(mHeap.begin() + 1, // skip first, it was the current node this iteration
                                     mHeap.begin() + heapSize,
                                     mHeap.begin() + heapSize,
                                     mHeap.begin() + heapNewSize,
                                     std::back_inserter(mHeapBuffer),
                                     compareF);

            std::swap(mHeap, mHeapBuffer);

            // closedSet.insert(std::move(current.vertex));
            mClosedMap[current.vertex] = ClosedObject{current.f, policyCollisionSet};
        }

        // https://github.com/jdonszelmann/research-project/blob/master/python/mstar/rewrite/find_path.py

        std::vector<CartesianVertexType> path{};

        auto currentVertex = current.vertex;
        if(currentVertex != target)
            return path;

        // todo: backtrack
        path.push_back(target);

        while(currentVertex != start) // note: for multi-robot, we cannot assume than number of vertices is really the upper limit, 4x should be enough
        {
            if(path.size() > 1000)
                return std::vector<CartesianVertexType>{};
            currentVertex = mExploredMap[currentVertex].previous;
            path.push_back(currentVertex);
        }

        std::reverse(path.begin(), path.end());

        return path;
    }
};

// M*: A Complete Multirobot Path Planning Algorithm with Performance Bounds:
// https://citeseerx.ist.psu.edu/viewdoc/download?rep=rep1&type=pdf&doi=10.1.1.221.1909

// Subdimensional expansion for multirobot path planning
// https://pdf.sciencedirectassets.com/271585/1-s2.0-S0004370214X00123/1-s2.0-S0004370214001271/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEK3%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJHMEUCIAoGH2jFIH6eaM4%2FlutGi4DeuZ2hg9zG0cK5GxZUe2jDAiEAwc03VXhyHGZ8kJ2R9O2kUQXTByeR1rGT78oXQwk%2FmmIquwUI1v%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FARAFGgwwNTkwMDM1NDY4NjUiDE%2BCJBn2XWFzFSwPByqPBfvQ9kAqrtQM3QSj9tpkWhjQHhc4zTxeHGNJxaYGJIT4utnlTsEWdOyVBvsthXUtJuae3ppqbEWFi4Mdd%2Fqe5awtSYKX2HefBU8hrx9O0hk4Bu2bhmX75OZ4LICgZ20QBmhWDWT1wf53aObkLohqpuFPX8Mu6dvliU6ci0KI9C%2FHHig61dOl7XcOS3VSYBP2Re7L6oZbFsMS3uExQE8jemnawsqghkWzCmVGDJNbXj0MWuVFCDXCpJAkqcTbvqfgsIH5bwXFvhORri5clZUwK9Zz6yoxS0TvKLt3gCCzxcCF3UjjbnOcEzPEx17Dh9EIpOPagOssx1mVCOFuLQ89vzxkOZcjcPDcOgk26QkyG3X1NbItwHGkEYw9LTQ72ih9b8YhHlFIGJ6EmNLTvgfarF7%2BhPrubmAJb2Ki30fhjDgXR0uXnyXhEixgjeJTZLLh2Fz7ViFtkOVzkDEMgdHR6osKjxu8Br6MX0Nx4hQwpVSQVCxiWk89bR0VgzRdV2wAd9FWdKGfoAnylWy0uJvgyshMXEk71qZnfd%2FrLdDJtH%2BAd6woPTcCxnxLhazjDZBDkvUzHFmLCgXMrQ%2FwbL3c2QpHgLJWCc4T%2FDi124xefXW7cWfzdXXlBGZdsUT2Cqfa%2B6NUw1D2zvJqm%2F%2F87XBq4o6AnLMPcihvt9oFk4iRF5Sno3DvUzIC2sG%2B3lIb7uWzuz6PcMUS2E6oF3CY3RihzKzQB%2FrUYNKsbXBwMOtd%2FIIOH7f5wnZMHoj42AyajNkrzpiwoANPw5QTDhSWwhHcB6FvktOSCJvQ%2BkwcdzPauQnTOKu9az%2B57JHHojWLKYT6EBML%2FRDw%2BxeAK3DpMiost5QLzzaqNlzYXYCidRWMU%2Fowwsf%2BqQY6sQGsfOLGFScM9wAAk80iStUAjo2BfSqcSFDMBb8JBeVeX6D%2BsYx47QisiO5hYp5YwwPWw%2BPLAC55oQG3weRI8ilxzLZPArDbas9rnfegUSXwjOccUN2WbGl6MiuhYKA%2BmpzGaQGlKqL0CzkbyjyvfGXItxl%2BikKLvoyWdxrg4fTCTAjybAoD0rKEHEhQvd6e0kPd3%2Bcy5MS4JSmyjF%2FqmEl47UR9fDbfJABWFhfCalMQ8So%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20231030T130854Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYZBPGPBWY%2F20231030%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=12d249e97d240c494abe770d8814ce775f2232bf25194d3098688f852fe3d187&hash=dd17c10d887ae5cbc0c44cd4443851baecf6d269916f312a68c556c31370d50c&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S0004370214001271&tid=spdf-b2d1d29c-1919-44b6-8d52-46d2cd8e7e4b&sid=f1d5d06d33b702495918a227b13db629c00dgxrqb&type=client&tsoh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&ua=080f575106540657000c&rr=81e3e3bc998d6604&cc=nl

// tu delft student:
//https://github.com/jdonszelmann/research-project

} // namespace graph2
} // namespace vic