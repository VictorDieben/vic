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

template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor, typename TPolicy>
    requires ConceptEdgeCost<TEdgeCostFunctor> && ConceptEdgeCost<THeuristicFunctor>
struct MStar
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using CartesianVertexType = std::vector<VertexIdType>;

    MStar(const TGraph& graph, //
          TEdgeCostFunctor edgeCostFunctor,
          THeuristicFunctor heuristicFunctor,
          TPolicy policy)
        : mGraph(graph)
        , mEdgeCostFunctor(edgeCostFunctor)
        , mHeuristicFunctor(heuristicFunctor)
        , mPolicy(policy)
        , mOutIterator(graph)
    { }

private:
    const TGraph& mGraph;
    TEdgeCostFunctor mEdgeCostFunctor;
    THeuristicFunctor mHeuristicFunctor;
    TPolicy mPolicy;
    BaseOutVertexIterator<TGraph> mOutIterator;

public:
    using CostType = decltype(mEdgeCostFunctor(CartesianVertexType{}, CartesianVertexType{}));
    using HeuristicType = decltype(mHeuristicFunctor(CartesianVertexType{}, CartesianVertexType{}));
    static_assert(std::is_same_v<CostType, HeuristicType>);

    struct ExploredObject
    {
        CartesianVertexType vertex{}; // previous
        CollisionSet collisionSet;

        CostType f{std::numeric_limits<CostType>::max()};
        CostType g{std::numeric_limits<CostType>::max()};

        std::set<CartesianVertexType> backpropSet;
    };
    void ConstructPolicy(const CartesianVertexType& position, //
                         const CartesianVertexType& target,
                         CartesianVertexType& buffer) const
    {
        const auto numDims = position.size();
        buffer.clear();
        for(std::size_t i = 0; i < numDims; ++i)
            buffer.push_back(mPolicy.Policy(position.at(i), target.at(i)));
    }

    CollisionSet ConstructCollisionSet(const CartesianVertexType& start, const CartesianVertexType& target) const
    {
        const auto numAgents = start.size();

        CartesianVertexType current = start;
        CartesianVertexType next;

        CollisionSet collisionSet;

        // todo: this can probably be sped up by ignoring agents that are already in collision
        while(current != target)
        {
            ConstructPolicy(current, target, next);

            // for each pair of agents, check if their next step will conflict
            for(std::size_t a = 0; a < numAgents - 1; ++a)
            {
                for(std::size_t b = a + 1; b < numAgents; ++b)
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

    std::map<CartesianVertexType, ExploredObject> mExploredMap;
    std::vector<CartesianVertexType> mHeap;

    void Backprop(const CartesianVertexType& vertex, //
                  const CollisionSet collisionSet,
                  std::vector<uint64_t>& openList)
    {
        //
    }

    auto Run(const CartesianVertexType& start, const CartesianVertexType& target)
    {
        mExploredMap.clear();
        mExploredMap.at(start) = ExploredObject(start, CollisionSet{}, 0., 0., {});

        mHeap.clear();
        mHeap.push_back(start);

        SubsetOutIterator subsetIterator(mGraph, mOutIterator);

        // note: > because default make_heap behaviour is max heap for operator<
        const auto compareF = [&](const CartesianVertexType v1, const CartesianVertexType v2) { return mExploredMap.at(v1).f > mExploredMap.at(v2).f; };

        CartesianVertexType current;
        CartesianVertexType policy;

        while(!mHeap.empty())
        {
            std::pop_heap(mHeap.begin(), mHeap.end(), compareF);
            current = mHeap.back();
            mHeap.pop_back();

            if(current == target)
                break;

            ConstructPolicy(current, target, policy);

            const CollisionSet collisionSet = ConstructCollisionSet(current, target);

            subsetIterator.ForeachOutVertex(current, policy, collisionSet, [&](const CartesianVertexType& other) {
                //
            });
        }

        std::vector<CartesianVertexType> path;

        // todo: backtrack

        return path;
    }
};

// M*: A Complete Multirobot Path Planning Algorithm with Performance Bounds:
// https://citeseerx.ist.psu.edu/viewdoc/download?rep=rep1&type=pdf&doi=10.1.1.221.1909

// Subdimensional expansion for multirobot path planning
// https://pdf.sciencedirectassets.com/271585/1-s2.0-S0004370214X00123/1-s2.0-S0004370214001271/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEK3%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJHMEUCIAoGH2jFIH6eaM4%2FlutGi4DeuZ2hg9zG0cK5GxZUe2jDAiEAwc03VXhyHGZ8kJ2R9O2kUQXTByeR1rGT78oXQwk%2FmmIquwUI1v%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FARAFGgwwNTkwMDM1NDY4NjUiDE%2BCJBn2XWFzFSwPByqPBfvQ9kAqrtQM3QSj9tpkWhjQHhc4zTxeHGNJxaYGJIT4utnlTsEWdOyVBvsthXUtJuae3ppqbEWFi4Mdd%2Fqe5awtSYKX2HefBU8hrx9O0hk4Bu2bhmX75OZ4LICgZ20QBmhWDWT1wf53aObkLohqpuFPX8Mu6dvliU6ci0KI9C%2FHHig61dOl7XcOS3VSYBP2Re7L6oZbFsMS3uExQE8jemnawsqghkWzCmVGDJNbXj0MWuVFCDXCpJAkqcTbvqfgsIH5bwXFvhORri5clZUwK9Zz6yoxS0TvKLt3gCCzxcCF3UjjbnOcEzPEx17Dh9EIpOPagOssx1mVCOFuLQ89vzxkOZcjcPDcOgk26QkyG3X1NbItwHGkEYw9LTQ72ih9b8YhHlFIGJ6EmNLTvgfarF7%2BhPrubmAJb2Ki30fhjDgXR0uXnyXhEixgjeJTZLLh2Fz7ViFtkOVzkDEMgdHR6osKjxu8Br6MX0Nx4hQwpVSQVCxiWk89bR0VgzRdV2wAd9FWdKGfoAnylWy0uJvgyshMXEk71qZnfd%2FrLdDJtH%2BAd6woPTcCxnxLhazjDZBDkvUzHFmLCgXMrQ%2FwbL3c2QpHgLJWCc4T%2FDi124xefXW7cWfzdXXlBGZdsUT2Cqfa%2B6NUw1D2zvJqm%2F%2F87XBq4o6AnLMPcihvt9oFk4iRF5Sno3DvUzIC2sG%2B3lIb7uWzuz6PcMUS2E6oF3CY3RihzKzQB%2FrUYNKsbXBwMOtd%2FIIOH7f5wnZMHoj42AyajNkrzpiwoANPw5QTDhSWwhHcB6FvktOSCJvQ%2BkwcdzPauQnTOKu9az%2B57JHHojWLKYT6EBML%2FRDw%2BxeAK3DpMiost5QLzzaqNlzYXYCidRWMU%2Fowwsf%2BqQY6sQGsfOLGFScM9wAAk80iStUAjo2BfSqcSFDMBb8JBeVeX6D%2BsYx47QisiO5hYp5YwwPWw%2BPLAC55oQG3weRI8ilxzLZPArDbas9rnfegUSXwjOccUN2WbGl6MiuhYKA%2BmpzGaQGlKqL0CzkbyjyvfGXItxl%2BikKLvoyWdxrg4fTCTAjybAoD0rKEHEhQvd6e0kPd3%2Bcy5MS4JSmyjF%2FqmEl47UR9fDbfJABWFhfCalMQ8So%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20231030T130854Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYZBPGPBWY%2F20231030%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=12d249e97d240c494abe770d8814ce775f2232bf25194d3098688f852fe3d187&hash=dd17c10d887ae5cbc0c44cd4443851baecf6d269916f312a68c556c31370d50c&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S0004370214001271&tid=spdf-b2d1d29c-1919-44b6-8d52-46d2cd8e7e4b&sid=f1d5d06d33b702495918a227b13db629c00dgxrqb&type=client&tsoh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&ua=080f575106540657000c&rr=81e3e3bc998d6604&cc=nl

} // namespace graph2
} // namespace vic