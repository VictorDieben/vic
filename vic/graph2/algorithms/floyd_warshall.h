#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"

#include <algorithm>
#include <vector>

namespace vic
{
namespace graph2
{

// Calculate the matrix of shortest distances and shortest paths
// Can be used by other algorithms for policy.
template <typename TCost, typename TVertexId, bool directed = false>
class FloydWarshall
{
public:
    using VertexIdType = TVertexId;
    using CostType = TCost;

    template <typename TGraph, typename TEdgeCostFunctor>
    FloydWarshall(const TGraph& graph, TEdgeCostFunctor functor)
    {
        // todo: static_assert();
        Update(graph, functor);
    }

    FloydWarshall() = default;

private:
    // todo: replace these with a matrix
    std::vector<std::vector<CostType>> mCostMatrix{};
    std::vector<std::vector<VertexIdType>> mPolicyMatrix{};

public:
    template <typename TGraph, typename TEdgeCostFunctor>
    void Update(const TGraph& graph, TEdgeCostFunctor functor)
    {
        // todo: use Matrix from vic::linalg
        // initialize
        const auto n = graph.NumVertices();
        constexpr CostType maxval = std::numeric_limits<CostType>::max() / 4.;
        mCostMatrix = InitializeEmpty<CostType>(maxval, n, n);
        mPolicyMatrix = InitializeEmpty<VertexIdType>(n, n);

        const auto vertices = VertexIterator(graph);

        // vertex to itself costs zero
        for(const auto& id : vertices)
        {
            mCostMatrix[id][id] = 0.;
            mPolicyMatrix[id][id] = id;
        }

        // todo: fix for when 2 edges go between the same vertices

        // set value of the direct edges
        for(const auto& [source, sink] : EdgeIterator(graph))
        {
            mCostMatrix[source][sink] = functor(source, sink);
            mPolicyMatrix[source][sink] = sink;
            if constexpr(!directed)
            {
                mCostMatrix[sink][source] = functor(sink, source);
                mPolicyMatrix[sink][source] = source;
            }
        }

        for(const auto& vk : vertices)
        {
            for(const auto& vi : vertices)
            {
                for(const auto& vj : vertices)
                {
                    const double sum_ik_kj = mCostMatrix[vi][vk] + mCostMatrix[vk][vj];
                    if(sum_ik_kj < mCostMatrix[vi][vj])
                    {
                        mCostMatrix[vi][vj] = sum_ik_kj;
                        mPolicyMatrix[vi][vj] = mPolicyMatrix[vi][vk];
                    }
                }
            }
        }
    }

    CostType Cost(const VertexIdType source, const VertexIdType sink) const { return mCostMatrix[source][sink]; }

    VertexIdType Policy(const VertexIdType source, const VertexIdType sink) const { return mPolicyMatrix[source][sink]; }

    void PolicyPath(std::vector<VertexIdType>& path, const VertexIdType source, const VertexIdType sink) const
    {
        path.clear();
        path.push_back(source);
        while(path.back() != sink)
            path.push_back(mPolicyMatrix[path.back()][sink]);
    }

    std::vector<VertexIdType> PolicyPath(const VertexIdType source, const VertexIdType sink) const
    {
        std::vector<VertexIdType> buffer;
        PolicyPath(buffer, source, sink);
        return buffer;
    }

    template <bool shareHeadTail = false>
    bool PolicyPathsConflict(const VertexIdType start1, //
                             const VertexIdType target1,
                             const VertexIdType start2,
                             const VertexIdType target2) const
    {
        VertexIdType current1 = start1;
        VertexIdType current2 = start2;

        if(current1 == current2)
            return true;

        while(current1 != target1 || current2 != target2)
        {
            const VertexIdType next1 = mPolicyMatrix[current1][target1];
            const VertexIdType next2 = mPolicyMatrix[current2][target2];

            // check for conflict, if found, return true
            if constexpr(shareHeadTail)
            {
                if(next1 == next2 || (next1 == current2 && //
                                      next2 == current1))
                    return true;
            }
            else
            {
                if(next1 == current2 || (next2 == current1 || //
                                         next1 == next2))
                    return true;
            }

            // increase current values
            current1 = next1;
            current2 = next2;
        }

        return false; // todo
    }
};

// todo: higher order FW, where we store a policy for two robots driving their own path
//
//template <typename TGraph, typename TData>
//auto PolicyPath(const TGraph& graph, //
//                const std::vector<std::vector<TData>>& fw,
//                const typename TGraph::VertexIdType start,
//                const typename TGraph::VertexIdType target)
//{
//    std::vector<typename TGraph::VertexIdType> path;
//
//    path.push_back(start);
//    while(path.back() != target && path.size() < graph.NumVertices())
//        path.push_back(fw[path.back()][target].next);
//
//    return path;
//}
//
//template <typename TVertexId, typename FWData, bool shareHeadTail = false>
//bool PolicyPathsConflict(const FWData& fw, //
//                         const TVertexId start1,
//                         const TVertexId target1,
//                         const TVertexId start2,
//                         const TVertexId target2)
//{
//    TVertexId current1 = start1;
//    TVertexId current2 = start2;
//
//    if(current1 == current2)
//        return true;
//
//    while(current1 != target1 || current2 != target2)
//    {
//        const TVertexId next1 = fw[current1][target1].next;
//        const TVertexId next2 = fw[current2][target2].next;
//
//        // check for conflict, if found, return true
//        if constexpr(shareHeadTail)
//        {
//            if(next1 == next2 || (next1 == current2 && //
//                                  next2 == current1))
//                return true;
//        }
//        else
//        {
//            if(next1 == current2 || (next2 == current1 || //
//                                     next1 == next2))
//                return true;
//        }
//
//        // increase current values
//        current1 = next1;
//        current2 = next2;
//    }
//
//    return false; // todo
//}

template <typename TPath, bool shareHeadTail = false>
bool PathsConflict(const TPath& path1, const TPath& path2)
{
    auto it1 = path1.begin();
    auto it2 = path2.begin();

    while(true)
    {
        auto next1 = std::next(it1);
        auto next2 = std::next(it2);

        if(next1 == path1.end() && next2 == path2.end())
            break;

        if(next1 == path1.end())
            next1 = it1;
        else if(next2 == path2.end())
            next2 = it2;

        if constexpr(shareHeadTail)
        {
            if(*next1 == *next2 || (*next1 == *it2 && //
                                    *next2 == *it1))
                return true;
        }
        else
        {
            if(*next1 == *it2 || (*next2 == *it1 || //
                                  *next1 == *next2))
                return true;
        }

        it1 = next1;
        it2 = next2;
    }
    return false;
}
} // namespace graph2
} // namespace vic