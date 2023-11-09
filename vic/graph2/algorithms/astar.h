#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"
#include "vic/graph2/graph_types/cartesian_product_graph.h"
#include "vic/memory/flat_set.h"

#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace vic
{
namespace graph2
{

namespace detail
{
// todo: put  implementation here, where iterator object is passed as argument
}

template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
auto AStar(const TGraph& graph, //
           TEdgeCostFunctor edgeCostFunctor,
           THeuristicFunctor heuristicFunctor,
           const typename TGraph::VertexIdType start,
           const typename TGraph::VertexIdType target)
{
    using GraphType = TGraph;
    using VertexIdType = typename GraphType::VertexIdType;
    using EdgeIdType = typename GraphType::EdgeIdType;

    using CostType = decltype(edgeCostFunctor(VertexIdType{}, VertexIdType{}));
    using HeuristicType = decltype(heuristicFunctor(VertexIdType{}, VertexIdType{}));
    static_assert(std::is_same_v<CostType, HeuristicType>);

    const auto outIterator = GetOutVertexIterator(graph);

    struct ExploredObject
    {
        VertexIdType vertex{};
        CostType f{std::numeric_limits<CostType>::max()};
        CostType g{std::numeric_limits<CostType>::max()};
    };

    std::vector<VertexIdType> heap;
    heap.push_back(start);

    std::set<VertexIdType> closedSet;
    std::map<VertexIdType, ExploredObject> exploredMap;
    exploredMap[start] = ExploredObject{start, 0., 0.};

    // note: > because default make_heap behaviour is max heap for operator<
    const auto compareF = [&](const VertexIdType v1, const VertexIdType v2) { return exploredMap.at(v1).f > exploredMap.at(v2).f; };

    VertexIdType current = start;
    while(!heap.empty())
    {
        std::pop_heap(heap.begin(), heap.end(), compareF);
        current = heap.back();
        heap.pop_back();

        if(current == target)
            break;
        if(closedSet.contains(current))
            continue;
        closedSet.insert(current);

        // iterate over neighbours, check with best value so far
        outIterator.ForeachOutVertex(current, [&](const VertexIdType other) {
            const auto edgeCost = edgeCostFunctor(current, other);
            const auto newGScore = exploredMap[current].g + edgeCost;

            auto& item = exploredMap[other]; // adds item if it did not exist

            if(newGScore < item.g)
            {
                const CostType hscore = heuristicFunctor(other, target);
                item = {current, newGScore + hscore, newGScore};

                heap.push_back(other);
                std::push_heap(heap.begin(), heap.end(), compareF);
            }
        });
    }

    if(current != target)
        return std::vector<VertexIdType>{};

    std::vector<VertexIdType> path;
    path.push_back(target);
    while(path.back() != start && path.size() < graph.NumVertices())
    {
        if(path.back() == start)
            break;
        auto next = exploredMap[path.back()].vertex;
        path.push_back(next);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

// hash function for std::vector<T>
template <typename T>
    requires std::integral<T>
struct VectorHasher
{
    int operator()(const std::vector<uint16_t>& vec) const
    {
        int hash = vec.size();
        for(const auto& i : vec)
        {
            hash ^= i + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

template <typename TCost, typename TGraph>
struct CartesianAStar
{
public:
    explicit CartesianAStar(const TGraph& graph)
        : mOutIterator(graph)
    { }

    CartesianAStar() = default;

    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using CartesianVertexType = CartesianVertex<VertexIdType>;

    using CostType = TCost;

private:
    BaseOutVertexIterator<TGraph> mOutIterator;

public:
    struct ExploredObject
    {
        CartesianVertexType vertex{}; // previous
        CostType f{std::numeric_limits<CostType>::max()};
        CostType g{std::numeric_limits<CostType>::max()};
    };

    using Hasher = VectorHasher<VertexIdType>;

    std::set<CartesianVertexType> mClosedSet;
    std::map<CartesianVertexType, ExploredObject> mExploredMap;
    std::vector<CartesianVertexType> mHeap;

    template <typename TEdgeCostFunctor, typename THeuristicFunctor>
    auto Run(const CartesianVertexType& start, //
             const CartesianVertexType& target,
             const TEdgeCostFunctor& edgeCostFunctor,
             const THeuristicFunctor& heuristicFunctor)
    {
        assert(start.size() == target.size());

        const auto dims = start.size();

        const auto cartesianOutIterator = CartesianOutIterator(mOutIterator);

        mHeap.push_back(start);
        mExploredMap[start] = ExploredObject{start, 0., 0.};

        // note: > because default make_heap behaviour is max heap for operator<
        const auto compareF = [&](const CartesianVertexType& v1, const CartesianVertexType& v2) { return mExploredMap.at(v1).f > mExploredMap.at(v2).f; };

        CartesianVertexType current;

        while(!mHeap.empty())
        {
            std::pop_heap(mHeap.begin(), mHeap.end(), compareF);
            std::swap(current, mHeap.back()); // avoid destructing current, just swap it to the heap
            mHeap.pop_back();

            if(current == target)
                break;
            if(mClosedSet.contains(current))
                continue;
            mClosedSet.insert(current);

            // iterate over neighbours, check with best value so far
            cartesianOutIterator.ForeachValidOutVertex(current, [&](const CartesianVertexType& other) {
                const auto edgeCost = edgeCostFunctor(current, other);
                const auto newGScore = mExploredMap[current].g + edgeCost;

                auto& item = mExploredMap[other]; // adds item if it did not exist

                if(newGScore < item.g)
                {
                    const CostType hscore = heuristicFunctor(other, target);
                    item = ExploredObject{current, newGScore + hscore, newGScore};

                    mHeap.push_back(other);
                    std::push_heap(mHeap.begin(), mHeap.end(), compareF);
                }
            });
        }

        std::vector<CartesianVertexType> path;

        if(current != target)
            return path; // return an empty path if something failed

        path.push_back(target);

        current = target;
        while(current != start) // note: for multi-robot, we cannot assume than number of vertices is really the upper limit, 4x should be enough
        {
            if(path.size() > 1000)
                return std::vector<CartesianVertexType>{};
            current = mExploredMap[current].vertex;
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());

        mClosedSet.clear();
        mExploredMap.clear();
        mHeap.clear();

        return path;
    }
};

//template <typename T, std::size_t dims>
//using CartesianArrayVertex = std::array<T, dims>:

template <typename TCost, typename TVertex, std::size_t dims>
struct CartesianArrayAStarExploredObject
{
    std::array<TVertex, dims> vertex{}; // previous
    TCost f{std::numeric_limits<TCost>::max()};
    TCost g{std::numeric_limits<TCost>::max()};
};

template <typename TCost, typename TGraph>
struct CartesianArrayAStar
{
public:
    explicit CartesianArrayAStar(const TGraph& graph)
        : mOutIterator(graph)
    { }

    CartesianArrayAStar() = default;

    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using CostType = TCost;

private:
    BaseOutVertexIterator<TGraph> mOutIterator;

public:
    template <std::size_t dims, typename TEdgeCostFunctor, typename THeuristicFunctor>
    auto Run(const std::vector<VertexIdType>& start, //
             const std::vector<VertexIdType>& target,
             const TEdgeCostFunctor& edgeCostFunctor,
             const THeuristicFunctor& heuristicFunctor) const
    {
        assert(start.size() == dims && target.size() == dims);

        using ExploredObject = CartesianArrayAStarExploredObject<TCost, VertexIdType, dims>;
        using VertexType = std::array<VertexIdType, dims>;

        std::set<VertexType> closedSet;
        std::map<VertexType, ExploredObject> exploredMap;
        std::vector<VertexType> heap;

        const auto dims = start.size();

        const auto cartesianOutIterator = CartesianOutIterator(mOutIterator);

        const auto toArray = [](const std::vector<VertexIdType>& vec) -> VertexType {
            VertexType vecToArray;
            assert(vecToArray.size() == vec.size());
            std::copy_n(vec.begin(), vecToArray.size(), vecToArray.begin());
            return vecToArray;
        };

        const auto startArray = toArray(start);
        const auto targetArray = toArray(target);

        heap.push_back(startArray);
        exploredMap[startArray] = ExploredObject{startArray, 0., 0.};

        // note: > because default make_heap behaviour is max heap for operator<
        const auto compareF = [&](const VertexType& v1, const VertexType& v2) { return exploredMap.at(v1).f > exploredMap.at(v2).f; };

        VertexType current;

        while(!heap.empty())
        {
            std::pop_heap(heap.begin(), heap.end(), compareF);
            std::swap(current, heap.back()); // avoid destructing current, just swap it to the heap
            heap.pop_back();

            if(current == targetArray)
                break;
            if(closedSet.contains(current))
                continue;
            closedSet.insert(current);

            const auto currentVector = CartesianVertex<VertexIdType>{current.begin(), current.end()};

            // iterate over neighbours, check with best value so far
            cartesianOutIterator.ForeachValidOutVertex(currentVector, [&](const auto& other) {
                // temporary fix, iterator uses an std::vector, but we work with arrays
                const VertexType otherArray = toArray(other);
                const auto edgeCost = edgeCostFunctor(currentVector, other);
                const auto newGScore = exploredMap[current].g + edgeCost;

                auto& item = exploredMap[otherArray]; // adds item if it did not exist

                if(newGScore < item.g)
                {
                    const CostType hscore = heuristicFunctor(other, target);
                    item = ExploredObject{current, newGScore + hscore, newGScore};

                    heap.push_back(otherArray);
                    std::push_heap(heap.begin(), heap.end(), compareF);
                }
            });
        }

        std::vector<VertexType> path;

        if(current != targetArray)
            return path; // return an empty path if something failed

        path.push_back(targetArray);

        current = targetArray;
        while(current != startArray) // note: for multi-robot, we cannot assume than number of vertices is really the upper limit, 4x should be enough
        {
            if(path.size() > 1000)
                return std::vector<VertexType>{};
            current = exploredMap[current].vertex;
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());

        return path;
    }
};

} // namespace graph2
} // namespace vic