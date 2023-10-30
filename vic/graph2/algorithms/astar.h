#pragma once

#include "vic/linalg/linalg.h"

#include "vic/graph2/algorithms/iterator.h"
#include "vic/graph2/graph_types/cartesian_product_graph.h"
#include "vic/memory/flat_set.h"

#include <algorithm>
#include <map>
#include <set>

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

template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
struct CartesianAStarInstance
{
public:
    using VertexIdType = typename TGraph::VertexIdType;
    using EdgeIdType = typename TGraph::EdgeIdType;

    using CartesianVertexIdType = uint64_t;
    using CartesianVertexType = std::vector<VertexIdType>;
    // using CartesianEdgeType = std::vector<EdgeIdType>;

    CartesianAStarInstance(const TGraph& graph, //
                           TEdgeCostFunctor edgeCostFunctor,
                           THeuristicFunctor heuristicFunctor)
        : mGraph(graph)
        , mEdgeCostFunctor(edgeCostFunctor)
        , mHeuristicFunctor(heuristicFunctor)
        , mOutIterator(graph)
    { }

private:
    const TGraph& mGraph;

    TEdgeCostFunctor mEdgeCostFunctor;
    THeuristicFunctor mHeuristicFunctor;

    BaseOutVertexIterator<TGraph> mOutIterator;

public:
    using CostType = decltype(mEdgeCostFunctor(CartesianVertexType{}, CartesianVertexType{}));
    using HeuristicType = decltype(mHeuristicFunctor(CartesianVertexType{}, CartesianVertexType{}));
    // static_assert(std::is_same_v<CostType, HeuristicType>);

    struct ExploredObject
    {
        CartesianVertexIdType vertex{}; // previous
        CostType f{std::numeric_limits<CostType>::max()};
        CostType g{std::numeric_limits<CostType>::max()};
    };

    auto Run(const CartesianVertexType& start, const CartesianVertexType& target)
    {
        assert(start.size() == target.size());

        const auto numVertices = mGraph.NumVertices();
        const auto dims = start.size();

        const auto cartesianOutIterator = CartesianOutIterator(mGraph, mOutIterator);

        std::vector<CartesianVertexIdType> heap;

        const auto startId = ToId<CartesianVertexIdType>(start, numVertices);
        const auto targetId = ToId<CartesianVertexIdType>(target, numVertices);
        heap.push_back(startId);

        std::set<CartesianVertexIdType> closedSet;
        std::map<CartesianVertexIdType, ExploredObject> exploredMap;
        exploredMap[startId] = ExploredObject{startId, 0., 0.};

        // note: > because default make_heap behaviour is max heap for operator<
        const auto compareF = [&](const CartesianVertexIdType v1, const CartesianVertexIdType v2) { return exploredMap.at(v1).f > exploredMap.at(v2).f; };

        CartesianVertexType currentBuffer;

        CartesianVertexIdType currentId = startId;
        while(!heap.empty())
        {
            std::pop_heap(heap.begin(), heap.end(), compareF);
            currentId = heap.back();
            heap.pop_back();

            if(currentId == targetId)
                break;
            if(closedSet.contains(currentId))
                continue;
            closedSet.insert(currentId);

            ToVector(currentId, dims, numVertices, currentBuffer);

            // iterate over neighbours, check with best value so far
            cartesianOutIterator.ForeachValidOutVertex(currentBuffer, [&](const CartesianVertexType& other) {
                const auto otherId = ToId<CartesianVertexIdType>(other, numVertices);

                const auto edgeCost = mEdgeCostFunctor(currentBuffer, other);
                const auto newGScore = exploredMap[currentId].g + edgeCost;

                auto& item = exploredMap[otherId]; // adds item if it did not exist

                if(newGScore < item.g)
                {
                    const CostType hscore = mHeuristicFunctor(other, target);
                    item = {currentId, newGScore + hscore, newGScore};

                    heap.push_back(otherId);
                    std::push_heap(heap.begin(), heap.end(), compareF);
                }
            });
        }

        if(currentId != targetId)
            return std::vector<CartesianVertexType>{};

        std::vector<CartesianVertexType> path;
        path.push_back(target);

        currentId = targetId;
        while(path.size() < numVertices * 4) // note: for multi-robot, we cannot assume than number of vertices is really the upper limit, 4x should be enough
        {
            if(currentId == startId)
                break;
            auto& node = exploredMap[currentId].vertex;
            currentId = node;

            path.push_back(ToVector<VertexIdType>(currentId, dims, numVertices));
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};
//
//template <typename TGraph, typename TEdgeCostFunctor, typename THeuristicFunctor>
//auto CartesianAStar(const TGraph& graph, //
//                    TEdgeCostFunctor edgeCostFunctor,
//                    THeuristicFunctor heuristicFunctor,
//                    const std::vector<typename TGraph::VertexIdType>& start,
//                    const std::vector<typename TGraph::VertexIdType>& target)
//{
//    using GraphType = TGraph;
//    using VertexIdType = typename GraphType::VertexIdType;
//    using EdgeIdType = typename GraphType::EdgeIdType;
//
//    using CartesianVertexIdType = uint64_t;
//    using CartesianEdgeIdType = uint64_t;
//
//    using CartesianVertexType = std::vector<VertexIdType>;
//    using CartesianEdgeType = std::vector<EdgeIdType>;
//
//    using CostType = decltype(edgeCostFunctor(CartesianVertexType{}, CartesianVertexType{}));
//    using HeuristicType = decltype(heuristicFunctor(CartesianVertexType{}, CartesianVertexType{}));
//    static_assert(std::is_same_v<CostType, HeuristicType>);
//    assert(start.size() == target.size());
//
//    const auto numVertices = graph.NumVertices();
//    const auto dims = start.size();
//
//    const auto outIterator = GetOutVertexIterator(graph); // todo: pass as argument
//    const auto cartesianOutIterator = CartesianOutIterator(graph, outIterator);
//
//    struct ExploredObject
//    {
//        CartesianVertexIdType vertex{}; // previous
//        CostType f{std::numeric_limits<CostType>::max()};
//        CostType g{std::numeric_limits<CostType>::max()};
//    };
//
//    std::vector<CartesianVertexIdType> heap;
//
//    const auto startId = ToId<CartesianVertexIdType>(start, numVertices);
//    const auto targetId = ToId<CartesianVertexIdType>(target, numVertices);
//    heap.push_back(startId);
//
//    std::set<CartesianVertexIdType> closedSet;
//    std::map<CartesianVertexIdType, ExploredObject> exploredMap;
//    exploredMap[startId] = ExploredObject{startId, 0., 0.};
//
//    // note: > because default make_heap behaviour is max heap for operator<
//    const auto compareF = [&](const CartesianVertexIdType v1, const CartesianVertexIdType v2) { return exploredMap.at(v1).f > exploredMap.at(v2).f; };
//
//    CartesianVertexType currentBuffer;
//
//    CartesianVertexIdType current = startId;
//    while(!heap.empty())
//    {
//        std::pop_heap(heap.begin(), heap.end(), compareF);
//        current = heap.back();
//        heap.pop_back();
//
//        if(current == targetId)
//            break;
//        if(closedSet.contains(current))
//            continue;
//        closedSet.insert(current);
//
//        ToVector(current, dims, numVertices, currentBuffer);
//
//        // iterate over neighbours, check with best value so far
//        cartesianOutIterator.ForeachValidOutVertex(currentBuffer, [&](const CartesianVertexType& other) {
//            const auto otherId = ToId<CartesianVertexIdType>(other, numVertices);
//
//            const auto edgeCost = edgeCostFunctor(currentBuffer, other);
//            const auto newGScore = exploredMap[current].g + edgeCost;
//
//            auto& item = exploredMap[otherId]; // adds item if it did not exist
//
//            if(newGScore < item.g)
//            {
//                const CostType hscore = heuristicFunctor(other, target);
//                item = {current, newGScore + hscore, newGScore};
//
//                heap.push_back(otherId);
//                std::push_heap(heap.begin(), heap.end(), compareF);
//            }
//        });
//    }
//
//    if(current != targetId)
//        return std::vector<CartesianVertexType>{};
//
//    std::vector<CartesianVertexType> path;
//    path.push_back(target);
//
//    current = targetId;
//    while(path.size() < numVertices * 4) // note: for multi-robot, we cannot assume than number of vertices is really the upper limit, 4x should be enough
//    {
//        if(current == startId)
//            break;
//        auto& node = exploredMap[current].vertex;
//        current = node;
//
//        path.push_back(ToVector<VertexIdType>(current, dims, numVertices));
//    }
//
//    std::reverse(path.begin(), path.end());
//    return path;
//}

} // namespace graph2
} // namespace vic