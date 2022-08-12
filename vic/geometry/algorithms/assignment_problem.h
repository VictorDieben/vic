#pragma once

#include "vic/utils.h"
#include <concepts>
#include <unordered_set>
#include <utility>
#include <vector>

// todo: move to some other folder, not really part of geometry
namespace vic
{
namespace geom
{

//
template <typename T, typename Functor>
std::vector<std::pair<std::size_t, std::size_t>> GroupPairsOfTwo(const std::vector<T>& vec, Functor functor)
{
    // todo: what do we want to do when size of vec is odd?

    using FunctorType = decltype(functor(T{}, T{}));

    struct HeapObject
    {
        FunctorType value;
        std::size_t first;
        std::size_t second;
    };

    std::vector<HeapObject> heap;

    // brute force solution first. We add all possible combinations of 2 items in vec.
    // make sure we can store (vec.size()^2)/2 items
    // todo: a better solution would be to only add the best x combinations for each of the elements
    if(vec.size() < 2 || vec.size() % 2 != 0)
        return {};

    for(std::size_t i = 0; i < vec.size() - 1; i++)
        for(std::size_t j = i + 1; j < vec.size(); j++)
            heap.push_back(HeapObject{functor(vec.at(i), vec.at(j)), i, j});

    const auto heapFunctor = [&](const HeapObject& left, const HeapObject& right) {
        return left.value > right.value; //
    };

    std::make_heap(heap.begin(), heap.end(), heapFunctor);
    std::unordered_set<std::size_t> selected;
    std::vector<std::pair<std::size_t, std::size_t>> result{};

    // pop heap, if both indices are not yet taken, add this to the result
    HeapObject current;
    while(!heap.empty())
    {
        if(selected.size() == vec.size())
            break;

        // pick the lowest value out of heap
        std::pop_heap(heap.begin(), heap.end(), heapFunctor);
        current = heap.back();
        heap.pop_back();

        if(selected.contains(current.first) || selected.contains(current.second))
            continue;

        result.push_back({current.first, current.second});
        selected.insert(current.first);
        selected.insert(current.second);
    }

    return result;
}

} // namespace geom
} // namespace vic