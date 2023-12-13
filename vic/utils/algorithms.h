#pragma once

#include <iterator>
#include <type_traits>

#include "vic/utils/concepts.h" // less_than_comparable

namespace vic
{

// todo: similar to std::unique, move any value that occurs more than once in the array to the end,
// return an iterator to one-past-the-last of the new list of once-occurring items
template <typename TIter, typename TEquality>
    requires std::forward_iterator<TIter>
TIter remove_duplicates(TIter begin, TIter end, TEquality lambda)
{
    // todo: limit to forward and backward iterable, lambda which returns a bool
    auto it = begin;
    auto itWrite = begin;

    while(it != end)
    {
        // find the first item in range that is not equal to itFront
        auto itTmp = std::next(it);
        while(itTmp != end && lambda(*it, *itTmp))
            itTmp++;

        if(it + 1 == itTmp) // no duplicates
        {
            *itWrite = *it;

            it++;
            itWrite++;
        }
        else // found duplicates, forward it, but not itWrite
        {
            if(itTmp == end)
                return itWrite;
            else
            {
                it = itTmp;
            }
        }
    }

    return itWrite;
}

// re-sort 1 value in an otherwise sorted array
template <typename TIter, typename TCompare>
    requires std::random_access_iterator<TIter>
TIter sort_individual(TIter begin, //
                      TIter end,
                      TIter item,
                      TCompare lambda)
{
    if(begin == end || item == end)
        return end;

    auto itNewPos = std::lower_bound(begin, end, *item, lambda);
    if(itNewPos == item)
        return item;

    if(itNewPos < item)
    {
        std::rotate(itNewPos, item, item + 1);
        return itNewPos;
    }
    else
    {
        std::rotate(item, item + 1, itNewPos);
        return itNewPos - 1;
    }
}

// re-sort 1 value in an otherwise sorted array
template <typename TIter>
    requires std::random_access_iterator<TIter>
TIter sort_individual(TIter begin, //
                      TIter end,
                      TIter item)
{
    return sort_individual(begin, end, item, std::less<>{});
}

template <typename TBegin, typename TEnd>
    requires std::forward_iterator<TBegin> && std::forward_iterator<TEnd>
auto flatten(TBegin begin, TEnd end)
{
    using TData = decltype(std::begin(*begin));
    std::vector<TData> data;

    // calculate size
    std::size_t size = 0;
    for(auto it = begin; it != end; ++it)
        size += it->size();

    data.reserve(size);

    std::size_t size = 0;
    for(auto it = begin; it != end; ++it)
        for(auto it2 = std::begin(*it); it2 != std::end(*it); ++it2)
            data.push_back(*it2);

    return data;
}

} // namespace vic