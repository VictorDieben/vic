#pragma once

#include <iterator>

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

    return itWrite; // todo
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

    auto itNewPos = std::lower_bound(begin, end, *item);
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

} // namespace vic