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

} // namespace vic