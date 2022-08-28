

#pragma once

namespace vic
{
namespace sorting
{

// merge sorting algorithm
// sorts an array, where the subregions [begin; midpoint> and [midpoint; end> are already sorted.
// sorts in place, in O(m+n) steps
template <typename TIter, typename TCompare>
void merge_sort(TIter begin, TIter midpoint, TIter end, TCompare comp)
{
    if(begin >= midpoint || midpoint >= end)
        return; // data should already be sorted, or input is invalid

    TIter it1 = begin;
    TIter it2 = midpoint;

    while(true)
    {
        //
        if(comp(*it1, *it2))
        {
            // todo
            it1 = std::next(it1);
        }
        else
        {
            // todo
        }
        break;
    }
}

template <typename TIter>
void merge_sort(TIter begin, TIter midpoint, TIter end)
{
    return merge_sort(begin, midpoint, end, std::less<typename TIter::value_type>{});
}

} // namespace sorting
} // namespace vic