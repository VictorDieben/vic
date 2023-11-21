

#pragma once

namespace vic
{
namespace sorting
{

template <typename TIter1, typename TIter2, typename TBackInserter, typename TCompare>
void move_merge(TIter1 begin1, //
                TIter1 end1,
                TIter2 begin2,
                TIter2 end2,
                TBackInserter inserter,
                TCompare compare)
{
    // same merge algorithm as std::merge, but explicitly moves instead of copies
    while(true) //
    {
        const bool reachedEnd1 = (begin1 == end1);
        const bool reachedEnd2 = (begin2 == end2);
        if(reachedEnd1 && reachedEnd2)
            break;

        if(!reachedEnd1 && *begin1 < *begin2)
        {
            *inserter = std::move(*begin1);
            ++begin1;
        }
        else
        {
            *inserter = std::move(*begin2);
            ++begin2;
        }
        inserter++;
    }
}

template <typename TIter1, typename TIter2, typename TBackInserter>
void move_merge(TIter1 begin1, //
                TIter1 end1,
                TIter2 begin2,
                TIter2 end2,
                TBackInserter inserter)
{
    return move_merge(begin1, end1, begin2, end2, inserter, std::less{});
}

// merge sorting algorithm
// sorts an array, where the subregions [begin; midpoint> and [midpoint; end> are already sorted.
// O(n) when the entire vector is sorted, ~O(n^2) if the second half is entirely smaller than the first
template <typename TIter, typename TCompare>
void merge_sort(TIter begin, TIter midpoint, TIter end, TCompare comp)
{
    if(begin >= midpoint || midpoint >= end)
        return; // data should already be sorted, or input is invalid

    TIter it_begin = begin;
    TIter it_mid_l = std::prev(midpoint);
    TIter it_mid_r = midpoint;
    TIter it_end = std::prev(end);

    // todo: this only works for Ts that have a + operator.
    // maybe add an extra lambda?
    // const bool rotate = (*it_mid_l + *it_begin) > (*it_end + *it_mid_r);

    // If the second vector is smaller than the first:
    // swap iterator ranges, run as normal, and then rotate.
    // this way, the worst case O(n^2) now runs as O(n).

    const bool rotate = (*it_end < *it_mid_l) && (*it_mid_r < *it_begin);
    if(rotate)
    {
        std::swap(it_begin, it_mid_r);
        std::swap(it_mid_l, it_end);
    }

    while(true)
    {
        if(*it_begin <= *it_mid_r)
        {
            if(it_begin < it_mid_l)
                it_begin = std::next(it_begin);
        }
        else
        {
            std::swap(*it_begin, *it_mid_r);
            auto tmp = it_mid_r;
            while(tmp != it_end && *tmp >= *std::next(tmp))
            {
                std::swap(*tmp, *std::next(tmp));
                tmp++;
            }
        }

        if(*it_mid_l <= *it_end)
        {
            if(it_mid_r < it_end)
                it_end = std::prev(it_end);
        }
        else
        {
            std::swap(*it_mid_l, *it_end);
            auto tmp = it_mid_l;
            while(tmp != it_begin && *std::prev(tmp) >= *tmp)
            {
                std::swap(*std::prev(tmp), *tmp);
                tmp--;
            }
        }

        if(it_begin == it_mid_l && it_mid_r == it_end)
            break;
    }

    // do 1 final check for the remaining 2 items
    if(*it_mid_l > *it_mid_r)
        std::swap(*it_mid_l, *it_mid_r);

    // undo the rotation if we swapped the iterators earlier
    if(rotate)
        std::rotate(begin, midpoint, end);
}

template <typename TIter>
void merge_sort(TIter begin, TIter midpoint, TIter end)
{
    return merge_sort(begin, midpoint, end, std::less<typename TIter::value_type>{});
}

template <typename TIter, typename TBackInserter>
void merge_sort_back_insertion(const TIter begin, const TIter midpoint, const TIter end, TBackInserter backInsert)
{
    auto it1 = begin;
    auto it2 = midpoint;
    const auto end1 = midpoint;
    const auto end2 = end;
    while(true)
    {
        if(it1 == end1)
        {
            std::copy(it2, end2, backInsert);
            return;
        }
        else if(it2 == end2)
        {
            std::copy(it1, end1, backInsert);
            return;
        }

        if(*it1 < *it2)
        {
            *backInsert = *it1;
            it1++;
        }
        else
        {
            *backInsert = *it2;
            it2++;
        }
        backInsert++;
    }
}

} // namespace sorting
} // namespace vic