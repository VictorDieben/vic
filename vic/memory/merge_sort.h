

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
    while(true)
    {
        const bool reachedEnd1 = (begin1 == end1);
        const bool reachedEnd2 = (begin2 == end2);
        if(reachedEnd1 && reachedEnd2)
            break;
        else if(reachedEnd1 && !reachedEnd2)
        {
            std::move(begin2, end2, inserter);
            break;
        }
        else if(!reachedEnd1 && reachedEnd2)
        {
            std::move(begin1, end1, inserter);
            break;
        }
        else
        {
            if(compare(*begin1, *begin2))
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

} // namespace sorting
} // namespace vic