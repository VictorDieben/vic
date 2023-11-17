#pragma once

#include <iterator>

#include "vic/utils/concepts.h"

namespace vic
{

// 2d overlap
template <typename TMap1, typename TMap2>
    requires ConceptMap<TMap1> && ConceptMap<TMap2> && ConceptSameKey<TMap1, TMap2>
struct MapOverlapIterator
{
    using FirstIterator = typename TMap1::iterator;
    using SecondIterator = typename TMap2::iterator;

    using key_type = const typename TMap1::key_type;
    using mapped_type1 = typename FirstIterator::value_type::second_type;
    using mapped_type2 = typename SecondIterator::value_type::second_type;
    // using mapped_type = std::pair<mapped_type1, mapped_type2>;

    //
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;

    using value_type = std::tuple<const key_type&, mapped_type1&, mapped_type2&>;
    using pointer = value_type*;
    using reference = value_type&;

    value_type operator*() // note: was returning reference
    {
        return value_type{mIter1->first, mIter1->second, mIter2->second};
    }

    MapOverlapIterator(FirstIterator begin1, //
                       FirstIterator end1,
                       SecondIterator begin2,
                       SecondIterator end2)
        : mIter1(begin1)
        , mIter2(begin2)
        , mEnd1(end1)
        , mEnd2(end2)
    {
        // find first iterator pair where the key of it1 and it2 is equal.
        // if it does not exist, set both to end
        if(mIter1 == mEnd1)
            mIter2 = mEnd2;
        else if(mIter2 == mEnd2)
            mIter1 = mEnd1;
        else if(mIter1->first != mIter2->first)
            Increase();
    }

    // Prefix increment
    MapOverlapIterator& operator++()
    {
        Increase();
        return *this;
    }

    // Postfix increment
    MapOverlapIterator operator++(int)
    {
        Increase(); // todo: how to do this?
        return *this;
    }

    friend bool operator==(const MapOverlapIterator& a, const MapOverlapIterator& b) { return a.mIter1 == b.mIter1 && a.mIter2 == b.mIter2; };
    friend bool operator!=(const MapOverlapIterator& a, const MapOverlapIterator& b) { return !(operator==(a, b)); }

private:
    FirstIterator mIter1;
    SecondIterator mIter2;
    FirstIterator mEnd1;
    SecondIterator mEnd2;

    void Increase()
    {
        while(true)
        {
            if(mIter1->first < mIter2->first)
            {
                mIter1 = std::next(mIter1);
            }
            else if(mIter2->first < mIter1->first)
            {
                mIter2 = std::next(mIter2);
            }
            else
            {
                mIter1 = std::next(mIter1);
                mIter2 = std::next(mIter2);
            }

            if(mIter1 == mEnd1 || mIter2 == mEnd2)
            {
                // make sure at end, both iterators are equal to their respective end.
                mIter1 = mEnd1;
                mIter2 = mEnd2;

                return; // found end
            }
            else if(mIter1->first == mIter2->first)
                return; // found next match
        }
    }
};

template <typename TMap1, typename TMap2>
    requires ConceptSameKey<TMap1, TMap2>
struct OverlapInstance
{
    using IteratorType = MapOverlapIterator<TMap1, TMap2>;

    OverlapInstance(TMap1& map1, TMap2& map2)
        : mMap1(map1)
        , mMap2(map2)
    { }

    IteratorType begin() { return IteratorType{mMap1.begin(), mMap1.end(), mMap2.begin(), mMap2.end()}; }
    IteratorType end() { return IteratorType{mMap1.end(), mMap1.end(), mMap2.end(), mMap2.end()}; }

    IteratorType begin() const { return IteratorType{mMap1.begin(), mMap1.end(), mMap2.begin(), mMap2.end()}; }
    IteratorType end() const { return IteratorType{mMap1.end(), mMap1.end(), mMap2.end(), mMap2.end()}; }

    IteratorType cbegin() const { return IteratorType{mMap1.begin(), mMap1.end(), mMap2.begin(), mMap2.end()}; }
    IteratorType cend() const { return IteratorType{mMap1.end(), mMap1.end(), mMap2.end(), mMap2.end()}; }

private:
    TMap1& mMap1;
    TMap2& mMap2;
};

template <typename TMap1, typename TMap2>
    requires ConceptSameKey<TMap1, TMap2>
auto Overlap(TMap1& map1, TMap2& map2)
{
    return OverlapInstance(map1, map2);
}

} // namespace vic