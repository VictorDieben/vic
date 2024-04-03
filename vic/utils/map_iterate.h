#pragma once

#include <iterator>
#include <optional>

#include "vic/utils/concepts.h"

namespace vic
{

// 2d overlap

template <typename TKey, typename TValue1, typename TValue2>
struct MapOverlapValueType
{
    MapOverlapValueType(const TKey& key, TValue1& value1, TValue2& value2)
        : key(key)
        , first(value1)
        , second(value2)
    { }
    MapOverlapValueType(const MapOverlapValueType&) = default;

    const TKey& key;
    TValue1& first;
    TValue2& second;
};

template <typename TMap1, typename TMap2>
    requires ConceptMap<TMap1> && ConceptMap<TMap2> && ConceptSameKey<TMap1, TMap2>
struct MapOverlapIterator
{
    using FirstIterator = typename TMap1::iterator;
    using SecondIterator = typename TMap2::iterator;

    using key_type = std::remove_cv_t<typename TMap1::key_type>;
    using mapped_type1 = std::remove_cv_t<typename FirstIterator::value_type::second_type>;
    using mapped_type2 = std::remove_cv_t<typename SecondIterator::value_type::second_type>;
    // using mapped_type = std::pair<mapped_type1, mapped_type2>;

    //
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;

    // using value_type = std::tuple<key_type&, mapped_type1&, mapped_type2&>;
    using value_type = MapOverlapValueType<key_type, mapped_type1, mapped_type2>;

    using pointer = value_type*;
    using reference = value_type&;

    value_type mPlaceholder;

    reference operator*() // note: was returning reference
    {
        // assert(mPlaceholder.has_value());
        // NOTE/TODO/FIXME: this has no right to work, and yet it does.
        //mPlaceholder = std::make_tuple(std::ref(mIter1->first), //
        //                               std::ref(mIter1->second),
        //                               std::ref(mIter2->second));
        //mPlaceholder = value_type(mIter1->first, //
        //                          mIter1->second,
        //                          mIter2->second);
        // std::get<0>(mPlaceholder) = mIter1->first;
        auto tmp = value_type(mIter1->first, mIter1->second, mIter2->second);

        mPlaceholder = tmp;
        return mPlaceholder;
    }

    //pointer operator->()
    //{
    //    // mPlaceholder = value_type{mIter1->first, mIter1->second, mIter2->second};
    //    return &mPlaceholder;
    //}

    MapOverlapIterator(FirstIterator begin1, //
                       FirstIterator end1,
                       SecondIterator begin2,
                       SecondIterator end2)
        : mIter1(begin1)
        , mIter2(begin2)
        , mEnd1(end1)
        , mEnd2(end2)
        //, mPlaceholder(declval(FirstIterator::value_type::first_type{}), //
        //               declval(FirstIterator::value_type::second_type{}),
        //               declval(SecondIterator::value_type::second_type{}))
        , mPlaceholder(*(key_type*)nullptr, *(mapped_type1*)nullptr, *(mapped_type2*)nullptr)
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

    using Iter1 = typename TMap1::iterator;
    using Iter2 = typename TMap2::iterator;

    OverlapInstance(TMap1& map1, TMap2& map2)
        : mBegin1(map1.begin())
        , mEnd1(map1.end())
        , mBegin2(map2.begin())
        , mEnd2(map2.end())
    { }

    OverlapInstance(Iter1& begin1, //
                    Iter1& end1,
                    Iter2& begin2,
                    Iter2& end2)
        : mBegin1(begin1)
        , mEnd1(end1)
        , mBegin2(begin2)
        , mEnd2(end2)
    { }

    IteratorType begin() { return IteratorType{mBegin1, mEnd1, mBegin2, mEnd2}; }
    IteratorType end() { return IteratorType{mEnd1, mEnd1, mEnd2, mEnd2}; }

    IteratorType begin() const { return IteratorType{mBegin1, mEnd1, mBegin2, mEnd2}; }
    IteratorType end() const { return IteratorType{mEnd1, mEnd1, mEnd2, mEnd2}; }

    IteratorType cbegin() const { return IteratorType{mBegin1, mEnd1, mBegin2, mEnd2}; }
    IteratorType cend() const { return IteratorType{mEnd1, mEnd1, mEnd2, mEnd2}; }

private:
    Iter1 mBegin1;
    Iter1 mEnd1;
    Iter2 mBegin2;
    Iter2 mEnd2;
};

template <typename TMap1, typename TMap2>
    requires ConceptSameKey<TMap1, TMap2>
auto Overlap(TMap1& map1, TMap2& map2)
{
    return OverlapInstance(map1, map2);
}

} // namespace vic