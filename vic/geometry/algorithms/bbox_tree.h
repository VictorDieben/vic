#pragma once

#include "vic/geometry/geometry.h"
#include "vic/utils.h"

namespace vic
{
namespace geom
{

template <typename T, std::size_t dims>
using BBox = CubeAxisAligned<T, dims>;

template <typename T>
constexpr bool Overlaps(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return (interval1.min <= interval2.max) && (interval2.min <= interval1.max);
}

template <typename T, std::size_t dims>
constexpr bool Overlaps(const BBox<T, dims>& bbox1, const BBox<T, dims>& bbox2)
{
    for(std::size_t i = 0; i < dims; ++i)
        if(!Overlaps(bbox1.intervals.at(i), bbox2.intervals.at(i)))
            return false;
    return true;
}

template <typename T>
constexpr bool Includes(const Interval<T>& interval1, const Interval<T>& interval2)
{
    // return if interval 2 is completely enveloped by interval 1
    return (interval1.min <= interval2.min) && (interval1.max >= interval2.max);
}

template <typename T, std::size_t dims>
constexpr bool Includes(const BBox<T, dims>& bbox1, const BBox<T, dims>& bbox2)
{
    for(std::size_t i = 0; i < dims; ++i)
        if(!Includes(bbox1.intervals.at(i), bbox2.intervals.at(i)))
            return false;
    return true;
}

template <typename T>
constexpr Interval<T> Combine(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return Interval<T>{Min(interval1.min, interval2.min), Max(interval1.max, interval2.max)};
}

template <typename T, std::size_t dims>
constexpr BBox<T, dims> Combine(const BBox<T, dims>& bbox1, const BBox<T, dims>& bbox2)
{
    BBox<T, dims> bbox{};
    for(std::size_t i = 0; i < dims; ++i)
        bbox.intervals[i] = Combine(bbox1.intervals.at(i), bbox2.intervals.at(i));
    return bbox;
}

// http://delab.csd.auth.gr/papers/TRSurveyRtree03_mnpt.pdf
// http://www-db.deis.unibo.it/courses/SI-LS/papers/Gut84.pdf

template <typename TKey, std::size_t dims, typename TLambda>
class BBoxTree
{
    constexpr static std::size_t Dims = dims;
    using Box = BBox<double, dims>;

    TLambda mLambda;
    std::vector<std::pair<TKey, Box>> mData{};

public:
    BBoxTree(TLambda lambda)
        : mLambda(lambda)
    { }

    void Insert(const TKey key)
    {
        //
    }

    void Remove(const TKey key)
    {
        //
    }

    Box GetBox(const TKey key) { return mLambda(key); }

private:
};

//template <typename TKey, std::size_t dims, typename TLambda>
//auto GetTree(TLambda lambda)
//{
//    return BBoxTree{lambda};
//}

} // namespace geom
} // namespace vic