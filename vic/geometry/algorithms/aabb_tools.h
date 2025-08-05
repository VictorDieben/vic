#pragma once

#include "vic/linalg/matrices/matrix.h"

namespace vic
{
namespace geom
{

// calculate the overlap of two intervals.
// min > max, if the two intervals do not overlap
template <typename T>
constexpr Interval<T> Overlap(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return Interval<T>{Max(interval1.min, interval2.min), Min(interval1.max, interval2.max)};
}

template <typename T>
constexpr bool Overlaps(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return (interval1.min <= interval2.max) && (interval2.min <= interval1.max);
}

template <typename T, std::size_t dims>
constexpr bool Overlaps(const AABB<T, dims>& aabb1, const AABB<T, dims>& aabb2)
{
    for(std::size_t i = 0; i < dims; ++i)
        if(!Overlaps(aabb1.intervals.at(i), aabb2.intervals.at(i)))
            return false;
    return true;
}

// return if interval 2 is completely enveloped by interval 1
template <typename T>
constexpr bool Includes(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return (interval1.min <= interval2.min) && (interval1.max >= interval2.max);
}

// return if AABB 2 is completely enveloped by AABB 1
template <typename T, std::size_t dims>
constexpr bool Includes(const AABB<T, dims>& aabb1, const AABB<T, dims>& aabb2)
{
    for(std::size_t i = 0; i < dims; ++i)
        if(!Includes(aabb1.intervals.at(i), aabb2.intervals.at(i)))
            return false;
    return true;
}

template <typename T>
constexpr Interval<T> Combine(const Interval<T>& interval1, const Interval<T>& interval2)
{
    return Interval<T>{Min(interval1.min, interval2.min), Max(interval1.max, interval2.max)};
}

template <typename T, std::size_t dims>
constexpr AABB<T, dims> Combine(const AABB<T, dims>& aabb1, const AABB<T, dims>& aabb2)
{
    AABB<T, dims> aabb{};
    for(std::size_t i = 0; i < dims; ++i)
        aabb.intervals[i] = Combine(aabb1.intervals.at(i), aabb2.intervals.at(i));
    return aabb;
}

template <typename T>
constexpr T Volume(const Interval<T>& interval)
{
    return interval.max - interval.min;
}

template <typename T, std::size_t dims>
constexpr T Volume(const AABB<T, dims>& aabb)
{
    T volume{1.};
    for(std::size_t i = 0; i < dims; ++i)
        volume *= Volume(aabb.intervals[i]);
    return volume;
}

template <typename T>
constexpr T Center(const Interval<T>& interval)
{
    return (interval.min + interval.max) / 2.;
}

template <typename T>
constexpr T Extent(const Interval<T>& interval)
{
    return (interval.max - interval.min) / 2.;
}

template <typename T>
constexpr Interval<T> ToInterval(const T center, const T extent)
{
    return Interval<T>{center - extent, center + extent};
}

template <typename T, std::size_t dims>
constexpr linalg::VectorN<T, dims> Center(const AABB<T, dims>& aabb)
{
    linalg::VectorN<T, dims> center{};
    for(std::size_t i = 0; i < dims; ++i)
        center[i] = Center(aabb.intervals[i]);
    return center;
}

template <typename T, std::size_t dims>
constexpr linalg::VectorN<T, dims> Extent(const AABB<T, dims>& aabb)
{
    linalg::VectorN<T, dims> extent{};
    for(std::size_t i = 0; i < dims; ++i)
        extent[i] = Extent(aabb.intervals[i]);
    return extent;
}

template <typename T, std::size_t dims>
constexpr linalg::VectorN<T, dims> CenterExtentToAABB(const linalg::VectorN<T, dims>& center, //
                                                      const linalg::VectorN<T, dims>& extent)
{
    AABB<T, dims> aabb{};
    for(std::size_t i = 0; i < dims; ++i)
        aabb.intervals[i] = ToInterval(center[i], extent[i]);
    return aabb;
}

template <typename T, std::size_t dims>
constexpr linalg::VectorN<T, dims> MinMaxToAABB(const linalg::VectorN<T, dims>& min, //
                                                const linalg::VectorN<T, dims>& max)
{
    AABB<T, dims> aabb{};
    for(std::size_t i = 0; i < dims; ++i)
        aabb.intervals[i] = Interval(min[i], max[i]);
    return aabb;
}

template <typename T, std::size_t dims>
constexpr AABB<T, dims> Rotate(const AABB<T, dims>& aabb, const linalg::Matrix3<T>& rotation)
{
    const auto center = Center(aabb);
    const auto extent = Extent(aabb);

    AABB<T, dims> result = aabb;
    linalg::VectorN<T, dims> newExtent{};

    for(std::size_t i = 0; i < dims; ++i)
    {
        for(std::size_t j = 0; j < dims; ++j)
        {
            newExtent[i] += std::abs(rotation.Get(i, j) * extent[j]);
        }
    }

    // todo: https://x.com/Herschel/status/1188613724665335808/photo/2

    return CenterExtentToAABB(center, newExtent);
}

} // namespace geom
} // namespace vic