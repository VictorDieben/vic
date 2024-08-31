#pragma once

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

} // namespace geom
} // namespace vic