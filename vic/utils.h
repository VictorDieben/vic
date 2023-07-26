#pragma once

#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace vic
{

// Finally class, useful for RAII enforced cleanup.
// Should generally only be used when working with c code.
template <typename TFunctor>
class Finally
{
public:
    Finally(TFunctor functor)
        : mFunctor(functor)
    { }
    ~Finally() { mFunctor(); }

    Finally(const Finally&) = delete;
    Finally(Finally&&) = delete;
    Finally& operator=(const Finally&) = delete;
    Finally& operator=(Finally&&) = delete;

private:
    TFunctor mFunctor;
};

// constexpr Abs
template <typename T>
constexpr T Abs(const T& val)
{
    if(val < T{0})
        return -val;
    return val;
}

// Integer exponent
template <typename TRet, typename TBase, typename TExp>
constexpr TRet Power(const TBase base, const TExp exp)
{
    TRet ret{1};
    for(TExp i = 0; i < exp; ++i)
    {
        ret = ret * base;
    }
    return ret;
}

// constexpr integer exponent
template <std::size_t exp, typename TBase, typename TRet = TBase>
constexpr TRet Pow(const TBase base)
{
    if constexpr(exp == 0)
        return 1;
    else
        return base * Pow<exp - 1>(base);
}

template <typename T>
constexpr T Min(const T& val1, const T& val2)
{
    return val1 < val2 ? val1 : val2;
}

template <typename T>
constexpr T Max(const T& val1, const T& val2)
{
    return val1 > val2 ? val1 : val2;
}

// wrapper for sqrt, so that it can be overloaded for BPI types, and a constexpr version can be added
template <typename T>
constexpr T Sqrt(const T& val)
{
    return std::sqrt(val);
}

template <typename T>
constexpr int Sign(const T val)
{
    return static_cast<int>(std::copysign(T{1}, val));
}

template <typename T>
constexpr int Signum(const T val)
{
    return ((T{0} < val) - (val < T{0})); // todo: make overload for unsigned types
}

template <typename T>
constexpr bool IsPowerOfTwo(const T val)
{
    return (val > T{}) && (!(val & (val - 1))); // check nonzero, and val bitwise and with val-1 should be 0
}

constexpr auto NextPowerOf2(const std::unsigned_integral auto n)
{
    return std::bit_ceil(n); // round up to the next power of 2
}

template <typename T>
constexpr std::pair<T, T> ModulusAndRemainder(const T i, const T divisor)
{
    return std::pair<T, T>{i % divisor, i / divisor}; // returns modulus and remainder
}

// base conversion (e.g. normal representation to hex)
// todo: make a version that works on a range of iterators
template <typename TVec, typename TValue, typename TBase>
void ToBase(const TValue value, const TBase base, std::vector<TVec>& buffer)
{
    assert(base > 0);
    const std::size_t size = buffer.size();
    TValue val = value; // mutable copy
    buffer.clear();
    while(val)
    {
        buffer.emplace_back(static_cast<TVec>(TBase{val % base}));
        val /= base;
    }
}

template <typename TReturn, typename TValue, typename TBase>
std::vector<TReturn> ToBase(const TValue value, const TBase base)
{
    std::vector<TReturn> result; // todo: initialize to good value
    ToBase(value, base, result);
    return result;
}

// convert a list of numbers in a certain base back to normal representation
// todo: make a version that works with an iterator range instead of vector
template <typename TOut, typename TVec, typename TBase>
TOut FromBase(const std::vector<TVec>& values, const TBase base)
{
    // todo: assert no value in values is larger than base?
    uint64_t tmp = 1;
    TOut ret{0};
    for(const auto& value : values)
    {
        assert(0 <= value && value < base);
        ret = ret + TOut{tmp} * value;
        tmp = tmp * base;
    }
    return TOut{ret};
}

template <typename T>
std::vector<std::vector<T>> InitializeEmpty(const T value, const std::size_t size1, const std::size_t size2)
{
    std::vector<std::vector<T>> result;
    for(std::size_t i = 0; i < size1; ++i)
    {
        std::vector<T> newvec(size2, value);
        result.push_back(newvec);
    }
    return result;
}

template <typename T>
std::vector<std::vector<T>> InitializeEmpty(const std::size_t size1, const std::size_t size2)
{
    return InitializeEmpty(T{}, size1, size2);
}

template <typename T>
constexpr void Linspace(const T& begin,
                        const T& end, //
                        typename T::value_type beginVal,
                        typename T::value_type endVal)
{
    using TVal = typename T::value_type;
    const auto n = std::distance(begin, end);
    if(n == 0)
        return;
    if(n == 1)
    {
        *begin = beginVal;
        return;
    }
    const TVal delta = (endVal - beginVal) / static_cast<TVal>(n - 1);
    std::size_t i = 0;
    for(auto it = begin; it < end; ++it)
    {
        *it = beginVal + (delta * i);
        ++i;
    }
}

template <typename T>
constexpr std::vector<T> Linspace(const T start, const T end, const std::size_t n)
{
    std::vector<T> result{};
    result.resize(n);
    Linspace(std::begin(result), std::end(result), start, end);
    return result;
}

// todo: look at the following link and fix this:
// https://committhis.github.io/2020/10/14/zip-iterator.html
template <typename T1, typename T2>
class Zip
{
public:
    Zip(const T1& range1, const T2& range2)
        : mRange1(range1)
        , mRange2(range2)
    { }

    struct ZipIterator
    {
        using iterator_category = std::random_access_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = std::pair<typename T1::value_type, typename T2::value_type>;
        using pointer = value_type*;
        using reference = value_type&;

        ZipIterator() = default;

        //reference operator*()
        //{
        //    auto val = value_type{}; //
        //    return val;
        //}
        //pointer operator->()
        //{
        //    return &mValue; //
        //}
        ZipIterator& operator++()
        {
            //mValue += mStride;
            return *this;
        }
        ZipIterator operator++(int)
        {
            ZipIterator tmp = *this;
            ++(*this);
            return tmp;
        }
        friend bool operator==(const ZipIterator& a, const ZipIterator& b) { return true; };
        friend bool operator!=(const ZipIterator& a, const ZipIterator& b) { return !operator==(a, b); };

    private:
    };

    ZipIterator begin() { return {}; }
    ZipIterator end() { return {}; }

private:
    const T1& mRange1;
    const T2& mRange2;
};

// allows us to write:
// for (const auto& val : Range(0, 10))
template <typename T>
class Range
{
public:
    Range(T end)
        : Range(0, end, 1)
    { }

    Range(T begin, T end)
        : Range(begin, end, 1)
    { }

    Range(T begin, T end, T stride)
        : mBegin(begin)
        , mEnd(end)
        , mStride(stride)
    {
        assert(stride != 0);
    }

    struct RangeIterator
    {
        using iterator_category = std::random_access_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = T*;
        using reference = T&;

        RangeIterator() = default;
        RangeIterator(T value, T end, T stride)
            : mValue(value)
            , mEnd(end)
            , mStride(stride)
        {
            assert(mStride != 0);
        }

        reference operator*()
        {
            return mValue; //
        }
        pointer operator->()
        {
            return &mValue; //
        }
        RangeIterator& operator++()
        {
            mValue += mStride;
            return *this;
        }
        RangeIterator operator++(int)
        {
            RangeIterator tmp = *this;
            ++(*this);
            return tmp;
        }
        friend bool operator==(const RangeIterator& a, const RangeIterator& b)
        {
            if(a.mEnd != b.mEnd || a.mStride != b.mStride)
                return false;
            if(a.mValue == b.mValue)
                return true; // same value is always equal
            if(a.mStride > 0)
            {
                if(a.mValue >= a.mEnd && b.mValue >= b.mEnd)
                    return true;
            }
            else
            {
                if(a.mValue <= a.mEnd && b.mValue <= b.mEnd)
                    return true;
            }
            return false;
        };
        friend bool operator!=(const RangeIterator& a, const RangeIterator& b)
        {
            return !operator==(a, b); //
        };

    private:
        T mValue{};
        T mEnd{};
        T mStride{};
    };

    RangeIterator begin() { return RangeIterator{mBegin, mEnd, mStride}; }
    RangeIterator end() { return RangeIterator{mEnd, mEnd, mStride}; }

private:
    T mBegin{};
    T mEnd{};
    T mStride{};
};

// based on:
// https://stackoverflow.com/questions/6167598/why-was-pair-range-access-removed-from-c11
template <class TIterator>
struct IteratorRange
{
    IteratorRange(TIterator begin, TIterator end)
        : mBegin(begin)
        , mEnd(end)
    { }
    IteratorRange(const std::pair<TIterator, TIterator>& pair)
        : mBegin(pair.first)
        , mEnd(pair.second)
    { }
    TIterator begin() const { return mBegin; }
    TIterator end() const { return mEnd; }

private:
    TIterator mBegin;
    TIterator mEnd;
};

template <class TIterator>
inline IteratorRange<TIterator> AsRange(std::pair<TIterator, TIterator> const& x)
{
    return IteratorRange<TIterator>(x);
}

//
} // namespace vic