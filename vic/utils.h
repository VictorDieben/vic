#pragma once

#include <chrono>
#include <vector>

namespace vic
{

class CTimer
{
public:
    CTimer()
        : mStart(std::chrono::high_resolution_clock::now())
    { }

    using ClockType = std::chrono::high_resolution_clock;

    template <typename T, typename R>
    std::chrono::duration<T, R> GetTime() const
    {
        const auto now = ClockType::now();
        return std::chrono::duration_cast<std::chrono::duration<T, R>>(now - mStart);
    }

    void Reset() { mStart = std::chrono::high_resolution_clock::now(); }

private:
    std::chrono::time_point<ClockType> mStart;
};

// example:
// struct SomeCountedClass : public Counted<SomeCountedClass> {};
// SomeCountedClass::GetCount();
template <typename T>
class Counted
{
public:
    Counted() { mCount++; }
    Counted(const Counted<T>& other) { mCount++; }
    Counted(Counted<T>&& other) noexcept { mCount++; }
    ~Counted() { mCount--; }
    static std::size_t GetCount() { return mCount; }

private:
    static inline std::size_t mCount{0};
};

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
    if(val < 0.)
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

// base conversion (e.g. normal representation to hex)
// todo: make a version that works on a range of iterators
template <typename TVec, typename TValue, typename TBase>
void ToBase(const TValue value, const TBase base, std::vector<TVec>& buffer)
{
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

} // namespace vic