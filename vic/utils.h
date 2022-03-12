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

//// example:
//// struct SomeCountedClass : public Counted<SomeCountedClass> {};
//// SomeCountedClass::GetCount();
//template <typename T>
//class Counted
//{
//public:
//    Counted() { mCount++; }
//    Counted(const Counted<T>& other) { mCount++; }
//    Counted(Counted<T>&& other) noexcept { mCount++; }
//    ~Counted() { mCount--; }
//
//private:
//    static inline std::size_t mCount{0};
//};

// Integer exponent
// TODO(vicdie): there are far more efficient algorithms for this,
// let the compiler figure it out by just nesting the template
template <typename TRet, typename TBase, typename TExp>
TRet Power(const TBase base, const TExp exp)
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

// base conversion (e.g. decimals to hex)
// TODO(vicdie): make a version that works on a range of iterators
template <typename TBase, typename TIn, typename TOut>
std::vector<TOut>& ToBase(std::vector<TOut>& buffer, const TIn value, const TBase base)
{
    const std::size_t size = buffer.size();
    TIn val = value; // mutable copy
    buffer.clear();
    while(val)
    {
        buffer.emplace_back(value % base);
        value /= base;
    }
    if(buffer.size() < size)
        buffer.resize(size); // appends T{0}'s to the end of the list
    return buffer;
}

template <typename TBase, typename TIn, typename TOut>
std::vector<TOut> ToBase(const TIn value, const TBase base)
{
    std::vector<TOut> result; // TODO(vicdie): initialize to good value
    ToBase<TBase, TIn, TOut>(result, value, base);
    return result;
}

// convert a list of numbers in a certain base back to normal representation
// TODO(vicdie): make a version that works with an iterator range instead of vector
template <typename TBase, typename TIn, typename TOut>
TOut FromBase(const std::vector<TIn>& values, const TBase base)
{
    // TODO(vicdie): assert no value in values is larger than base?
    uint64_t tmp = 1;
    TOut ret{0};
    for(const auto& value : values)
    {
        ret = ret + (tmp * value);
        tmp = tmp * base;
    }
    return ret;
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