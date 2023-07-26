#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <optional>
#include <vector>

namespace vic
{
namespace geom
{

std::size_t FromBinaryIndex(const std::size_t i, const std::size_t capacity)
{
    assert(capacity % 2 == 0);
    if(i % 2) // right child
    {
        const auto halfCapacity = capacity / 2;
        return halfCapacity + ((i - 1) / 2);
    }
    else // left child
        return i / 2;
}
std::size_t ToBinaryIndex(const std::size_t i, const std::size_t capacity)
{
    assert(IsPowerOfTwo(capacity));
    return {}; //
}

template <typename T>
constexpr uint64_t BinaryTreeDepthSize(const uint64_t level)
{
    return {};
}

template <typename T>
constexpr uint64_t BinaryTreeFullSize(const uint64_t level)
{
    return {};
}

template <typename T, typename TLambda>
struct BinaryTreeVector
{
    enum class InsertStatus
    {
        Ok,
        Resize
    };

    BinaryTreeVector(TLambda lambda)
        : mLambda(lambda)
    {
        mData.reserve(HeapSize(mHeapSizeLevel)); //
    }

    bool IsFilled(const std::size_t i) const
    {
        if(i < capacity() / 2)
            return true;
        return false;
    }

    T& at(std::size_t i) { return mData.at(i); }
    const T& at(std::size_t i) const { return mData.at(i); }

    // similar to vector::push_back()
    void push_back(const T& value)
    {
        assert(IsPowerOfTwo(capacity())); // check should not be needed, but lets keep it for now
        if(size() < capacity())
        {
            // just push back
            mData.push_back(value);
        }
        else
        {
            // shift up. then push back
            mData.reserve(capacity() * 2);
        }
        std::sort(mData.begin(), mData.end(), mLambda);
    }

    InsertStatus Insert(const T& value) { return {}; }

    std::size_t capacity() const { return mData.capacity(); }
    std::size_t size() const { return mData.size(); }

    std::size_t HeapSize(const std::size_t level) const { return Power<std::size_t>(2, level); }

private:
    std::vector<T> mData;
    const TLambda mLambda;

    std::size_t mHeapSizeLevel{1};
};

enum class EBorder
{
    Begin,
    End
};

template <typename TKey, typename TLambda>
class IntervalSorted
{

    struct Item
    {
        TKey key;
        EBorder border{EBorder::Begin};
    };

public:
    using TInterval = Interval<double>; // todo: get Interval type from lambda
    using TIntervalType = double; // todo

    IntervalSorted(TLambda lambda)
        : mLambda(lambda)
    { }

    // both adds key, and pushes it into the heap
    void Insert(const TKey key) { Resort(); }

    // add key, but do not resort
    void Add(const TKey key)
    {
        mList.push_back({key, EBorder::Begin}); //
        mList.push_back({key, EBorder::End});
    }

    bool Remove(const TKey key, const bool resort = true)
    {
        return false; //
    }

    void Resort()
    {
        std::sort(mList.begin(), mList.end(), [&](const auto& item1, const auto& item2) {
            const auto& interval1 = mLambda(item1.key);
            const auto& interval2 = mLambda(item2.key);
            return ((item1.border == EBorder::Begin) ? interval1.min : interval1.max) < //
                   ((item2.border == EBorder::Begin) ? interval2.min : interval2.max);
        });
    }

    void Overlap(const TInterval& interval, std::vector<TKey>& output) const
    {
        //
    }

private:
    TLambda mLambda;

    std::vector<Item> mList{};
};

template <typename T>
constexpr T HeapParentIndex(const T idx)
{
    return (idx - 1) / 2; //
}

template <typename T>
constexpr T HeapLeftChild(const T idx)
{
    return (2 * idx) + 1;
}

template <typename T>
constexpr T HeapRightChild(const T idx)
{
    return (2 * idx) + 2;
}

// An object that wraps a forward and backward heap for interval objects.
// can be used to create n-dimensional box trees.
// NOTE: performance is quite disappointing, no better than brute force
template <typename TKey, typename TLambda>
class IntervalHeap
{
public:
    using TInterval = Interval<double>; // todo: get Interval type from lambda
    using TIntervalType = double; // todo

    IntervalHeap(TLambda lambda)
        : mLambda(lambda)
    { }

    // both adds key, and pushes it into the heap
    void Insert(const TKey key)
    {
        mHeap.push_back(key);
        Resort();
    }

    // add key, but do not resort
    void Add(const TKey key) { mHeap.push_back(key); }

    //
    bool Remove(const TKey key, const bool resort = true)
    {
        return false; //
    }

    void Resort()
    {
        // make a heap based on interval start
        std::make_heap(mHeap.begin(),
                       mHeap.end(), //
                       [&](const auto k1, const auto k2) { return mLambda(k1).min > mLambda(k2).min; });
        // iterate over the heap backwards, store max value of subheap
        mHeapIntervalMax.clear();
        mHeapIntervalMax.resize(mHeap.size());
        for(auto& item : mHeapIntervalMax)
            item = std::numeric_limits<double>::lowest(); // initialize
        for(std::size_t i = mHeap.size() - 1; i > 0; --i)
        {
            const auto key = mHeap.at(i);
            const double maxval = mLambda(key).max;
            mHeapIntervalMax[i] = Max(mHeapIntervalMax[i], maxval);
            const std::size_t parentIdx = HeapParentIndex(i);
            mHeapIntervalMax[parentIdx] = Max(mHeapIntervalMax[parentIdx], mHeapIntervalMax[i]);
        }
        // do index 0 separately, as we should not look for the parent of the root
        if(!mHeapIntervalMax.empty())
            mHeapIntervalMax[0] = Max(mHeapIntervalMax[0], mLambda(mHeap.at(0)).max);
    }

    void Overlap(const TInterval& interval, std::vector<TKey>& output) const
    {
        mRecursiveCounter = 0;
        // iterate over heap in depth first order
        if(!mHeap.empty())
            OverlapRecursive(interval, 0, output);
    }

private:
    TLambda mLambda;
    std::vector<TKey> mHeap{}; // min heap of intervals
    std::vector<TIntervalType> mHeapIntervalMax{}; // max values in this sub-tree

    mutable int mRecursiveCounter{0};
    void OverlapRecursive(const TInterval& interval, const std::size_t idx, std::vector<TKey>& output) const
    {
        mRecursiveCounter++;
        // std::cout << idx << std::endl;
        // check this index
        const auto nodeInterval = mLambda(mHeap.at(idx));
        if(interval.max < nodeInterval.min)
            return; // all deeper nodes are after this one

        if(interval.min < nodeInterval.max)
            output.push_back(mHeap.at(idx));

        const auto subheapmax = mHeapIntervalMax.at(idx);
        if(subheapmax < interval.min)
            return; // furtherst node in this subtree is not deep enough

        const auto left = HeapLeftChild(idx);
        if(left < mHeap.size())
        {
            OverlapRecursive(interval, left, output);

            const auto right = HeapRightChild(idx);
            if(right < mHeap.size())
                OverlapRecursive(interval, right, output);
        }
    }
};

} // namespace geom
} // namespace vic
