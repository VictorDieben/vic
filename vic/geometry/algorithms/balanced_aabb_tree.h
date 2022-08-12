#pragma once

#include "vic/geometry/geometry.h"
#include "vic/utils.h"
#include <bit>
#include <concepts>
#include <type_traits>

namespace vic
{
namespace geom
{

constexpr std::size_t PyramidSize(const std::size_t level)
{
    return Power<std::size_t, std::size_t, std::size_t>(2, level) - 1; //
}

constexpr std::size_t PyramidFirstIndex(const std::size_t level)
{
    return Power<std::size_t, std::size_t, int>(2, std::max(0, static_cast<int>(level - 1))) - 1; //
}

constexpr std::size_t PyramidIndex(const std::size_t level, const std::size_t index)
{
    return PyramidFirstIndex(level) + index; //
}

constexpr std::size_t PyramidLevelSize(const std::size_t level)
{
    return PyramidSize(level) - PyramidSize(std::max(0, static_cast<int>(level - 1))); //
}

constexpr std::size_t ParentIndex(const std::size_t childIndex)
{
    return {}; //
}

constexpr std::size_t LeftChildIndex(const std::size_t parentIndex)
{
    return {}; //
}

constexpr std::size_t RightChildIndex(const std::size_t parentIndex)
{
    return {}; //
}

constexpr bool IsLeftChild(const std::size_t index)
{
    return {}; //
}

// Example of a pyramid vector:
// 0:
// 1:   0
// 2:   1 2
// 3:   3 4 5 6
// 4:   7 8 9 10 11 12 13 14
// 5:   15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30

template <typename T>
class PyramidVector
{
public:
    static_assert(std::is_default_constructible_v<T>);
    PyramidVector() = default;

    void SetLevel(const std::size_t level)
    {
        mLevel = level;
        mVector.resize(PyramidSize(level));
    }

    std::size_t GetLevel() const { return mLevel; }
    const std::vector<T> GetVector() const { return mVector; }
    std::size_t GetSize() const { return mVector.size(); }

    auto LevelIterator(const std::size_t level)
    {
        const auto begin = mVector.begin() + PyramidFirstIndex(level);
        const auto end = begin + PyramidLevelSize(level);
        return IteratorRange(begin, end);
    }

    // wrapped vector functions
    T& at(const std::size_t level, const std::size_t levelIndex) { return mVector.at(PyramidIndex(level, levelIndex)); }
    const T& at(const std::size_t level, const std::size_t levelIndex) const { return mVector.at(PyramidIndex(level, levelIndex)); }

    T& at(const std::size_t index) { return mVector.at(index); }
    const T& at(const std::size_t index) const { return mVector.at(index); }

private:
    std::size_t mLevel{0};
    std::vector<T> mVector;
};

// Example of balancedTreeVector:
// 0, 4, 1, 5, 2, 6, 3, 7

// a vector that first fills in the even indices, and then fills the odds.

//template <typename T>
//class BTreeVector
//{
//public:
//    static_assert(std::is_default_constructible_v<T>);
//    BTreeVector() { mVector.resize(GetCapacity()); }
//    BTreeVector(const std::vector<T>& vec)
//        : mVector(vec)
//        , mLevel(NextPowerOf2(vec.size()))
//    {
//        mVector.resize(GetCapacity());
//    }
//
//    void push_back(const T& data)
//    {
//        const auto capacity = GetCapacity();
//        if(capacity <= mFilledItems)
//        {
//            mLevel++;
//            mVector.resize(GetCapacity(mLevel));
//        }
//        mVector.at(mFilledItems) = data;
//        mFilledItems++;
//    }
//
//    void pop_back()
//    {
//        assert(mFilledItems != 0);
//        const auto subCapacity = GetCapacity(mLevel - 1);
//        if(mFilledItems == subCapacity)
//        {
//            mLevel--;
//            mVector.resize(subCapacity);
//        }
//        mFilledItems--;
//    }
//
//    std::size_t GetLevel() const { return mLevel; }
//
//    std::size_t GetCapacity() const { return GetCapacity(mLevel); }
//    std::size_t GetCapacity(const std::size_t level) const
//    {
//        return Power<std::size_t, std::size_t, std::size_t>(2, level); //
//    }
//    std::size_t GetSize() const { return mFilledItems; }
//
//private:
//    std::size_t mFilledItems{0};
//    std::size_t mLevel{0};
//    std::vector<T> mVector;
//};

template <typename T, std::size_t dims>
using BBox = AABB<T, dims>;

template <typename TBBox, typename TObject>
struct TreeLeaf
{
    TBBox mBBox{};
    TObject mObject{};
};

template <typename TBBox>
struct TreeBranch
{
    TBBox mBBox{};
};

// combine all bboxes in an iterator range
template <typename TFloat, std::size_t dims, typename TIter>
AABB<TFloat, dims> Combine(TIter begin, TIter end)
{
    AABB<TFloat, dims> result{};
    for(auto it = begin; it < end; ++it)
        result = Combine(result, *it);
    return result;
}

// combine all items in a vector
template <typename TFloat, std::size_t dims>
AABB<TFloat, dims> Combine(const std::vector<AABB<TFloat, dims>>& vec)
{
    return Combine(vec.begin(), vec.end());
}

//template <typename T, typename TFunctor>
//std::vector<T> SortNestedPairsOfTwo(const std::vector<T>& vec, TFunctor functor)
//{
//    if(vec.size() < 3)
//        return vec; // function is useless otherwise
//
//    std::vector<std::vector<std::size_t>> intermediate;
//
//    std::vector<TId> result;
//    result.reserve(vec.size());
//    // todo
//    return result;
//}

template <typename TKey, typename TFloat, std::size_t dims>
class BalancedAABBTree
{
public:
    using BBox = AABB<TFloat, dims>;
    using LeafType = TreeLeaf<BBox, TKey>;
    using BranchType = TreeBranch<BBox>;

    // BBox needs to be default constructible, because we will allocate the entire vector in advance
    // we should also check that the intervals are fully negative ( [max; min] )
    static_assert(std::is_default_constructible_v<BBox>);

    BalancedAABBTree()
    {
        //
        mLeafs.resize(1);
    }

    void Insert(const LeafType& leaf)
    {
        // add item to list of leafs
        const auto size = mLeafs.size();
        if(mNrItems == size)
            mLeafs.resize(size * 2);
        mLeafs.at(mNrItems) = leaf;
        mNrItems++;

        // todo: always rebalance tree?
    }

    void Update()
    {
        UpdateLeafOrder();
        UpdatePyramid();
    }

    void UpdatePyramid()
    {
        // todo: update upper layer of pyramid

        // iterate backwards over all levels. This way, we can still iterate forward over the branches,
        // and the loop itself will not change the range that it is reading.
        // todo: we might want to store each level separately,
        // so we can take const and non-const references to levels separately
        for(int iLevel = int(mBranches.GetLevel()) - 1; iLevel > 0; iLevel--)
        {
            for(std::size_t iBranch = 0; iBranch < PyramidLevelSize(std::size_t(iLevel)); iBranch++)
            {
                const auto branchIdx = PyramidIndex(iLevel, iBranch);
                const auto leftIdx = LeftChildIndex(branchIdx);
                const auto rightIdx = RightChildIndex(branchIdx);
                mBranches.at(branchIdx).mBBox = Combine(mBranches.at(leftIdx).mBBox, //
                                                        mBranches.at(rightIdx).mBBox);
            }
        }
    }

    //
    void Swap(const std::size_t level, const std::size_t first, const std::size_t second)
    {
        //
    }

private:
    void UpdateLeafOrder()
    {
        // todo: when we update leaf order, we will also need to update the pyramid.
        // the reverse is not always true

        // todo: assert mLeafs.size() is power of 2?

        // initialize
        std::vector<std::vector<std::size_t>> indices;
        std::vector<BBox> partialBoxes;
        for(const auto idx : Range<std::size_t>(0, mLeafs.size()))
        {
            indices.push_back({idx});
            partialBoxes.push_back(mLeafs.at(idx).mBBox);
        }

        const auto volumeLambda = [](const BBox& left, const BBox& right) {
            return Volume(Combine(left, right)); //
        };

        while(indices.size() > 2)
        {
            const auto combinations = GroupPairsOfTwo(partialBoxes, volumeLambda);

            std::vector<std::vector<std::size_t>> newIndices;
            std::vector<BBox> newPartialBoxes;

            // combine the sub arrays
            for(std::size_t i = 0; i < combinations.size(); ++i)
            {
                const auto& combination = combinations.at(i);

                const auto& firstVec = indices.at(combination.first);
                const auto& secondVec = indices.at(combination.second);
                std::vector<std::size_t> subIndices = firstVec;
                subIndices.insert(subIndices.end(), secondVec.begin(), secondVec.end());
                newIndices.push_back(subIndices);
                newPartialBoxes.push_back(Combine(partialBoxes.at(combination.first), //
                                                  partialBoxes.at(combination.second)));
            }

            //
            indices = newIndices;
            partialBoxes = newPartialBoxes;
        }

        std::vector<LeafType> newLeafs;
        newLeafs.resize(mLeafs.size());
        std::size_t idx = 0;
        for(const auto& subIndices : indices)
        {
            for(const auto& item : subIndices)
            {
                newLeafs.at(idx) = mLeafs.at(item);
                idx++;
            }
        }
    }

    // store leafs and branches separately, to avoid runtime checks
    std::size_t mNrItems{0};
    std::vector<LeafType> mLeafs{};
    PyramidVector<BranchType> mBranches{};
};

} // namespace geom
} // namespace vic