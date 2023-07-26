#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace vic
{
namespace memory
{

// a map that stores the values in a densely packed vector.
// it also contains two vectors which contain the mapping from id to index, and index to id/
// Iterating over the map should always be in order of increasing key
template <typename TKey, typename TValue, typename TIndex = std::size_t>
class DenseMap
{
public:
    using KeyType = TKey;
    using ValueType = TValue;
    using IndexType = TIndex;

    DenseMap() = default;

    void Insert(KeyType key, ValueType value)
    {
        mValues.push_back({key, value});
        mKeyToIndex.push_back({key, mValues.size() - 1});
        // todo: insertion sort? array is already mostly sorted, only 1 item is out of place
        std::sort(mKeyToIndex.begin(),
                  mKeyToIndex.end(), //
                  [](const auto& left, const auto& right) { return left.first < right.first; });
    }

    ValueType& At(KeyType key)
    {
        return mValues[0].second; // todo
    }

    // todo: [] operator?

    void Remove(KeyType removeKey)
    {
        const auto size = mValues.size(); //
        assert(size > 0);

        // auto removeIt = std::lower_bound(mKeyToIndex.begin(), mKeyToIndex.end(), removeKey);

        const auto lowerBoundLambda = [](auto const& item, double d) -> bool { return item.second < d; };
        auto removeIt = std::lower_bound(mKeyToIndex.begin(), mKeyToIndex.end(), removeKey, lowerBoundLambda);

        assert(removeIt != mKeyToIndex.end());

        const auto removeIdx = removeIt->second;

        if(removeIdx == size - 1)
        {
            // item we want to remove is last in vector, no need to shift values
            mValues.pop_back();
            mKeyToIndex.erase(removeIt);
        }
        else
        {
            // move the last value into the newly empty index
            const auto keyLast = mValues.back().first;
            mValues.at(removeIdx) = mValues.back();
            mValues.pop_back();

            // update the mKeyToIndex:
            // - update item that points to the last object
            // - remove old key
            //auto moveIt = std::lower_bound(mKeyToIndex.begin(), mKeyToIndex.end(), keyLast);
            auto moveIt = std::lower_bound(mKeyToIndex.begin(), mKeyToIndex.end(), keyLast, lowerBoundLambda);
            moveIt->second = removeIdx;
            mKeyToIndex.erase(removeIt);
        }
    }

    IndexType GetIndex(KeyType key) const
    {
        const auto it = std::lower_bound(mKeyToIndex.begin(), mKeyToIndex.end(), key);
        assert(it != mKeyToIndex.end());
        return it->second;
    }

    void Write() const
    {
        std::cout << "values:\n";
        for(std::size_t i = 0; i < mValues.size(); ++i)
            std::cout << " - " << std::to_string(i) << "\t: " << mValues.at(i).first << ": " << mValues.at(i).second << "\n";
        std::cout << "key to index:\n";
        for(const auto [key, index] : mKeyToIndex)
            std::cout << " - " << std::to_string(key) << " -> " << std::to_string(index) << "\n";
    }

private:
    std::vector<std::pair<KeyType, TValue>> mValues{};
    std::vector<std::pair<KeyType, IndexType>> mKeyToIndex{};
};

} // namespace memory
} // namespace vic