
#pragma once

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

#include "vic/utils/algorithms.h" // sort_individual
#include "vic/utils/concepts.h" // less_than_comparable

namespace vic
{
namespace memory
{

// should be drop-in replacement for (most of) std::map
// stores data in a simple sorted std::vector
// good for small amounts of data, when the main use case is iterating over all data
template <typename TKey, typename TValue>
    requires less_than_comparable<TKey> //  && less_than_comparable<TValue>
class FlatMap
{
public:
    using value_type = std::pair<TKey, TValue>;

private:
    std::vector<value_type> mData{};

public:
    using iterator = decltype(mData.begin());
    using const_iterator = decltype(mData.cbegin());
    using key_type = TKey;
    using mapped_type = TValue;
    using difference_type = std::ptrdiff_t;
    // todo: key_compare
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = std::size_t;

    static constexpr auto SortValueType = [](const value_type& item1, const value_type& item2) { return item1.first < item2.first; };
    static constexpr auto LowerBoundCompare = [](const value_type& item, const key_type& k) -> bool { return item.first < k; };

    // element access
    TValue& at(const TKey& key) { return find(key)->second; }
    const TValue& at(const TKey& key) const { return find(key)->second; }
    TValue& operator[](const TKey& key)
    {
        auto it = find(key);
        if(it != mData.end())
            return it->second;
        mData.push_back({key, mapped_type{}});
        const auto n = mData.size();
        // let sorting algorithm figure out if we need to do anything
        return vic::sort_individual(mData.begin(), //
                                    mData.end(),
                                    mData.end() - 1, // sort last item
                                    SortValueType)
            ->second;
    }

    // Iterators
    auto begin() { return mData.begin(); }
    auto begin() const { return mData.begin(); }
    auto cbegin() const { return mData.cbegin(); }

    auto end() { return mData.end(); }
    auto end() const { return mData.end(); }
    auto cend() const { return mData.cend(); }

    // Capacity
    bool empty() const { return mData.empty(); }
    std::size_t size() const { return mData.size(); }
    std::size_t max_size() const { return std::numeric_limits<difference_type>::max(); }

    // Modifiers
    void clear() { mData.clear(); }

    template <bool sort = true>
    std::pair<iterator, bool> insert(const value_type& pair)
    {
        if constexpr(!sort)
        {
            // just insert, assume key is unique, let user call sort afterwards
            const auto size = mData.size();
            mData.push_back(pair);
            return std::pair(std::prev(mData.end()), true);
        }
        else
        {
            // proper insert, check if key exists and resort afterwards
            const auto& key = pair.first;
            auto it = find(key);
            if(it != mData.end())
                return std::pair(it, false);

            const auto size = mData.size();
            mData.push_back(pair);

            if(size != 0 && mData[size].first < mData[size - 1].first)
            {
                vic::sort_individual(mData.begin(), //
                                     mData.end(),
                                     mData.end() - 1,
                                     SortValueType);
                return std::pair(find(key), true);
            }
            else
            {
                // insert at end is already sorted, or we do not want to sort
                return std::pair(std::prev(mData.end()), true);
            }
        }
    }

    template <bool sort = true>
    std::pair<iterator, bool> insert(value_type&& pair)
    {
        const auto copy = pair;
        return insert<sort>(copy); // tmp, do proper move later
    }

    template <class... Args>
    std::pair<iterator, bool> emplace(Args&&... args)
    {
        // similar to map emplace, we may construct the item and directly destruct it if needed
        mData.emplace_back(args...);
        const auto key = mData.back().first;
        const auto itToFirst = find(key);
        if(itToFirst == std::prev(mData.end()))
        {
            mData.pop_back();
            return std::pair(itToFirst, false);
        }

        auto itNewPos = vic::sort_individual(mData.begin(), mData.end(), std::prev(mData.end()), SortValueType);
        return std::pair(itNewPos, true);
    }

    iterator erase(iterator pos) { return mData.erase(pos); }
    iterator erase(const_iterator pos) { return mData.erase(pos); }
    iterator erase(const TKey& key) { return mData.erase(find(key)); }

    size_type count(const TKey& key) const { return find(key) == mData.end() ? 0 : 1; }
    iterator find(const TKey& key)
    {
        auto it = std::lower_bound(mData.begin(), mData.end(), key, LowerBoundCompare);
        return (it != mData.end() && it->first == key) ? it : mData.end();
    }
    const_iterator find(const TKey& key) const
    {
        auto it = std::lower_bound(mData.begin(), mData.end(), key, LowerBoundCompare);
        return (it != mData.end() && it->first == key) ? it : mData.end();
    }
    bool contains(const TKey& key) const { return find(key) != mData.end(); }

    //
    // Custom, not similar to std::map
    //

    bool Relabel(const TKey oldKey, const TKey newKey)
    {
        if(oldKey == newKey)
            return true;

        if(find(newKey) != mData.end())
            return false; // newKey already exits, not available

        auto it = find(oldKey);
        if(it == mData.end())
            return false; // oldKey does not exist in this map

        it->first = newKey; // change label

        vic::sort_individual(mData.begin(), mData.end(), it, SortValueType);

        return true;
    }

    auto Reserve(const std::size_t size) { mData.reserve(size); }

    // call sort after multiple insert<false>()
    void Sort() { std::sort(mData.begin(), mData.end(), SortValueType); }

private:
};

} // namespace memory
} // namespace vic