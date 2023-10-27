
#pragma once

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace vic
{
namespace memory
{

// should be drop-in replacement for (most of) std::map
// stores data in a simple sorted std::vector
// good for small amounts of data, when the main use case is iterating over all data
template <typename TKey, typename TCompare = std::less<TKey>>
class FlatSet
{
public:
    using Key = TKey;
    using Compare = TCompare;

    using key_type = Key;
    using value_type = Key;

    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using key_compare = Compare;
    using value_compare = Compare;

    using reference = value_type&;
    using const_reference = const value_type&;
    //using pointer =
    //using const_pointer =

private:
    std::vector<value_type> mData{};

public:
    using iterator = decltype(mData.begin());
    using const_iterator = decltype(mData.cbegin());

    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    //using node_handle =
    //using insert_return_type =

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
    std::pair<iterator, bool> insert(const Key& key) noexcept
    {
        if constexpr(!sort)
        {
            // just insert, assume key is unique, let user call sort afterwards
            mData.push_back(key);
            return std::pair(std::prev(mData.end()), true);
        }
        else
        {
            // proper insert, check if key exists and resort afterwards
            auto it = find(key);
            if(it != mData.end())
                return std::pair(it, false);

            const auto size = mData.size();
            mData.push_back(key);

            if(size != 0 && mData.at(size) < mData.at(size - 1))
            {
                Sort();
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
    std::pair<iterator, bool> insert(value_type&& pair) noexcept
    {
        const auto copy = pair;
        return insert<sort>(copy); // tmp, do proper move later
    }

    iterator erase(iterator pos) { return mData.erase(pos); }
    iterator erase(const_iterator pos) { return mData.erase(pos); }
    iterator erase(const TKey& key) { return mData.erase(find(key)); }

    size_type count(const TKey& key) const noexcept { return find(key) == mData.end() ? 0 : 1; }
    iterator find(const TKey& key) noexcept
    {
        auto it = mData.begin();
        for(; it != mData.end(); ++it)
            if(*it == key)
                return it;
        return it;
    }
    const_iterator find(const TKey& key) const noexcept
    {
        // todo: binary search
        auto it = mData.cbegin();
        for(; it != mData.cend(); ++it)
            if(*it == key)
                return it;
        return it;
    }
    bool contains(const TKey& key) const noexcept { return find(key) != mData.end(); }

    void pop_back() { mData.pop_back(); }

    //
    // Custom, not similar to std::map
    //

    auto Reserve(const std::size_t size) noexcept { mData.reserve(size); }

    // call sort after multiple insert<false>()
    void Sort() noexcept
    {
        std::sort(mData.begin(),
                  mData.end(), //
                  [&](const auto& item1, const auto& item2) { return item1 < item2; });
    }

private:
};

// should be drop-in replacement for (most of) std::unordered_set
template <typename TKey, typename TKeyEqual = std::equal_to<TKey>>
class UnorderedFlatSet
{
public:
    UnorderedFlatSet() = default;

    template <typename TBegin, typename TEnd>
    UnorderedFlatSet(TBegin begin, TEnd end)
    {
        if constexpr(std::contiguous_iterator<TBegin>)
            mData.reserve(std::distance(begin, end));

        for(auto it = begin; it != end; ++it)
            if(std::find(mData.begin(), mData.end(), *it) == mData.end())
                mData.push_back(*it);
    }
    using Key = TKey;
    using KeyEqual = TKeyEqual;

    using key_type = Key;
    using value_type = Key;

    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using key_equal = KeyEqual;

    using reference = value_type&;
    using const_reference = const value_type&;
    //using pointer =
    //using const_pointer =

private:
    std::vector<value_type> mData{};

public:
    using iterator = decltype(mData.begin());
    using const_iterator = decltype(mData.cbegin());

    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    //using node_handle =
    //using insert_return_type =

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
    void clear() noexcept { mData.clear(); }

    std::pair<iterator, bool> insert(const Key& key) noexcept
    {
        for(auto it = mData.begin(); it != mData.end(); ++it)
            if(*it == key)
                return std::pair(it, false);
        mData.push_back(key);
        return std::pair(std::prev(mData.end()), true);
    }

    std::pair<iterator, bool> insert(value_type&& pair) noexcept
    {
        const auto copy = pair;
        return insert(copy); // tmp, do proper move later
    }

    iterator erase(iterator pos) { return mData.erase(pos); }
    iterator erase(const_iterator pos) { return mData.erase(pos); }
    iterator erase(const TKey& key) { return mData.erase(find(key)); }

    iterator find(const TKey& key) noexcept
    {
        auto it = mData.begin();
        for(; it != mData.end(); ++it)
            if(*it == key)
                return it;
        return it;
    }
    const_iterator find(const TKey& key) const noexcept
    {
        auto it = mData.begin();
        for(; it != mData.end(); ++it)
            if(*it == key)
                return it;
        return it;
    }
    bool contains(const TKey& key) const noexcept
    {
        for(auto it = mData.begin(); it != mData.end(); ++it)
            if(*it == key)
                return true;
        return false;
    }

    size_type count(const TKey& key) const noexcept { return contains(key) ? 1 : 0; }

    //
    // Custom, not similar to std::unordered_set
    //

    auto reserve(const std::size_t size) { mData.reserve(size); }

    void pop_back() { mData.pop_back(); }
};

} // namespace memory
} // namespace vic