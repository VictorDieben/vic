
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
template <typename TKey, typename TValue>
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

    // element access
    TValue& at(const TKey& key) { return *find(key); }
    const TValue& at(const TKey& key) const { return *find(key); }
    TValue& operator[](const TKey& key)
    {
        auto it = find(key);
        if(it != mData.end())
            return it->second;
        mData.push_back({key, mapped_type{}});
        Sort();
        return find(key)->second;
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
    std::pair<iterator, bool> insert(const value_type& pair) { throw std::runtime_error("not implemented"); }
    std::pair<iterator, bool> insert(value_type&& pair) { throw std::runtime_error("not implemented"); }

    iterator erase(iterator pos) { return mData.erase(pos); }
    iterator erase(const_iterator pos) { return mData.erase(pos); }
    iterator erase(const TKey& key) { return mData.erase(find(key)); }

    size_type count(const TKey& key) const { return find(key) == mData.end() ? 0 : 1; }
    iterator find(const TKey& key)
    {
        for(std::size_t i = 0; i < mData.size(); ++i)
            if(mData.at(i).first == key)
                return mData.begin() + i;
        return mData.end();

        // todo: binary search? probably not worth the time

        //const auto lambda = [](auto const& item, const key_type k) -> bool { return item.first < k; };
        //auto it = std::lower_bound(mData.begin(), mData.end(), key, lambda);
        //return it;
    }
    const_iterator find(const TKey& key) const
    {
        for(std::size_t i = 0; i < mData.size(); ++i)
            if(mData.at(i).first == key)
                return mData.begin() + i;
        return mData.end();
    }
    bool contains(const TKey& key) const { return find(key) != mData.end(); }

private:
    void Sort()
    {
        std::sort(mData.begin(),
                  mData.end(), //
                  [&](const auto& item1, const auto& item2) { return item1.first < item2.first; });
    }
};

} // namespace memory
} // namespace vic