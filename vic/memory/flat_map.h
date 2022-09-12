
#pragma once

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
    using iterator = decltype(mData.begin());
    using const_iterator = decltype(mData.cbegin());
    using key_type = TKey;
    using mapped_type = TValue;
    using value_type = std::pair<TKey, TValue>;
    using difference_type = std::size_t;
    // todo: key_compare
    using reference = value_type&;
    using const_reference = const value_type&;


    // element access
    TValue& at(const TKey& key) { return {}; }
    const TValue& at(const TKey& key) const { return {}; }
    TValue& operator[](const TKey& key) { return {}; }

    // Iterators
    auto begin() { return mData.begin(); }
    auto cbegin() const { return mData.cbegin(); }

    auto end() { return mData.end(); }
    auto cend() const { return mData.cend(); }

    // Capacity
    bool empty() const { return mData.empty(); }
    std::size_t size() const { return mData.size(); }
    std::size_t max_size() const { return std::numeric_limits<difference_type>::max(); }

    // Modifiers
    void clear() { mData.clear(); }
    std::pair<Iterator, bool> insert(const value_type& pair) { return {}; }
    std::pair<Iterator, bool> insert(value_type&& pair) { return {}; }

    iterator erase(iterator pos){ return {}; }
    iterator erase(const_iterator pos){ return {}; }
    
    iterator erase(const TKey& key){ return {}; }

    size_type count(const TKey& key) const { return {}; }
    iterator find(const TKey& key) { return {}; }
    const_iterator find(const TKey& key) {return {}; }
    bool contains(const TKey& key) { return {}; }


private:
    std::vector<value_type> mData;
};

} // namespace memory
} // namespace vic