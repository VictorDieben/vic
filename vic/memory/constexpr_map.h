#pragma once

#include <algorithm>
#include <cstddef>

// this file defines a map, similar to std::map, that can be used at compile time.

template <typename TKey, typename TValue, std::size_t size>
class ConstexprMap
{
public:
    using ArrayType = std::array<std::pair<TKey, TValue>, size>;

    constexpr ConstexprMap(const ArrayType& map)
        : mMap(map)
    {
        // TODO: check duplicates?
    }

    constexpr TValue at(TKey key) const
    {
        const auto it = std::find_if(mMap.begin(),
                                     mMap.end(), //
                                     [&](const auto& item) { return item.first == key; });
        if constexpr(it == mMap.end())
            throw std::range_error("key not found");
        else
            return it->second;
    }

    // note: no [] operator, in std::map it might add the key

private:
    ArrayType mMap;
};