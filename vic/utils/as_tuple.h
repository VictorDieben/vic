#pragma once

#include <concepts>
#include <tuple>

#include "vic/utils/concepts.h"

namespace vic
{

// todo: remove once "structured binding can introduce a pack" is accepted
// the giant if constexpr mess down below is due to a language limitation

struct any_type
{
    template <class T>
    operator T()
    { }
};

// Count the number of members in an aggregate by (ab-)using
// aggregate initialization
template <typename T>
    requires ConceptAggregate<T>
consteval std::size_t count_members(auto... members)
{
    if constexpr(!requires { T{members...}; })
        return sizeof...(members) - 1;
    else
        return count_members<T>(members..., any_type{});
}

template <typename T>
    requires ConceptAggregate<T>
constexpr auto as_tuple(T& data)
{
    constexpr std::size_t fieldCount = count_members<T>();

    if constexpr(fieldCount == 0)
        return std::tie();
    else if constexpr(fieldCount == 1)
    {
        auto& [m1] = data;
        return std::tie(m1);
    }
    else if constexpr(fieldCount == 2)
    {
        auto& [m1, m2] = data;
        return std::tie(m1, m2);
    }
    else if constexpr(fieldCount == 3)
    {
        auto& [m1, m2, m3] = data;
        return std::tie(m1, m2, m3);
    }
    else if constexpr(fieldCount == 4)
    {
        auto& [m1, m2, m3, m4] = data;
        return std::tie(m1, m2, m3, m4);
    }
    else if constexpr(fieldCount == 5)
    {
        auto& [m1, m2, m3, m4, m5] = data;
        return std::tie(m1, m2, m3, m4, m5);
    }
    else if constexpr(fieldCount == 6)
    {
        auto& [m1, m2, m3, m4, m5, m6] = data;
        return std::tie(m1, m2, m3, m4, m5, m6);
    }
    else if constexpr(fieldCount == 7)
    {
        auto& [m1, m2, m3, m4, m5, m6, m7] = data;
        return std::tie(m1, m2, m3, m4, m5, m6, m7);
    }
    else if constexpr(fieldCount == 8)
    {
        auto& [m1, m2, m3, m4, m5, m6, m7, m8] = data;
        return std::tie(m1, m2, m3, m4, m5, m6, m7, m8);
    }
    else if constexpr(fieldCount == 9)
    {
        auto& [m1, m2, m3, m4, m5, m6, m7, m8, m9] = data;
        return std::tie(m1, m2, m3, m4, m5, m6, m7, m8, m9);
    }
    else if constexpr(fieldCount == 10)
    {
        auto& [m1, m2, m3, m4, m5, m6, m7, m8, m9, m10] = data;
        return std::tie(m1, m2, m3, m4, m5, m6, m7, m8, m9, m10);
    }
    else
    {
        static_assert(fieldCount != fieldCount, "Too many fields for as_tuple(...)! add more if statements!");
    }
}

} // namespace vic