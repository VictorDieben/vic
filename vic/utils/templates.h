#pragma once

#include <array>
#include <type_traits>

namespace vic
{
namespace templates
{

// find out if T is contained in Ts
template <typename T, typename... Ts>
constexpr bool Contains()
{
    return (std::is_same_v<T, Ts> || ...);
}

template <typename T1, typename T2, typename... Args>
constexpr bool IsUnique_helper()
{
    if(std::is_same_v<T1, T2>)
        return false;

    if constexpr(sizeof...(Args) > 0)
    {
        constexpr bool b1 = Contains<T1, T2, Args...>();
        constexpr bool b2 = IsUnique_helper<T2, Args...>();
        return (!b1) && b2;
    }

    return true;
}

template <typename... Ts>
constexpr bool IsUnique()
{
    if constexpr(sizeof...(Ts) > 1)
        return IsUnique_helper<Ts...>();
    else
        return true;
}

//
//
//

template <typename T1, typename T2, typename... Ts>
struct to_unique_helper
{
    using tuple_type = std::conditional_t<IsUnique<T1, T2, Ts...>(), //
                                          std::tuple<T1, T2, Ts...>,
                                          double>; // todo

    // std::conditional_t<Contains<T1, T2, Ts...>() typename to_unique_helper<Ts...>::tuple_type>;
};

template <typename... Ts>
struct to_unique
{
    using tuple_type = typename to_unique_helper<Ts...>::tuple_type;
};

//
//
//

template <typename T, typename U, typename... Us>
constexpr auto GetIndex()
{
    if constexpr(std::is_same_v<T, U>)
    {
        return 0;
    }
    else
    {
        if constexpr(sizeof...(Us) > 0)
        {
            return 1 + GetIndex<T, Us...>();
        }
        else
        {
            return -1; // use -1 as "does not exist"
        }
    }
}

} // namespace templates
} // namespace vic