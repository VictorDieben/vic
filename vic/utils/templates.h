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

template <typename T, typename... Ts>
concept ConceptContains = Contains<T, Ts...>();

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
constexpr int GetIndex()
{
    if constexpr(std::is_same_v<T, U>)
    {
        return 0;
    }
    else
    {
        if constexpr(sizeof...(Us) > 0)
        {
            constexpr auto idx = GetIndex<T, Us...>();
            return idx == -1 ? -1 : 1 + idx;
        }
        else
        {
            return -1; // use -1 as "does not exist"
        }
    }
}

//
//
//

namespace detail
{
// To allow ADL with custom begin/end
using std::begin;
using std::end;

template <typename T>
auto is_iterable_impl(int) -> decltype(begin(std::declval<T&>()) != end(std::declval<T&>()), // begin/end and operator !=
                                       void(), // Handle evil operator ,
                                       ++std::declval<decltype(begin(std::declval<T&>()))&>(), // operator ++
                                       void(*begin(std::declval<T&>())), // operator*
                                       std::true_type{});

template <typename T>
std::false_type is_iterable_impl(...);

} // namespace detail

template <typename T>
using is_iterable = decltype(detail::is_iterable_impl<T>(0));

template <typename T>
concept is_enum = std::is_enum_v<T>;

} // namespace templates
} // namespace vic