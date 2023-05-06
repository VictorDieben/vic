#pragma once

#include <array>

namespace vic
{
namespace templates
{

namespace detail
{
template <typename T1, typename T2, typename... Args>
constexpr bool contains_helper()
{
    if(std::is_same_v<T1, T2>)
        return true;
    if constexpr(sizeof...(Args) > 0)
        return contains_helper<T1, Args...>();
    return false;
}
} // namespace detail

// find out if T is contained in Ts
template <typename T, typename... Ts>
constexpr bool contains()
{
    return detail::contains_helper<T, Ts...>();
}

} // namespace templates

} // namespace vic