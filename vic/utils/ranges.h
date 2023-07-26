#pragma once

#include <concepts>
#include <ranges>
#include <vector>

namespace vic
{
namespace ranges
{

// todo: replace with c++23 drop last, once available
struct drop_last_t
{
    template <std::ranges::sized_range R>
    requires std::ranges::viewable_range<R>
    friend auto operator|(R&& r, drop_last_t)
    {
        return r | std::ranges::views::reverse | std::ranges::views::drop(1) | std::ranges::views::reverse; //
    }
};
inline constexpr drop_last_t drop_last;

namespace view
{
template <typename T>
concept MapConcept = requires(T map)
{
    typename T::key_type;
    typename T::mapped_type;
    typename T::iterator;
    typename T::const_iterator;
    map.begin();
    map.end();
    map.at(std::declval<typename T::key_type>());
    map[std::declval<typename T::key_type>()];
};

template <typename T>
using iterator_t = decltype(std::begin(std::declval<T&>()));

template <typename T>
using range_reference_t = decltype(*std::begin(std::declval<T&>()));

struct map_intersection_fn : public std::ranges::view_base
{
private:
    struct iterator_type // : iterator_t<urng_t>
    { };

public:
    using reference = uint64_t;
    using const_reference = uint64_t;
    using value_type = uint64_t;

    using iterator = iterator_type;
    using const_iterator = iterator_type;

    map_intersection_fn() = default;

    template <MapConcept R1, MapConcept R2>
    requires std::same_as<typename R1::key_type, typename R2::key_type>
    auto operator()(R1&& r1, R2&& r2) const
    {
        using KeyType = R1::key_type;
        using Type1 = R1::mapped_type;
        using Type2 = R2::mapped_type;

        // todo: ValueType should have iterators to the input maps
        using ValueType = std::tuple<KeyType, Type1*, Type2*>;

        return std::vector<ValueType>{};
    }
};

inline constexpr map_intersection_fn map_intersection;
} // namespace view

} // namespace ranges
} // namespace vic