#pragma once

#include <ranges>

namespace vic
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
} // namespace vic