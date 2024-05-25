#pragma once

#include <iterator>
#include <optional>
#include <ranges>

#include <range/v3/algorithm/set_algorithm.hpp>
#include <range/v3/all.hpp> // get everything
#include <range/v3/view.hpp>
#include <range/v3/view/set_algorithm.hpp>

#include "vic/utils/concepts.h"

namespace vic
{

// 2d overlap

template <typename TMap1, typename TMap2>
    requires ConceptSameKey<TMap1, TMap2>
auto Overlap(TMap1& map1, TMap2& map2)
{
    // test code:
    // https://godbolt.org/#z:OYLghAFBqd5QCxAYwPYBMCmBRdBLAF1QCcAaPECAMzwBtMA7AQwFtMQByARg9KtQYEAysib0QXACx8BBAKoBnTAAUAHpw

    //using KeyType = typename TMap1::key_type;
    //using ValueType1 = typename TMap1::mapped_type;
    //using ValueType2 = typename TMap2::mapped_type;

    //const auto keys1 = std::views::keys(map1);
    //const auto keys2 = std::views::keys(map2); // std::ranges::keys_view

    //namespace rv = ranges::views;
    //const auto intersection = rv::set_intersection(keys1, keys2);

    //auto res1 = intersection | std::views::transform([&map1](const auto& key) { return &map1[key]; });
    //auto res2 = intersection | std::views::transform([&map2](const auto& key) { return &map2[key]; });

    //auto res1 = intersection | std::views::transform([&map1, it1 = map1.begin()](const auto& key) {
    //                return &map1[key]; //
    //            });

    //auto res2 = intersection | std::views::transform([&map2, it2 = map2.begin()](const auto& key) {
    //                return &map2[key]; //
    //            });

    //// what we want:
    //auto zipView = std::views::zip(intersection, res1, res2);
    //return std::ranges::to<std::vector>(zipView);

    // what we want:
    // return std::views::zip_transform(f, intersection, res1, res2);

    // not ideal:
    //auto keys = std::ranges::to<std::vector>(intersection);
    //auto values1 = std::ranges::to<std::vector>(res1);
    //auto values2 = std::ranges::to<std::vector>(res2);
    //return std::views::zip_transform(f, keys, values1, values2);

    const auto keys1 = std::views::keys(map1);
    const auto keys2 = std::views::keys(map2);
    const auto intersection = ranges::views::set_intersection(keys1, keys2);

    const auto zipView = std::views::zip(intersection, //
                                         intersection | std::views::transform([&map1, it1 = map1.begin()](const auto& key) mutable {
                                             return &map1[key]; //
                                         }),
                                         intersection | std::views::transform([&map2, it2 = map2.begin()](const auto& key) mutable {
                                             return &map2[key]; //
                                         }));

    return std::ranges::to<std::vector>(zipView);
}

} // namespace vic