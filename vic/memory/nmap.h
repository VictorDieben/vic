#pragma once

#include "vic/utils/templates.h"

#include <range/v3/all.hpp> // get everything
#include <ranges>

namespace vic
{
// attempt at simplifying the ecs folder. do not use yet.

//

template <typename TKey, typename TValue>
class SubMap
{
public:
    SubMap() = default;

    TValue& Add(const TKey key, auto&&... args)
    {
        auto res = mData.try_emplace(key, std::forward<decltype(args)>(args)...);
        return res.first->second;
    }

    TValue& Get(const TKey key)
    {
        auto it = mData.find(key);
        if(it == std::end(mData))
            throw std::runtime_error(std::string("key ") + std::to_string(key) + " has no " + typeid(TValue).name());
        return it->second;
    }

    const TValue& Get(const TKey key) const
    {
        auto it = mData.find(key);
        if(it == std::end(mData))
            throw std::runtime_error(std::string("key ") + std::to_string(key) + " has no " + typeid(TValue).name());
        return it->second;
    }

    TValue* TryGet(const TKey key)
    {
        auto it = mData.find(key);
        return (it == std::end(mData)) ? nullptr : &it->second;
    }

    const TValue* TryGet(const TKey key) const
    {
        auto it = mData.find(key);
        return (it == std::end(mData)) ? nullptr : &it->second;
    }

    std::size_t Size() const noexcept { return mData.size(); }
    bool Empty() const noexcept { return mData.empty(); }
    bool Has(const TKey key) const noexcept { return mData.find(key) != std::end(mData); }

    auto Keys() const noexcept
    {
        // todo: figure out why std::views::keys does not work
        // return  std::views::keys(mData) ;
        return mData | ranges::views::transform([](auto&& item) { return item.first; });
    }

    auto Values(const std::ranges::input_range auto& range)
    {
        return range | ranges::views::transform([this](const auto item) {
                   return Get(item); //
               });
    }

    auto Iterate(const std::ranges::input_range auto& range)
    {
        return range | ranges::views::transform([this](const auto item) {
                   return TryGet(item); //
               });
    }

    // private:
    std::map<TKey, TValue> mData{};
};

template <typename TKey, typename... TValues>
    requires templates::ConceptUnique<TValues...>
class NMap : public SubMap<TKey, TValues>...
{
public:
    NMap() = default;

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    T& Add(const TKey key, auto&&... args)
    {
        return SubMap<TKey, T>::Add(key, std::forward<decltype(args)>(args)...);
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    T& Get(const TKey key)
    {
        return SubMap<TKey, T>::Get(key);
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    const T& Get(const TKey key) const
    {
        return SubMap<TKey, T>::Get(key);
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    T* TryGet(const TKey key)
    {
        return SubMap<TKey, T>::TryGet(key);
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    const T* TryGet(const TKey key) const
    {
        return SubMap<TKey, T>::TryGet(key);
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    auto Keys() const noexcept
    {
        return SubMap<TKey, T>::Keys();
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    auto Values(const std::ranges::input_range auto& range)
    {
        return SubMap<TKey, T>::Values(range);
    }

    template <typename T>
        requires templates::ConceptContains<T, TValues...>
    std::size_t Size() const noexcept
    {
        return SubMap<TKey, T>::Size();
    }

    template <typename T1, typename T2, typename... Ts>
        requires templates::ConceptContains<T1, TValues...> && //
                 templates::ConceptContains<T2, TValues...> && //
                 templates::ConceptUnique<T1, T2, Ts...>
    auto Intersection() const noexcept
    {
        const auto intersection = ranges::views::set_intersection(Keys<T1>(), Keys<T2>());

        if constexpr(sizeof...(Ts) == 0)
            return intersection;

        else if constexpr(sizeof...(Ts) == 1)
            return ranges::views::set_intersection(intersection, Keys<templates::FirstType<Ts...>>());

        else
            return ranges::views::set_intersection(intersection, Intersection<Ts...>());
    }

    template <typename T1, typename T2, typename... Ts>
        requires templates::ConceptContains<T1, TValues...> && //
                 templates::ConceptContains<T2, TValues...> && //
                 templates::ConceptUnique<T1, T2, Ts...>
    auto Union() const noexcept
    {
        const auto setUnion = ranges::views::set_union(Keys<T1>(), Keys<T2>());

        if constexpr(sizeof...(Ts) == 0)
            return setUnion;

        else if constexpr(sizeof...(Ts) == 1)
            return ranges::views::set_union(setUnion, Keys<templates::FirstType<Ts...>>());

        else
            return ranges::views::set_union(setUnion, Union<Ts...>());
    }

    template <typename T1, typename T2>
        requires templates::ConceptContains<T1, TValues...> && //
                 templates::ConceptContains<T2, TValues...>
    auto Filter()
    {
        const auto ids = Intersection<T1, T2>();
        return ranges::views::zip(ids, //
                                  SubMap<TKey, T1>::Values(ids),
                                  SubMap<TKey, T2>::Values(ids));
    }

    template <typename T1, typename T2>
        requires templates::ConceptContains<T1, TValues...> && //
                 templates::ConceptContains<T2, TValues...>
    auto Iterate(const std::ranges::input_range auto& range)
    {
        return ranges::views::zip(range, //
                                  SubMap<TKey, T1>::Iterate(range),
                                  SubMap<TKey, T2>::Iterate(range));
    }

    //template <typename... Ts>
    //    requires templates::ConceptUnique<Ts...> // &&
    //// templates::ConceptContains<T1, TValues...>&& //
    //// templates::ConceptContains<T2, TValues...> auto
    //auto Iterate(std::ranges::input_range auto&& range)
    //{
    //    const auto dummy = []<typename T>(auto&& range, T&& dummy) { return SubMap<TKey, T>::Iterate(range); };
    //    return ranges::views::zip(range, //
    //                              (dummy(range, declval<Ts...>()), ...);
    //}

private:
};

} // namespace vic