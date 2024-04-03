#pragma once

#include "vic/entity_system/definitions.h"
#include <algorithm>
#include <cassert>
#include <ranges>
#include <tuple>
#include <vector>
#include <stdexcept>
	

namespace vic
{
namespace ecs
{

namespace algorithms
{
// 3d
template <typename T1, typename T2, typename T3, typename TSystem, typename TFunctor>
void FilterForeach(TSystem& system, TFunctor functor)
{
    using Type1 = std::remove_cv_t<T1>;
    using Type2 = std::remove_cv_t<T2>;
    using Type3 = std::remove_cv_t<T3>;

    auto it1 = system.begin<Type1>();
    auto it2 = system.begin<Type2>();
    auto it3 = system.begin<Type3>();

    const auto it1End = system.end<Type1>();
    const auto it2End = system.end<Type2>();
    const auto it3End = system.end<Type3>();

    while(it1 != it1End && it2 != it2End && it3 != it3End)
    {
        if(it1->first < it2->first)
            it1 = std::next(it1);
        else if(it2->first < it1->first)
            it2 = std::next(it2);
        else // it1 == it2
        {
            if(it2->first < it3->first)
                it2 = std::next(it2);
            else if(it3->first < it2->first)
                it3 = std::next(it3);
            else // it2 == it3
            {
                functor(it1->first, it1->second, it2->second, it3->second);
                it1 = std::next(it1);
            }
        }
    }
}

template <typename T1, typename T2, typename TSystem, typename TFunctor>
void FilterForeach(TSystem& system, TFunctor functor)
{
    using Type1 = std::remove_cv_t<T1>;
    using Type2 = std::remove_cv_t<T2>;

    auto it1 = system.begin<Type1>();
    auto it2 = system.begin<Type2>();
    const auto it1End = system.end<Type1>();
    const auto it2End = system.end<Type2>();

    while(it1 != it1End && it2 != it2End)
    {
        if(it1->first < it2->first)
            it1 = std::next(it1);
        else if(it2->first < it1->first)
            it2 = std::next(it2);
        else
        {
            functor(it1->first, it1->second, it2->second);
            it1 = std::next(it1);
            it2 = std::next(it2);
        }
    }
}

template <typename T, typename TSystem, typename TFunctor>
void FilterForeach(TSystem& system, TFunctor functor)
{
    using Type1 = std::remove_cv_t<T>;

    auto itBegin = system.begin<Type1>();
    const auto itEnd = system.end<Type1>();
    for(auto it = itBegin; it != itEnd; ++it)
    {
        functor(it->first, it->second);
    }
}

// 3d
template <typename T1, typename T2, typename T3, typename TSystem, typename TResult>
auto Filter(TSystem& system, std::vector<TResult>& result)
{
    result.clear();
    const auto functor = [&](EntityId id, T1& first, T2& second, T3& third) {
        result.push_back(TResult{id, &first, &second, &third}); //
    };
    FilterForeach<T1, T2, T3>(system, functor);
    return result;
}

template <typename T1, typename T2, typename TSystem, typename TResult>
auto Filter(TSystem& system, std::vector<TResult>& result)
{
    result.clear();
    const auto functor = [&](EntityId id, T1& first, T2& second) {
        result.push_back(TResult{id, &first, &second}); //
    };
    FilterForeach<T1, T2>(system, functor);
    return result;
}

template <typename T, typename TSystem, typename TResult>
auto Filter(TSystem& system, std::vector<TResult>& result)
{
    result.clear();
    const auto functor = [&](EntityId id, T& first) {
        result.push_back(TResult{id, &first}); //
    };
    FilterForeach<T>(system, functor);
    return result;
}

// 3d
template <typename T1, typename T2, typename T3, typename TSystem>
auto Filter(TSystem& system)
{
    using ResultType = std::tuple<EntityId, T1*, T2*, T3*>;
    std::vector<ResultType> result{};
    return Filter<T1, T2, T3>(system, result);
}

// 2d
template <typename T1, typename T2, typename TSystem>
auto Filter(TSystem& system)
{
    using ResultType = std::tuple<EntityId, T1*, T2*>;
    std::vector<ResultType> result{};
    return Filter<T1, T2>(system, result);
}

// 1d
template <typename T, typename TSystem>
auto Filter(TSystem& system)
{
    using ResultType = std::tuple<EntityId, T*>;
    std::vector<ResultType> result{};
    return Filter<T>(system, result);
}

// ranges

//template <typename T1, typename T2, typename TSystem>
//auto Filter(TSystem& system)
//{
//    //
//}

template <typename TComponent, typename TEcs, typename TIter>
auto Iterate(TEcs& ecs, const TIter begin, const TIter end)
{
#ifdef _DEBUG
    if(!std::is_sorted(begin, end))
        throw std::runtime_error("Iterate(): range is not sorted!");
#endif

    using ResultType = std::pair<EntityId, TComponent*>;
    std::vector<ResultType> result;
    if constexpr(std::contiguous_iterator<TIter>)
        result.reserve(std::distance(begin, end));

    using iterType = decltype(ecs.begin<TComponent>());

    iterType it = ecs.begin<TComponent>();
    TIter entityIt = begin;

    while(it != ecs.end<TComponent>() && entityIt != end)
    {
        if(it->first < *entityIt)
            it = std::next(it);
        else if(*entityIt < it->first)
            entityIt = std::next(entityIt);
        else
        {
            result.push_back({it->first, &(it->second)});
            it = std::next(it);
            entityIt = std::next(entityIt);
        }
    }
    return result;
}

template <typename TComponent, typename TEcs, typename TIter>
auto Iterate(const TEcs& ecs, const TIter begin, const TIter end)
{
#ifdef _DEBUG
    if(!std::is_sorted(begin, end))
        throw std::runtime_error("Iterate(): range is not sorted!");
#endif

    using ResultType = std::pair<EntityId, const TComponent*>;
    std::vector<ResultType> result;
    if constexpr(std::contiguous_iterator<TIter>)
        result.reserve(std::distance(begin, end));

    using iterType = decltype(ecs.begin<TComponent>());

    iterType it = ecs.begin<TComponent>();
    TIter entityIt = begin;

    while(it != ecs.end<TComponent>() && entityIt != end)
    {
        if(it->first < *entityIt)
            it = std::next(it);
        else if(*entityIt < it->first)
            entityIt = std::next(entityIt);
        else
        {
            result.push_back({it->first, &(it->second)});
            it = std::next(it);
            entityIt = std::next(entityIt);
        }
    }
    return result;
}

template <typename T1, typename T2, typename TEcs, typename TIter>
auto Iterate2d(TEcs& ecs, const TIter begin, const TIter end)
{
#ifdef _DEBUG
    if(!std::is_sorted(begin, end))
        throw std::runtime_error("Iterate(): range is not sorted!");
#endif

    // todo: pass placeholder vector, or turn into std::range
    using ResultType = std::tuple<EntityId, T1*, T2*>;

    std::vector<ResultType> result;
    if constexpr(std::contiguous_iterator<TIter>)
        result.reserve(std::distance(begin, end));

    auto hint1 = ecs.begin<T1>();
    auto hint2 = ecs.begin<T2>();
    const auto it1End = ecs.end<T1>();
    const auto it2End = ecs.end<T2>();
    for(auto it = begin; it != end; ++it)
    {
        const EntityId id = *it;

        hint1 = ecs.ComponentSystem<T1>::lower_bound_with_hint(id, hint1);
        T1* t1Ptr = (hint1 != it1End) && (hint1->first == id) ? &(hint1->second) : nullptr;

        hint2 = ecs.ComponentSystem<T2>::lower_bound_with_hint(id, hint2);
        T2* t2Ptr = (hint2 != it2End) && (hint2->first == id) ? &(hint2->second) : nullptr;

        result.push_back({id, t1Ptr, t2Ptr});
    }
    return result;
}

template <typename T1, typename T2, typename T3, typename TEcs, typename TIter>
auto Iterate3d(TEcs& ecs, const TIter begin, const TIter end)
{
#ifdef _DEBUG
    if(!std::is_sorted(begin, end))
        throw std::runtime_error("Iterate(): range is not sorted!");
#endif

    // todo: pass placeholder vector, or turn into std::range
    using ResultType = std::tuple<EntityId, T1*, T2*, T3*>;

    std::vector<ResultType> result;
    if constexpr(std::contiguous_iterator<TIter>)
        result.reserve(std::distance(begin, end));

    auto hint1 = ecs.begin<T1>();
    auto hint2 = ecs.begin<T2>();
    auto hint3 = ecs.begin<T3>();
    const auto it1End = ecs.end<T1>();
    const auto it2End = ecs.end<T2>();
    const auto it3End = ecs.end<T3>();
    for(auto it = begin; it != end; ++it)
    {
        const EntityId id = *it;

        hint1 = ecs.ComponentSystem<T1>::lower_bound_with_hint(id, hint1);
        T1* t1Ptr = (hint1 != it1End) && (hint1->first == id) ? &(hint1->second) : nullptr;

        hint2 = ecs.ComponentSystem<T2>::lower_bound_with_hint(id, hint2);
        T2* t2Ptr = (hint2 != it2End) && (hint2->first == id) ? &(hint2->second) : nullptr;

        hint3 = ecs.ComponentSystem<T3>::lower_bound_with_hint(id, hint3);
        T3* t3Ptr = (hint3 != it3End) && (hint3->first == id) ? &(hint3->second) : nullptr;

        result.push_back({id, t1Ptr, t2Ptr, t3Ptr});
    }
    return result;
}

} // namespace algorithms
} // namespace ecs
} // namespace vic