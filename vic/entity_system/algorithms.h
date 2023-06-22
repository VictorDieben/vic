#pragma once

#include "vic/entity_system/definitions.h"
#include <cassert>
#include <ranges>
#include <tuple>
#include <vector>

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
auto Iterate(TEcs& ecs, TIter begin, TIter end)
{
    // todo: check if range is sorted in debug?
    assert(std::is_sorted(begin, end));

    // todo: pass placeholder vector, or turn into std::range
    using ResultType = std::pair<EntityId, TComponent*>;
    std::vector<ResultType> result;
    auto hint = ecs.begin<TComponent>();
    for(auto it = begin; it != end; ++it)
    {
        auto cur = ecs.ComponentSystem<TComponent>::lower_bound_with_hint(*it, hint);
        if(cur->first == *it)
            result.push_back({cur->first, &(cur->second)});
        hint = cur;
    }
    return result;
}

} // namespace algorithms
} // namespace ecs
} // namespace vic