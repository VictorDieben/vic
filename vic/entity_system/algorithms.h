#pragma once

#include <tuple>
#include <vector>

#include "vic/entity_system/definitions.h"

namespace vic
{
namespace entity
{

template <typename T1, typename T2, typename TSystem, typename TFunctor>
void FilterForeach(TSystem& system, TFunctor functor)
{
    auto it1 = system.ComponentSystem<T1>::begin();
    auto it2 = system.ComponentSystem<T2>::begin();
    const auto it1End = system.ComponentSystem<T1>::end();
    const auto it2End = system.ComponentSystem<T2>::end();

    while(it1 != it1End && it2 != it2End)
    {
        if(it1->first == it2->first)
        {
            functor(it1->first, it1->second, it2->second);
            it1 = std::next(it1);
        }
        else if(it1->first < it2->first)
            it1 = std::next(it1);
        else
            it2 = std::next(it2);
    }
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

template <typename T1, typename T2, typename TSystem>
auto Filter(TSystem& system)
{
    using ResultType = std::tuple<EntityId, T1*, T2*>;
    std::vector<ResultType> result{};
    return Filter<T1, T2>(system, result);
}

} // namespace entity
} // namespace vic