#pragma once

#include "vic/utils/templates.h"

namespace vic
{
// attempt at simplifying the ecs folder. do not use yet.

//

template <typename TKey, typename TValue>
class SubMap
{
public:
private:
};

template <typename TKey, typename... TValues>
class NMap : public SubMap<TValues>...
{
public:
    template <typename TValue>
        requires templates::ConceptContained<TValue, TValues>
    std::pair<TKey, TValue&> Filter()
    {
        return {};
    }

private:
};

} // namespace vic