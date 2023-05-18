#pragma once

#include <set>

namespace vic
{
namespace ecs
{

// A collection is a list of (sorted) entity ids.
// each of the entities is supposed to have all of the types in TComponents
template <typename... TComponents>
class Collection
{
public:
    using TupleType = std::tuple<TComponents...>;
    constexpr static bool TempIsCollection = true; // todo: find better solution

    Collection() = default;
    Collection(auto&&... args)
        : mItems{std::forward<decltype(args)>(args)...}
    { }

    template <typename TEcs>
    bool Verify(const TEcs& ecs)
    {
        bool result = true;
        for(const auto& id : mItems)
            result = result && (ecs.template Has<TComponents>(id) && ...); // todo: optimize
        return result;
    }

    auto begin() { return mItems.begin(); }
    auto end() { return mItems.end(); }

    auto begin() const { return mItems.cbegin(); }
    auto end() const { return mItems.cend(); }

    auto Insert(const EntityId id) { mItems.insert(id); }
    auto Remove(const EntityId id) { mItems.erase(id); }
    bool Has(const EntityId id) const { return mItems.contains(id); }

private:
    // todo: flat set type
    std::set<EntityId> mItems{};
};

template <typename T>
concept ConceptCollection = requires(T component)
{
    T::TempIsCollection == true;
    component.Verify(int{}); // todo: verify that the argument is an ecs
};

} // namespace ecs
} // namespace vic