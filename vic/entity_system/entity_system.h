#pragma once

#include <algorithm>
#include <cassert>
#include <map>
#include <ranges>
#include <string>
#include <tuple>
#include <vector>

#include "vic/entity_system/algorithms.h"
#include "vic/entity_system/definitions.h"
#include "vic/memory/flat_map.h"
#include "vic/utils/map_iterate.h"
#include "vic/utils/templates.h"

namespace vic
{
namespace ecs
{

template <typename T>
struct EntityHandle
{
    using SystemType = T;

    EntityHandle() = default;
    EntityHandle(const EntityId id, T* system)
        : mId(id)
        , mSystem(system)
    { }

    // cast to EntityId
    operator EntityId() const { return mId; }
    EntityId Id() const { return mId; }

    operator bool() const { return (mId != 0) && (mSystem != nullptr); }

    template <typename TComponent, typename... Args>
    EntityHandle<T> Add(Args&&... args)
    {
        static_assert(SystemType::template ContainsComponent<TComponent>(), "Unknown component T");
        assert(mSystem && mId);
        mSystem->Add<TComponent>(mId, std::forward<Args>(args)...);
        return *this; // return copy of self
    }

    template <typename TComponent>
    EntityHandle<T> Set(const TComponent& component)
    {
        assert(mSystem && mId);
        mSystem->Set<TComponent>(mId, component);
        return *this; // return copy of self
    }

    template <typename T2>
    T2& Get()
    {
        assert(mSystem && mId);
        return mSystem->Get<T2>(mId);
    }

    template <typename T2>
    const T2& Get() const
    {
        assert(mSystem && mId);
        return mSystem->Get<T2>(mId);
    }

    template <typename T2>
    T2* TryGet()
    {
        assert(mSystem && mId);
        return mSystem->TryGet<T2>(mId);
    }

    template <typename T2>
    const T2* TryGet() const
    {
        assert(mSystem && mId);
        return mSystem->TryGet<T2>(mId);
    }

    template <typename T2>
    bool Has() const
    {
        assert(mSystem && mId);
        return mSystem->Has<T2>(mId);
    }

    bool HasAny() const
    {
        assert(mSystem && mId);
        return mSystem->HasAny(mId);
    }

    template <typename T2>
    bool Remove()
    {
        assert(mSystem && mId);
        return mSystem->Remove<T2>(mId);
    }

private:
    EntityId mId{0};
    T* mSystem{nullptr};
};

template <typename T>
class ComponentSystem
{
private:
    // todo: think about optimized map container (specialized for many/few large/small objects?)
    // a flat map (like boost::flat_map) might be a better solution,
    // as we really need fast iteration, but don't really care about insert/remove
    std::map<EntityId, T> mComponents{};
    // vic::memory::FlatMap<EntityId, T> mComponents{};

    static constexpr auto FindLowerBound = [](const auto& item, const EntityId value) { return item.first < value; };

    static_assert(std::is_move_constructible_v<T> && std::is_move_assignable_v<T>, "T cannot be moved!");

public:
    // todo: make a custom component system, that does not rely on std::map
    // example: https://austinmorlan.com/posts/entity_component_system/
    // maybe make a custom container, a densely packed map or something
    using ComponentType = T;

    using iterator = typename decltype(mComponents)::iterator;
    using const_iterator = typename decltype(mComponents)::const_iterator;

    using value_type = std::pair<const EntityId, T>;

    T& Add(const EntityId id, auto&&... args)
    {
        // todo: find out if piecewise construct is worth it / how to make it work with any type
        //auto res = mComponents.emplace(std::piecewise_construct, //
        //                               std::forward_as_tuple(id),
        //                               std::forward_as_tuple(args...));

        // auto res = mComponents.emplace(std::pair(id, std::move(T{std::forward<decltype(args)>(args)...})));
        // return res.first->second;

        auto res = mComponents.emplace(id, T{std::forward<decltype(args)>(args)...});
        return res.first->second;
    }

    T& Set(const EntityId id, const T& component)
    {
        mComponents[id] = component;
        return mComponents[id];
    }

    T& Get(const EntityId id)
    {
        auto it = mComponents.find(id);
#ifdef _DEBUG
        if(it == mComponents.end())
            throw std::runtime_error(std::string("Entity ") + std::to_string(id) + " has no " + typeid(T).name());
#endif
        return it->second;
    }

    const T& Get(const EntityId id) const
    {
        auto it = mComponents.find(id);
#ifdef _DEBUG
        if(it == mComponents.end())
            throw std::runtime_error(std::string("Entity ") + std::to_string(id) + " has no " + typeid(T).name());
#endif
        return it->second;
    }

    T* TryGet(const EntityId id)
    {
        auto it = mComponents.find(id);
        if(it == mComponents.end())
            return nullptr;
        else
            return &(it->second);
    }

    const T* TryGet(const EntityId id) const
    {
        auto it = mComponents.find(id);
        if(it == mComponents.end())
            return nullptr;
        else
            return &(it->second);
    }

    bool Has(const EntityId id) const { return mComponents.find(id) != mComponents.end(); }

    bool Remove(const EntityId id)
    {
        auto it = mComponents.find(id);
        if(it == mComponents.end())
            return false;
        mComponents.erase(it);
        return true;
    }

    std::size_t Size() const { return mComponents.size(); }

    iterator begin() { return mComponents.begin(); }
    iterator end() { return mComponents.end(); }

    const_iterator begin() const { return mComponents.begin(); }
    const_iterator end() const { return mComponents.end(); }

    const_iterator cbegin() const { return mComponents.cbegin(); }
    const_iterator cend() const { return mComponents.cend(); }

    iterator lower_bound(const EntityId id) { return std::lower_bound(mComponents.begin(), mComponents.end(), id, FindLowerBound); }
    const_iterator lower_bound(const EntityId id) const { return std::lower_bound(mComponents.begin(), mComponents.end(), id, FindLowerBound); }

    iterator lower_bound_with_hint(const EntityId id, iterator hint) { return std::lower_bound(hint, mComponents.end(), id, FindLowerBound); }
    iterator lower_bound_with_hint(const EntityId id, iterator hint) const { return std::lower_bound(hint, mComponents.cend(), id, FindLowerBound); }

    template <typename TIter>
    void Insert(TIter begin, TIter end)
    {
        using TValue = std::iter_value_t<TIter>;
        static_assert(std::is_same_v<TValue, value_type>);

        // todo: only activate for debug builds:
        // Verify that the range is sorted
        if(begin != end)
        {
            auto prev = begin;
            for(auto it = std::next(begin); it != end; ++it)
                assert(prev->first < it->first);
        }

        auto hint = begin;
        for(auto it = begin; it != end; ++it)
        {
            auto res = mComponents.insert(*it);
        }
    }

    bool Relabel(const EntityId from, const EntityId to)
    {
        // todo: this should really only ever be called from the ecs itself
        if(from == to)
            return true; // no need to do anything when relabeling to the same label

        auto it1 = mComponents.find(from);
        if(it1 == mComponents.end())
            return false; // item to relabel cannot be found

        auto it2 = mComponents.find(to);
        if(it2 != mComponents.end())
            return false; // cannot relabel to an existing name

        // todo: this can be optimized for sorted lists (flat map), if we know that the order will not change
        // todo: move? not needed for pod
        mComponents[to] = mComponents.at(from);
        mComponents.erase(from);
        return true;
    }

    EntityId Minimum() const { return mComponents.size() == 0 ? std::numeric_limits<EntityId>::max() : mComponents.begin()->first; }
    EntityId Maximum() const { return mComponents.size() == 0 ? std::numeric_limits<EntityId>::min() : std::prev(mComponents.end())->first; }

    auto& Data() { return mComponents; };
    const auto& Data() const { return mComponents; };
};

using ComponentIndexType = int;

template <typename... TComponents>
class ECS : public ComponentSystem<TComponents>...
{
public:
    static_assert(templates::IsUnique<TComponents...>(), "Component list is not Unique!");

    // todo: better way to define EntityHandle(decltype(*this))
    using Handle = EntityHandle<ECS<TComponents...>>;

    ECS() = default;

    // disable move and copy, because we will pass the ecs adress along with handles etc.
    ECS(const ECS&) = delete;
    ECS(ECS&&) = delete;
    ECS& operator=(const ECS&) = delete;
    ECS& operator=(ECS&&) = delete;

    Handle NewEntity()
    {
        EntityHandle newEntity(mEntityCounter, this);
        mEntityCounter++;
        return newEntity;
    }

    Handle GetEntity(EntityId id)
    {
        // note: there is no way for us to know if id actually exists.
        // No list of valid ids is stored anywhere.
        // we could make EntityId only constructable by EntitySystem,
        // that waywe can at least be sure that it was valid at some point in time

        // other note: if a user makes a deserializer (from json or something), they will want to be able to create objects with specific ids.
        // GetEntity() could be used (bit hacky but whatever),
        // but the counter will need to be updated if the id is larger than any we've seen so far.
        if(id >= mEntityCounter)
            mEntityCounter = id + 1;
        return EntityHandle(id, this);
    }

    void RemoveEntity(EntityId id)
    {
        (ComponentSystem<TComponents>::Remove(id), ...); //
    }

    template <typename T>
    void Add(EntityId id, auto&&... args)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        ComponentSystem<T>::Add(id, std::forward<decltype(args)>(args)...);
    }

    template <typename T>
    void Set(EntityId id, const T& component)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        ComponentSystem<T>::Set(id, component);
    }

    template <typename T>
    T& Get(EntityId id)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::Get(id);
    }

    template <typename T>
    const T& Get(EntityId id) const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::Get(id);
    }

    template <typename T>
    T* TryGet(EntityId id)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::TryGet(id);
    }

    template <typename T>
    const T* TryGet(EntityId id) const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::TryGet(id);
    }

    template <typename T>
    bool Has(EntityId id) const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::Has(id);
    }

    bool HasAny(EntityId id) const
    {
        return (ComponentSystem<TComponents>::Has(id) || ...); //
    }

    template <typename T>
    bool Remove(EntityId id)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::Remove(id);
    }

    template <typename T, typename TIter>
    bool RemoveRange(const TIter begin, const TIter end)
    {
        assert(std::is_sorted(begin, end));
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");

        // todo: optimize, [begin; end> is sorted
        bool res = true;
        for(auto it = begin; it != end; ++it)
            res = res && Remove<T>(*it);
        return res;
    }

    template <typename T, typename TIterable>
    bool RemoveRange(const TIterable& iterable)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return RemoveRange<T>(iterable.begin(), iterable.end());
    }

    template <typename T>
    std::size_t Size() const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::Size();
    }

    // todo: remove list of entities

    template <typename T>
    auto begin()
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::begin();
    }

    template <typename T>
    auto end()
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::end();
    }

    template <typename T>
    auto begin() const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::begin();
    }

    template <typename T>
    auto end() const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::end();
    }

    template <typename T>
    auto cbegin() const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::cbegin();
    }

    template <typename T>
    auto cend() const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::cend();
    }

    EntityId Minimum() const
    {
        return std::min({ComponentSystem<TComponents>::Minimum()...}); //
    }

    EntityId Maximum() const
    {
        return std::max({ComponentSystem<TComponents>::Maximum()...}); //
    }

    // todo: Filter with buffer arguments
    template <typename T>
    auto Filter()
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return algorithms::Filter<T>(*this);
    }

    template <typename T1, typename T2>
    auto Filter()
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        return algorithms::Filter<T1, T2>(*this);
    }

    template <typename T1, typename T2, typename T3>
    auto Filter()
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        static_assert(templates::Contains<T3, TComponents...>(), "Unknown component T3");
        return algorithms::Filter<T1, T2, T3>(*this);
    }

    //// Overlap
    //template <typename T1, typename T2>
    //auto Overlap()
    //{
    //    static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
    //    static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
    //    return vic::Overlap(ComponentSystem<T1>::Data(), ComponentSystem<T2>::Data());
    //}

    // Iterate 1d

    template <typename T, typename TIter>
    auto Iterate(const TIter begin, const TIter end)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return algorithms::Iterate<T>(*this, begin, end);
    }

    template <typename T, typename TIterable>
    auto Iterate(const TIterable& iterable)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return algorithms::Iterate<T>(*this, iterable.begin(), iterable.end());
    }

    template <typename T, typename TIter>
    auto Iterate(const TIter begin, const TIter end) const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        const auto& thisRef = *this;
        return algorithms::Iterate<T>(thisRef, begin, end);
    }

    template <typename T, typename TIterable>
    auto Iterate(const TIterable& iterable) const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return algorithms::Iterate<T>(*this, iterable.begin(), iterable.end());
    }

    //

    template <typename T1, typename T2, typename TIter>
    auto Iterate2d(const TIter begin, const TIter end)
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        return algorithms::Iterate2d<T1, T2>(*this, begin, end);
    }

    template <typename T1, typename T2, typename TIterable>
    auto Iterate2d(const TIterable& iterable)
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        return algorithms::Iterate2d<T1, T2>(*this, iterable.begin(), iterable.end());
    }

    //

    template <typename T1, typename T2, typename T3, typename TIter>
    auto Iterate3d(const TIter begin, const TIter end)
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        static_assert(templates::Contains<T3, TComponents...>(), "Unknown component T3");
        return algorithms::Iterate3d<T1, T2, T3>(*this, begin, end);
    }

    template <typename T1, typename T2, typename T3, typename TIterable>
    auto Iterate3d(const TIterable& iterable)
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        static_assert(templates::Contains<T3, TComponents...>(), "Unknown component T3");
        return algorithms::Iterate3d<T1, T2, T3>(*this, iterable.begin(), iterable.end());
    }

    //

    template <typename T, typename TIter>
    auto Insert(TIter begin, TIter end)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        ComponentSystem<T>::Insert(begin, end);
    }

    void Relabel()
    {
        const auto min = Minimum();
        const auto max = Maximum();
        EntityId newCounter{1};
        for(EntityId i = Minimum(); i <= Maximum(); ++i)
        {
            if(HasAny(i))
            {
                (ComponentSystem<TComponents>::Relabel(i, newCounter), ...);
                newCounter++;
            }
        }
        mEntityCounter = newCounter;
    }

    template <typename T>
    static constexpr bool ContainsComponent()
    {
        return templates::Contains<T, TComponents...>();
    }

    template <typename TFunctor>
    void ForeachComponentType(TFunctor functor)
    {
        // call functor with each of the component types as template argument
        (functor.template operator()<TComponents>(), ...);
    }

    template <typename TFunctor>
    void ForeachEntity(TFunctor functor)
    {
        const auto minimum = Minimum();
        const auto maximum = Maximum();
        for(auto i = minimum; i <= maximum; ++i)
            if(HasAny(i))
                functor(i); // todo: inefficient, there might be a better way
    }

    template <typename T>
    static constexpr ComponentIndexType GetIndex()
    {
        return (ComponentIndexType)templates::GetIndex<T, TComponents...>();
    }

    template <int index>
    using NthType = decltype(std::get<index>(std::tuple<TComponents...>));

private:
    EntityId mEntityCounter{1}; // reserve 0 for emtpy, not sure if I need that
};

template <typename TEcs, typename... TComponents>
class System
{
public:
    using ECSType = TEcs;

    static_assert((TEcs::template ContainsComponent<std::remove_cv_t<TComponents>>() && ...), "ECS does not contain all required components!");

    System(TEcs& ecs)
        : mEcs(ecs)
    { }

    template <typename T>
    static constexpr bool ContainsComponent()
    {
        return templates::Contains<T, TComponents...>();
    }

protected:
    // annoying small protected segment for helper function

    template <typename TOtherSystem, typename TComponent>
    constexpr static bool IsBlockedByHelper()
    {
        using TBase = std::remove_cv_t<TComponent>;
        if constexpr(std::is_const_v<TComponent>)
        {
            // other system cannot contain a non-const version of TComponent
            constexpr bool containtsT = (TOtherSystem::template ContainsComponent<TBase>());
            return containtsT;
        }
        else
        {
            // if we have a non-const version, other system should not contain TComponent at all
            constexpr bool containsConstT = TOtherSystem::template ContainsComponent<const TBase>();
            constexpr bool containsNonConstT = TOtherSystem::template ContainsComponent<TBase>();
            return containsConstT || containsNonConstT;
        }
    }

public:
    template <typename TOtherSystem>
    constexpr static bool IsBlockedBy()
    {
        // todo: check that ECS type is the same?
        // for each component type,
        // if we use a const view, check that TSystem does not write to it (non-const)
        // if we use a non-const view, the other system cannot have this component at all

        return ((IsBlockedByHelper<TOtherSystem, TComponents>() || ...));
    }

protected:
    TEcs& mEcs;
};

} // namespace ecs
} // namespace vic