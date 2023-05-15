#pragma once

#include <cassert>
#include <map>
#include <tuple>
#include <vector>

#include "vic/entity_system/algorithms.h"
#include "vic/entity_system/definitions.h"
#include "vic/memory/flat_map.h"
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

    template <typename TComponent>
    EntityHandle<T>& Add(auto&&... args)
    {
        assert(mSystem && mId);
        mSystem->Add<TComponent>(mId, std::forward<decltype(args)>(args)...);
        return *this; // return this, so you can chain adds
    }

    template <typename T2>
    T2& Get()
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
    // std::map<EntityId, T> mComponents{};
    vic::memory::FlatMap<EntityId, T> mComponents{};

public:
    // todo: make a custom component system, that does not rely on std::map
    // example: https://austinmorlan.com/posts/entity_component_system/
    // maybe make a custom container, a densely packed map or something
    using ComponentType = T;

    using iterator = typename decltype(mComponents)::iterator;
    using const_iterator = typename decltype(mComponents)::const_iterator;

    using value_type = std::pair<const EntityId, T>;

    T& Add(EntityId id, auto&&... args)
    {
        mComponents[id] = T{args...};
        return mComponents[id];
    }

    T& Get(const EntityId id)
    {
        auto it = mComponents.find(id);
        assert(it != mComponents.end());
        return it->second;
    }

    const T& Get(const EntityId id) const
    {
        auto it = mComponents.find(id);
        assert(it != mComponents.end());
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

    const_iterator begin() const { return mComponents.cbegin(); }
    const_iterator end() const { return mComponents.cend(); }

    const_iterator cbegin() const { return mComponents.cbegin(); }
    const_iterator cend() const { return mComponents.cend(); }

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
};

template <typename... TComponents>
class ECS : public ComponentSystem<TComponents>...
{
public:
    static_assert(templates::IsUnique<TComponents...>() && "Component list is not Unique!");

    // todo: better way to define EntityHandle(decltype(*this))
    using Handle = EntityHandle<ECS<TComponents...>>;

    ECS() = default;

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
    T& Get(EntityId id)
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
        return ComponentSystem<T>::cbegin();
    }

    template <typename T>
    auto end() const
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        return ComponentSystem<T>::cend();
    }

    // filter entities that have two specific components
    // todo: filter on 3 or more? combine one filter with another?
    template <typename T1, typename T2>
    auto Filter()
    {
        static_assert(templates::Contains<T1, TComponents...>(), "Unknown component T1");
        static_assert(templates::Contains<T2, TComponents...>(), "Unknown component T2");
        return Filter<T1, T2>(*this);
    }

    template <typename T, typename TIter>
    auto Insert(TIter begin, TIter end)
    {
        static_assert(templates::Contains<T, TComponents...>(), "Unknown component T");
        ComponentSystem<T>::Insert(begin, end);
    }

    void Relabel()
    {
        EntityId mNewCounter{1};

        if(HasAny(mNewCounter))
        { // Entity exists
        }
        else
        { // Entity without components, can be removed
        }
    }

    template <typename T>
    static constexpr bool ContainsComponent()
    {
        return templates::Contains<T, TComponents...>();
    }

private:
    EntityId mEntityCounter{1}; // reserve 0 for emtpy, not sure if I need that

    //template <typename TFunctor, typename T>
    //void ForeachComponent(TFunctor& functor) const
    //{
    //    functor.Do<T>(); //
    //}
    //template <typename TFunctor, typename T1, typename T2, typename... Ts>
    //bool ForeachComponent(TFunctor& functor) const
    //{
    //    functor.Do<T1>(); // hacky way to call lambda with template argument
    //    return ForeachComponent<T2, Ts...>(functor);
    //}
};

template <typename TEcs, typename... TComponents>
class System
{
public:
    using ECSType = TEcs;

    static_assert((TEcs::template ContainsComponent<std::remove_cv_t<TComponents>>() && ...) && "ECS does not contain all required components!");

    System(TEcs& system)
        : mSystem(system)
    { }

protected:
    TEcs& mSystem;
};

} // namespace ecs
} // namespace vic