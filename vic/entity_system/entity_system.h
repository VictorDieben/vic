#pragma once

#include <cassert>
#include <map>
#include <tuple>
#include <vector>

#include "vic/entity_system/algorithms.h"
#include "vic/entity_system/definitions.h"
#include "vic/memory/flat_map.h"

namespace vic
{
namespace entity
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

    template <typename T2>
    void Add(const T2 component)
    {
        assert(mSystem && mId);
        mSystem->Add<T2>(mId, component);
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

    T& Add(const EntityId id, const T& component)
    {
        mComponents[id] = component;
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
class EntitySystem : public ComponentSystem<TComponents>...
{
public:
    // todo: better way to define EntityHandle(decltype(*this))
    using Handle = EntityHandle<EntitySystem<TComponents...>>;

    EntitySystem() = default;

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
        RemoveEntityRecursive<TComponents...>(id); //
    }

    template <typename T>
    void Add(EntityId id, T item)
    {
        ComponentSystem<T>::Add(id, item);
    }

    template <typename T>
    T& Get(EntityId id)
    {
        return ComponentSystem<T>::Get(id);
    }

    template <typename T>
    T* TryGet(EntityId id)
    {
        return ComponentSystem<T>::TryGet(id);
    }

    template <typename T>
    bool Has(EntityId id) const
    {
        return ComponentSystem<T>::Has(id);
    }

    bool HasAny(EntityId id) const
    {
        return HasAnyRecursive<TComponents...>(id); //
    }

    template <typename T>
    bool Remove(EntityId id)
    {
        return ComponentSystem<T>::Remove(id);
    }

    template <typename T>
    std::size_t Size() const
    {
        return ComponentSystem<T>::Size();
    }

    // todo: remove list of entities

    template <typename T>
    auto begin()
    {
        return ComponentSystem<T>::begin();
    }

    template <typename T>
    auto end()
    {
        return ComponentSystem<T>::end();
    }

    template <typename T>
    auto begin() const
    {
        return ComponentSystem<T>::cbegin();
    }

    template <typename T>
    auto end() const
    {
        return ComponentSystem<T>::cend();
    }

    // filter entities that have two specific components
    // todo: filter on 3 or more? combine one filter with another?
    template <typename T1, typename T2>
    auto Filter()
    {
        return Filter<T1, T2>(*this);
    }

    template <typename T, typename TIter>
    auto Insert(TIter begin, TIter end)
    {
        ComponentSystem<T>::Insert(begin, end);
    }

    void Relabel()
    {
        EntityId mNewCounter{1};

        if(HasAny(mNewCounter))
        { // entity exists
        }
        else
        { }
    }

private:
    EntityId mEntityCounter{1}; // reserve 0 for emtpy, not sure if I need that

    // todo: set of occupied entity ids?

    template <typename T>
    void RemoveEntityRecursive(EntityId id)
    {
        ComponentSystem<T>::Remove(id);
    }
    template <typename T1, typename T2, typename... Ts>
    void RemoveEntityRecursive(EntityId id)
    {
        ComponentSystem<T1>::Remove(id);
        RemoveEntityRecursive<T2, Ts...>(id);
    }

    //
    //
    //

    template <typename T>
    bool HasAnyRecursive(EntityId id) const
    {
        return ComponentSystem<T>::Has(id);
    }
    template <typename T1, typename T2, typename... Ts>
    bool HasAnyRecursive(EntityId id) const
    {
        if(ComponentSystem<T1>::Has(id))
            return true;
        else
            return HasAnyRecursive<T2, Ts...>(id);
    }

    //
    //
    //

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

} // namespace entity
} // namespace vic