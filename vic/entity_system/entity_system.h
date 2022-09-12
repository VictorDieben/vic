#pragma once

#include <cassert>
#include <map>
#include <tuple>
#include <vector>

#include "vic/entity_system/algorithms.h"
#include "vic/entity_system/definitions.h"

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
    bool Has()
    {
        assert(mSystem && mId);
        return mSystem->Has<T2>(mId);
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
public:
    // todo: make a custom component system, that does not rely on std::map
    // example: https://austinmorlan.com/posts/entity_component_system/
    // maybe make a custom container, a densely packed map or something
    using ComponentType = T;
    using Iterator = std::map<EntityId, T>::iterator;

    void Add(const EntityId id, const T& component) { mComponents[id] = component; }

    T& Get(const EntityId id)
    {
        auto it = mComponents.find(id);
        assert(it != mComponents.end());
        return it->second;
    }

    bool Has(const EntityId id) { return mComponents.find(id) != mComponents.end(); }

    bool Remove(const EntityId id)
    {
        auto it = mComponents.find(id);
        if(it == mComponents.end())
            return false;
        mComponents.erase(it);
        return true;
    }

    Iterator begin() { return mComponents.begin(); }
    Iterator end() { return mComponents.end(); }

    //Iterator begin() const { return mComponents.begin(); }
    //Iterator end() const { return mComponents.end(); }

    const Iterator cbegin() const { return mComponents.begin(); }
    const Iterator cend() const { return mComponents.end(); }

private:
    // todo: think about optimized map container (specialized for many/few large/small objects?)
    // a flat map (like boost::flat_map) might be a better solution, 
    // as we really need fast iteration, but don't really care about insert/remove
    std::map<EntityId, T> mComponents{};
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

    void RemoveEntity(EntityId id) { RemoveEntityRecursive<TComponents...>(id); }

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
    bool Has(EntityId id)
    {
        return ComponentSystem<T>::Has(id);
    }

    template <typename T>
    bool Remove(EntityId id)
    {
        return ComponentSystem<T>::Remove(id);
    }

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
        return ComponentSystem<T>::begin();
    }

    template <typename T>
    auto end() const
    {
        return ComponentSystem<T>::end();
    }

    // filter entities that have two specific components
    // todo: filter on 3 or more? combine one filter with another?
    template <typename T1, typename T2>
    auto Filter()
    {
        return Filter<T1, T2>(*this);
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
};

} // namespace entity
} // namespace vic