#pragma once

#include <map>

namespace vic
{

using EntityId = uint64_t;

template <typename T>
struct Entity
{
	using SystemType = T;

	Entity() = default;
	Entity(EntityId id, T* system)
		: mId(id)
		, mSystem(system) {}

	// cast to EntityId
	operator EntityId() const { return mId; }
	EntityId Id() const { return mId; }

	template <typename T2>
	void Add(T2 component) { mSystem->Add<T2>(mId, component); }

	template <typename T2>
	T2& Get() const { return mSystem->Get<T2>(mId); }

	template <typename T2>
	bool Has() const { return mSystem->Has<T2>(mId); }

	template <typename T2>
	bool Remove() { return mSystem->Remove<T2>(mId); }

private:
	EntityId mId{ 0 };
	T* mSystem{ nullptr };
};

template <typename T>
class ComponentSystem
{
public:
	ComponentSystem() = default;

	void Add(EntityId id, T component)
	{
		// assert(mComponents.find(id) == mComponents.end()); // optional add? 
		mComponents[id] = component;
	}

	bool Has(EntityId id)
	{
		return mComponents.find(id) != mComponents.end();
	}

	bool Remove(EntityId id)
	{
		auto it = mComponents.find(id);
		if (it == mComponents.end())
			return false;
		mComponents.erase(it);
		return true;
	}

	T& Get(EntityId id)
	{
		auto it = mComponents.find(id);
		// assert(it != mComponents.end());
		return it->second;
	}

	auto begin() const { return mComponents.begin(); }
	auto end() const { return mComponents.end(); }

private:
	std::map<EntityId, T> mComponents{}; // TODO: actual system instead of a simple std::map
};


template <typename ... Ts>
class BaseEntitySystem
	: protected ComponentSystem<Ts>...
{
public:
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
};

// entity system is split up into base and derived in order to let Entity_t point to a defined class
// entities can only see the functions inside the BaseEntitySystem
template <typename ... Ts>
class EntitySystem
	: public BaseEntitySystem<Ts...>
{
public:
	using Entity_t = Entity<BaseEntitySystem<Ts...>>;

	Entity_t NewEntity()
	{
		mEntId++;
		Entity_t newEntity(mEntId, this);
		return newEntity;
	}

	// filter entities that have two specific components
	template < typename T1, typename T2>
	std::vector<EntityId> Filter() const
	{
		std::vector<EntityId> result{};
		auto it1 = ComponentSystem<T1>::begin();
		auto it2 = ComponentSystem<T2>::begin();
		auto it1End = ComponentSystem<T1>::end();
		auto it2End = ComponentSystem<T2>::end();
		while (true)
		{
			if (it1 == it1End || it2 == it2End)
				break;

			if (it1->first == it2->first)
				result.push_back(it1->first);

			if (it1->first < it2->first)
				it1 = std::next(it1);
			else
				it2 = std::next(it2);

		}
		return result;
	}
private:

	EntityId mEntId{ 0 };
};



}