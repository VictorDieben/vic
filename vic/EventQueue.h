#pragma once

#include <vector>
#include <tuple>
#include <memory>
#include <deque>
#include <functional>

namespace vic
{

// base event
struct Event
{
	Event() = default;
	virtual ~Event() = default;
};

class EventListener
{
public:
	EventListener() = default;
	virtual ~EventListener() = default;

	virtual void OnEvent(Event& event) = 0;
};

class PolymorphicEventQueue
{
public:
	PolymorphicEventQueue() = default;
	~PolymorphicEventQueue() = default;

	void Add(std::unique_ptr<Event> event)
	{
		mEvents.push_back(std::move(event));
	}

	// emplace() is the same as add, only do we not need to move the newly created object,
	// at the cost of an extra template function
	template<typename T, typename... Targs>
	void EmplaceBack(Targs&&... args)
	{
		mEvents.emplace_back(new T(std::forward<Targs>...));
	}


	bool Next()
	{
		if (!mEvents.empty())
		{
			std::unique_ptr<Event> event = std::move(mEvents.front());	// this moves the value out, front is now nullptr
			mEvents.pop_front();										// remove the nullptr
			for (EventListener* listener : mListeners)
				listener->OnEvent(*event);
			return true;
		}
		else
			return false;
	}

	void AddListener(EventListener& listener)
	{
		// add the listener to the list, make sure that it is unique
		assert(std::find(mListeners.begin(), mListeners.end(), &listener) == mListeners.end());
		mListeners.push_back(&listener);
	}
	void RemoveListener(EventListener& listener)
	{
		// remove all occurances of listener
		mListeners.erase(std::find(mListeners.begin(), mListeners.end(), &listener));
	}

protected:

private:
	std::deque<std::unique_ptr<Event>> mEvents{};

	// todo: separate vector for every type of event? useful if we have a lot of listeners
	std::vector <EventListener*> mListeners{};
};

template <typename Tevent>
class SpecializedListener : public EventListener
{
public:
	SpecializedListener(PolymorphicEventQueue& queue, std::function<void(Tevent&)> lambda)
		: mQueue(queue)
		, mLambda(lambda)
	{
		queue.AddListener(*this);
	}

	~SpecializedListener() override
	{
		mQueue.RemoveListener(*this);
	}

	void OnEvent(Event& event) override
	{
		if (auto* castEvent = dynamic_cast<Tevent*>(&event)) {
			mLambda(*castEvent);
		}
	}
private:
	PolymorphicEventQueue& mQueue;
	std::function<void(Tevent&)> mLambda;
};

}