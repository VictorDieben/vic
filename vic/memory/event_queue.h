#pragma once

#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <tuple>
#include <vector>

namespace vic
{

namespace memory
{

// base event
struct Event
{
    Event() = default;
    virtual ~Event() = default;
};

class BaseEventListener
{
public:
    BaseEventListener() = default;
    virtual ~BaseEventListener() = default;

    virtual void OnEvent(Event& event) = 0;
};

class EventQueue
{
public:
    EventQueue() = default;
    ~EventQueue() = default;

    void Add(std::unique_ptr<Event> event) { mEvents.push_back(std::move(event)); }

    // emplace() is the same as add, only do we not need to move the newly created object,
    // at the cost of an extra template function
    template <typename T, typename... Targs>
    void EmplaceBack(Targs&&... args)
    {
        mEvents.emplace_back(new T(std::forward<Targs>...));
    }

    bool Next()
    {
        if(!mEvents.empty())
        {
            std::unique_ptr<Event> event = std::move(mEvents.front()); // this moves the value out, front is now nullptr
            mEvents.pop_front(); // remove the nullptr
            for(BaseEventListener* listener : mListeners)
                listener->OnEvent(*event); // todo: filter listeners per type
            return true;
        }
        else
            return false;
    }

    void AddListener(BaseEventListener& listener)
    {
        // add the listener to the list, make sure that it is unique
        assert(std::find(mListeners.begin(), mListeners.end(), &listener) == mListeners.end());
        mListeners.push_back(&listener);
    }
    void RemoveListener(BaseEventListener& listener)
    {
        // remove all occurances of listener
        mListeners.erase(std::find(mListeners.begin(), mListeners.end(), &listener));
    }

protected:
private:
    std::deque<std::unique_ptr<Event>> mEvents{};

    // todo: separate vector for every type of event? useful if we have a lot of listeners
    std::vector<BaseEventListener*> mListeners{};
};

template <typename Tevent>
class EventListener : public BaseEventListener
{
public:
    EventListener(EventQueue& queue, std::function<void(Tevent&)> lambda)
        : mQueue(queue)
        , mLambda(lambda)
    {
        queue.AddListener(*this);
    }

    ~EventListener() override { mQueue.RemoveListener(*this); }

    EventListener(const EventListener&) = delete;
    EventListener& operator=(const EventListener&) = delete;
    EventListener(EventListener&&) = delete;
    EventListener& operator=(EventListener&&) = delete;

    void OnEvent(Event& event) override
    {
        if(auto* castEvent = dynamic_cast<Tevent*>(&event))
        {
            mLambda(*castEvent);
        }
    }

private:
    EventQueue& mQueue;
    std::function<void(Tevent&)> mLambda;
};

} // namespace memory
} // namespace vic