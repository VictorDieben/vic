#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <typeindex>
#include <typeinfo>
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
        // mEvents.emplace_back(new T(std::forward<Targs>...));
        mEvents.emplace_back(std::make_unique<T>(std::forward<Targs>...));
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
    // todo: ring buffer?
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

//
//
//

using EventTypeId = std::type_index;

struct BaseEvent
{ };

class NewBaseEventHandler;

class NewEventQueue
{
public:
    NewEventQueue() = default;
    ~NewEventQueue();

    using EventPair = std::pair<EventTypeId, std::unique_ptr<BaseEvent>>;

    template <typename TEvent, typename... TArgs>
    void EmplaceEvent(TArgs&&... args)
    {
        static_assert(std::is_base_of_v<BaseEvent, TEvent>);
        auto eventPair = EventPair(EventTypeId(TEvent), //
                                   std::make_unique<TEvent>(std::forward<TArgs>...));
        {
            std::unique_lock eventLock(mEventMutex);
            mEvents.push_back(std::move(eventPair));
        }
        mCv.notify_all();
    }

    void Run(const std::atomic<bool>& keepRunning);

protected:
    // called from base event handler
    friend class NewBaseEventHandler;
    void Subscribe(NewBaseEventHandler& handler, const EventTypeId eventTypeId)
    {
        std::unique_lock eventLock(mHandlerMutex);
        mHandlerMap[eventTypeId].push_back(&handler); //
    }
    void Unsubscribe(NewBaseEventHandler& handler, const EventTypeId eventTypeId)
    {
        std::unique_lock eventLock(mHandlerMutex);
        auto& vec = mHandlerMap.at(eventTypeId);
        std::erase(vec, &handler);
    }

private:
    using HandlerMutex = std::mutex; // UniqueType<std::mutex>;
    HandlerMutex mHandlerMutex;
    std::map<EventTypeId, std::vector<NewBaseEventHandler*>> mHandlerMap;

    using EventMutex = std::mutex; // UniqueType<std::mutex>;
    EventMutex mEventMutex;
    std::deque<EventPair> mEvents{};

    // condition variable for adding new events
    std::condition_variable mCv;
};

class NewBaseEventHandler
{
public:
    NewBaseEventHandler(NewEventQueue& queue, const EventTypeId eventTypeId)
        : mQueue(&queue)
        , mEventTypeId(eventTypeId)
    {
        mQueue->Subscribe(*this, mEventTypeId);
    }
    ~NewBaseEventHandler()
    {
        if(mQueue)
            mQueue->Unsubscribe(*this, mEventTypeId);
    }

    virtual void OnEvent(const BaseEvent& event) noexcept = 0;

protected:
    friend class NewEventQueue;
    void StartListening(NewEventQueue& queue) { mQueue = &queue; }
    void StopListening() { mQueue = nullptr; }

private:
    NewEventQueue* mQueue{nullptr};
    const EventTypeId mEventTypeId;
};

inline NewEventQueue::~NewEventQueue()
{
    // stop any listeners that were still active.
    // should be last resort, eventqueue should not outlive listeners
    std::unique_lock eventLock(mHandlerMutex);
    for(auto& [_, handlers] : mHandlerMap)
        for(auto& handler : handlers)
            handler->StopListening();
}

inline void NewEventQueue::Run(const std::atomic<bool>& keepRunning)
{
    while(keepRunning)
    {
        std::unique_lock eventLock(mEventMutex);
        if(!mCv.wait_for(eventLock,
                         std::chrono::milliseconds(100), //
                         [&]() { return !mEvents.empty(); }))
            continue;

        const auto eventPair = std::move(mEvents.front());
        mEvents.pop_front();
        eventLock.unlock();

        const auto typeId = eventPair.first;

        std::unique_lock handlerLock(mHandlerMutex);
        for(auto& handler : mHandlerMap[typeId])
            handler->OnEvent(*eventPair.second.get());
    }
}

} // namespace memory
} // namespace vic