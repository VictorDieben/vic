
#include "pch.h"

#include "vic/memory/event_queue.h"

using namespace vic::memory;

struct MyEvent : public Event
{
    std::string mText{"My Event"};
};

struct MyOtherEvent : public Event
{
    std::string mOtherText{"My Other Event"};
};

TEST(TestEventQueue, Startup)
{
    PolymorphicEventQueue queue;

    SpecializedListener<MyEvent> listener(queue, [](MyEvent& event) { std::cout << "tadaaa: " << event.mText << std::endl; });

    SpecializedListener<MyOtherEvent> otherListener(queue, [](MyOtherEvent& event) { std::cout << "tadaaa: " << event.mOtherText << std::endl; });

    queue.EmplaceBack<MyEvent>();
    queue.EmplaceBack<MyOtherEvent>();

    while(queue.Next())
    {
        // loop through all events
    }
}
