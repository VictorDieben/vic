
#include "gtest/gtest.h"

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
    EventQueue queue;

    int val = 0;

    EventListener<MyEvent> listener(queue, //
                                    [&](const MyEvent& event) {
                                        std::cout << "tadaaa: " << event.mText << std::endl; //
                                        EXPECT_EQ(val, 0);
                                        val++;
                                    });

    EventListener<MyOtherEvent> otherListener(queue, //
                                              [&](const MyOtherEvent& event) {
                                                  std::cout << "tadaaa: " << event.mOtherText << std::endl; //
                                                  EXPECT_EQ(val, 1);
                                                  val++;
                                              });

    queue.EmplaceBack<MyEvent>();
    queue.EmplaceBack<MyOtherEvent>();

    while(queue.Next())
    {
        // loop through all events
    }

    EXPECT_EQ(val, 2);
}