
#include "pch.h"

#include "vic/EventQueue.h"

using namespace vic;

struct MyEvent : public Event
{
	std::string mText{ "My Event" };
};

struct MyOtherEvent : public Event
{
	std::string mOtherText{ "My Other Event" };
};

TEST(TestEventQueue, Startup)
{
	vic::PolymorphicEventQueue queue;

	vic::SpecializedListener<MyEvent> listener(queue,
		[](MyEvent& event)
		{
			std::cout << "tadaaa: " << event.mText << std::endl;
		}
	);

	vic::SpecializedListener<MyOtherEvent> otherListener(queue,
		[](MyOtherEvent& event) { std::cout << "tadaaa: " << event.mOtherText << std::endl; }
	);

	queue.EmplaceBack<MyEvent>();
	queue.EmplaceBack<MyOtherEvent>();

	while (queue.Next())
	{
		// loop through all events
	}

	// ASSERT_TRUE(false);

}

