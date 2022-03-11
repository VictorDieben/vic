#include "pch.h"

#include "vic/EntitySystem.h"

#include <string>

using namespace vic;

TEST(TestEntitySystem, Startup)
{
	// create a system, add an entity, add components to that entity, retrieve them
	vic::EntitySystem<int, float, std::string> system;

	auto ent = system.NewEntity();

	ASSERT_FALSE(system.Has<int>(ent));
	system.Add<int>(ent, 1);
	ASSERT_TRUE(system.Has<int>(ent));

	// create another entity, add different component
	auto ent2 = system.NewEntity();
	ASSERT_FALSE(system.Has<int>(ent2));
	ASSERT_FALSE(system.Has<float>(ent2));
	system.Add<float>(ent2, 2.0f);
	ASSERT_FALSE(system.Has<int>(ent2));
	ASSERT_TRUE(system.Has<float>(ent2));
	EXPECT_DOUBLE_EQ(system.Get<float>(ent2), 2.0f);

	// access the components through the entity itself
	ASSERT_FALSE(ent.Has<std::string>());
	ent.Add<std::string>("bla");
	ASSERT_TRUE(ent.Has<std::string>());
	auto& textComp = ent.Get<std::string>();
	ASSERT_EQ(textComp, "bla");
	ent.Remove<std::string>();
	ASSERT_FALSE(ent.Has<std::string>());
}

TEST(TestEntitySystem, Filter)
{
	struct Name { std::string mName{}; int mI{}; };
	struct Fizz {};
	struct Buzz {};

	EntitySystem<Name, Fizz, Buzz> system;

	for (std::size_t i = 0; i < 100; ++i)
	{
		auto ent = system.NewEntity();

		ent.Add<Name>({ std::to_string(i), int(i) });

		if (i % 3 == 0)
			ent.Add<Fizz>({});

		if (i % 5 == 0)
			ent.Add<Buzz>({});
	}

	// now filter out all objects with both the Fizz and Buzz component
	int sum = 0;
	for (const auto& entId : system.Filter<Fizz, Buzz>())
	{
		const int& index = system.Get<Name>(entId).mI;
		EXPECT_TRUE((index % 3 == 0));
		EXPECT_TRUE((index % 5 == 0));
		sum++;
	}
	EXPECT_EQ(sum, 7);
}