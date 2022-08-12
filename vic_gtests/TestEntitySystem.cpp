#include "pch.h"

#include "vic/entity_system/entity_system.h"

#include "vic/entity_system/algorithms.h"

using namespace vic;

TEST(TestEntitySystem, Startup)
{
    // create a system, add an entity, add components to that entity, retrieve them
    using System = vic::entity::EntitySystem<int, float, std::string>;
    using Handle = System::Handle;

    System system;

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
    struct Name
    {
        std::string mName{};
        int mI{};
    };
    struct Fizz
    { };
    struct Buzz
    { };

    using System = vic::entity::EntitySystem<Name, Fizz, Buzz>;
    using Handle = System::Handle;

    System system;

    for(std::size_t i = 0; i < 100; ++i)
    {
        auto ent = system.NewEntity();

        ent.Add<Name>({std::to_string(i), int(i)});

        if(i % 3 == 0)
            ent.Add<Fizz>({});

        if(i % 5 == 0)
            ent.Add<Buzz>({});
    }

    // now filter out all objects with both the Fizz and Buzz component
    int sum = 0;
    for(auto [entId, fizzPtr, buzzPtr] : Filter<Fizz, Buzz>(system))
    {
        const int& index = system.Get<Name>(entId).mI;
        EXPECT_TRUE((index % 3 == 0));
        EXPECT_TRUE((index % 5 == 0));
        sum++;
    }
    EXPECT_EQ(sum, 7);
}

TEST(TestEntitySystem, Remove)
{

    struct A
    { };
    struct B
    { };
    struct C
    { };
    struct D
    { };

    using System = vic::entity::EntitySystem<A, B, C, D>;
    using Handle = System::Handle;

    System system;
    Handle handle = system.NewEntity();

    EXPECT_FALSE(handle.Has<A>() || handle.Has<B>() || handle.Has<C>() || handle.Has<D>());

    handle.Add<A>({});
    handle.Add<B>({});
    handle.Add<C>({});
    handle.Add<D>({});

    EXPECT_TRUE(handle.Has<A>() && handle.Has<B>() && handle.Has<C>() && handle.Has<D>());

    handle.Remove<A>();

    EXPECT_TRUE(handle.Has<B>() && handle.Has<C>() && handle.Has<D>());
    EXPECT_FALSE(handle.Has<A>());

    system.RemoveEntity(handle);

    // todo: invalidate handle?
    EXPECT_FALSE(handle.Has<A>() || handle.Has<B>() || handle.Has<C>() || handle.Has<D>());
}