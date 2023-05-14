#include "pch.h"

#include "vic/entity_system/entity_system.h"

#include "vic/entity_system/algorithms.h"

#include <optional>

using namespace vic;

TEST(TestEntitySystem, Startup)
{
    // create a system, add an entity, add components to that entity, retrieve them
    using System = vic::ecs::ECS<int, float, std::string>;
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

TEST(TestEntitySystem, ComponentSystem)
{
    struct TestType
    {
        uint64_t val;
    };
    using ComponentSystem = vic::ecs::ComponentSystem<TestType>;
    ComponentSystem components{};

    for(std::size_t i = 0; i < 10; ++i)
        components.Add(i, static_cast<uint64_t>(i));

    // create a const ref to the system, try to read from there
    const ComponentSystem& refComponents = components;
    EXPECT_EQ(refComponents.Get(1).val, 1u);

    std::optional<vic::ecs::EntityId> previous;
    for(const auto& comp : refComponents)
    {
        EXPECT_EQ(comp.first, comp.second.val);
        if(previous) // check increasing order
        {
            EXPECT_TRUE(previous.value() < comp.first);
            previous = comp.first;
        }
    }

    // test removing an item
    EXPECT_TRUE(components.Has(0));
    EXPECT_TRUE(components.Remove(0));
    EXPECT_FALSE(components.Has(0));
    EXPECT_FALSE(components.Remove(0)); // cannot remove an item that is already removed
    EXPECT_FALSE(components.Has(0));
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
    {
        std::string buzzName{};
    };

    using System = vic::ecs::ECS<Name, Fizz, Buzz>;
    using Handle = System::Handle;

    System system;

    for(std::size_t i = 0; i < 100; ++i)
    {
        auto ent = system.NewEntity();

        ent.Add<Name>(std::to_string(i), int(i));

        if(i % 3 == 0)
            ent.Add<Fizz>();

        if(i % 5 == 0)
            ent.Add<Buzz>();
    }

    // iterate 1d
    for(auto it = system.begin<Fizz>(); it != system.end<Fizz>(); ++it)
    {
        Name& name = system.Get<Name>(it->first);
        EXPECT_TRUE(name.mI % 3 == 0);
    }

    for(const auto& [id, fizzPtr] : Filter<Fizz>(system))
    {
        Name& name = system.Get<Name>(id);
        EXPECT_TRUE(name.mI % 3 == 0);
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

    // todo: filter all objects with a Fizz _or_ Buzz component

    // iterate 3d;
    for(auto [entId, namePtr, fizzPtr, buzzPtr] : Filter<Name, Fizz, Buzz>(system))
    {
        // no need to do anything, this should just compile
    }

    // todo: const iteration
    const System& refSystem = system;

    for(const auto& [entId, namePtr, buzzPtr] : Filter<const Name, const Buzz>(refSystem))
    {
        // no need to do anything, this should just compile
    }

    for(auto& [entId, namePtr, buzzPtr] : Filter<const Name, Buzz>(system))
    {
        // namePtr->mName = "bla"; // <-- this should not compile! assigning to const ptr
        buzzPtr->buzzName = namePtr->mName;
    }
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

    using System = vic::ecs::ECS<A, B, C, D>;
    using Handle = System::Handle;

    System system;
    Handle handle = system.NewEntity();
    EXPECT_FALSE(handle.HasAny());

    EXPECT_FALSE(handle.Has<A>() || handle.Has<B>() || handle.Has<C>() || handle.Has<D>());

    handle.Add<A>();

    EXPECT_TRUE(handle.HasAny());

    handle.Add<B>();
    handle.Add<C>();
    handle.Add<D>();

    EXPECT_TRUE(handle.Has<A>() && handle.Has<B>() && handle.Has<C>() && handle.Has<D>());
    EXPECT_TRUE(handle.HasAny());

    handle.Remove<A>();

    EXPECT_TRUE(handle.Has<B>() && handle.Has<C>() && handle.Has<D>());
    EXPECT_FALSE(handle.Has<A>());
    EXPECT_TRUE(handle.HasAny());

    system.RemoveEntity(handle);

    // todo: invalidate handle?
    EXPECT_FALSE(handle.Has<A>() || handle.Has<B>() || handle.Has<C>() || handle.Has<D>());
    EXPECT_FALSE(handle.HasAny());
}

TEST(TestEntitySystem, 3d)
{
    struct A
    { };
    struct B
    { };
    struct C
    { };
    struct D
    { };

    using System = vic::ecs::ECS<A, B, C, D>;
    using Handle = System::Handle;

    System system;
    Handle handle = system.NewEntity();
}

TEST(TestEntitySystem, Insert)
{
    struct A
    {
        int val;
    };
    struct B
    {
        double val;
    };

    using System = vic::ecs::ECS<A, B>;
    using Handle = System::Handle;

    // create some data that we can push into the component system at once.
    std::vector<std::pair<const vic::ecs::EntityId, A>> data;
    data.push_back({0, {0}});
    data.push_back({1, {1}});

    System system;
    system.ComponentSystem<A>::Insert(data.begin(), data.end());

    ASSERT_EQ(system.Size<A>(), 2u);
}

TEST(TestEntitySystem, TryGet)
{
    struct A
    {
        int val;
    };
    using System = vic::ecs::ECS<A>;
    using Handle = System::Handle;

    System system;

    auto firstEnt = system.NewEntity();
    auto secondEnt = system.NewEntity();
    secondEnt.Add<A>(5);

    if(auto nonexistantA = firstEnt.TryGet<A>())
        ASSERT_TRUE(false); // should not be reachable

    if(auto existingA = secondEnt.TryGet<A>())
        ASSERT_EQ(existingA->val, 5);
    else
        ASSERT_TRUE(false); // should not be reachable
}

TEST(TestEntitySystem, HasAny)
{
    struct A
    {
        int val;
    };
    struct B
    {
        int otherVal;
    };
    using System = vic::ecs::ECS<A, B>;
    System system;

    auto ent1 = system.NewEntity();
    ASSERT_FALSE(ent1.HasAny());
    ent1.Add<A>();
    ASSERT_TRUE(ent1.HasAny());
    ent1.Remove<A>();
    ASSERT_FALSE(ent1.HasAny());
    ent1.Add<B>();
    ASSERT_TRUE(ent1.HasAny());

    ASSERT_TRUE(System::ContainsComponent<A>());
    ASSERT_TRUE(System::ContainsComponent<B>());
    ASSERT_FALSE(System::ContainsComponent<int>());
    ASSERT_FALSE(System::ContainsComponent<double>());
}

TEST(TestEntitySystem, Relabel)
{
    //
}

TEST(TestEntitySystem, ChainAdds)
{
    struct A
    {
        int val;
    };
    struct B
    {
        int otherVal;
    };

    using MyEcs = vic::ecs::ECS<A, B>;
    MyEcs ecs;
    auto newEnt = ecs.NewEntity();

    newEnt //
        .Add<A>(1)
        .Add<B>(2);

    EXPECT_TRUE(newEnt.Has<A>() && newEnt.Get<A>().val == 1);
    EXPECT_TRUE(newEnt.Has<B>() && newEnt.Get<B>().otherVal == 2);
}

TEST(TestEntitySystem, System)
{
    struct A
    {
        int val;
    };
    struct B
    {
        int otherVal;
    };
    using MyEcs = vic::ecs::ECS<A, B>;
    MyEcs ecs;

    // define a system that only requires a component A
    vic::ecs::System<MyEcs, A> system1(ecs);

    // define a system that requires both A and B
    vic::ecs::System<MyEcs, A, B> system2(ecs);

    // define a system that needs a const A, and a normal B
    vic::ecs::System<MyEcs, const A, B> system3(ecs);

    // failure:
    // vic::ecs::System<ECS, double> system_failure(ecs);
}