#include "pch.h"

#include "vic/entity_system/entity_system.h"

#include "vic/entity_system/algorithms.h"

#include <optional>

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

TEST(TestEntitySystem, ComponentSystem)
{
    struct TestType
    {
        uint64_t val;
    };
    using ComponentSystem = vic::entity::ComponentSystem<TestType>;
    ComponentSystem components{};

    for(std::size_t i = 0; i < 10; ++i)
        components.Add(i, TestType{static_cast<uint64_t>(i)});

    // create a const ref to the system, try to read from there
    const ComponentSystem& refComponents = components;
    EXPECT_EQ(refComponents.Get(1).val, 1u);

    std::optional<vic::entity::EntityId> previous;
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

    // iterate 1d
    for(auto it = system.begin<Fizz>(); it != system.end<Fizz>(); ++it)
    {
        Name& name = system.Get<Name>(it->first);
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

    //// todo: iterate 3d;
    //for(auto [entId, namePtr, fizzPtr, buzzPtr] : Filter<Name, Fizz, Buzz>(system))
    //{
    //    //
    //}

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

    using System = vic::entity::EntitySystem<A, B, C, D>;
    using Handle = System::Handle;

    System system;
    Handle handle = system.NewEntity();
}