#include "gtest/gtest.h"

#include "vic/entity_system/ecs.h"

#include <optional>

using namespace vic;

using namespace vic::ecs;

struct A
{ };
struct B
{ };
struct C
{ };
struct D
{ };
struct E
{ };
struct F
{ };
struct G
{ };

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

    const std::size_t first = 1;
    for(std::size_t i = first; i < 10; ++i)
        components.Add(i, static_cast<uint64_t>(i));

    // create a const ref to the system, try to read from there
    const ComponentSystem& refComponents = components;
    EXPECT_EQ(refComponents.Get(1).val, 1u);

    std::optional<vic::ecs::EntityId> previous;
    for(const auto& comp : refComponents)
    {
        EXPECT_EQ(comp.first, comp.second.val);
        if(previous) // check increasing order
            EXPECT_TRUE(previous.value() < comp.first);

        previous = comp.first;
    }

    // test removing an item
    EXPECT_TRUE(components.Has(first));
    EXPECT_TRUE(components.Remove(first));
    EXPECT_FALSE(components.Has(first));
    EXPECT_FALSE(components.Remove(first)); // cannot remove an item that is already removed
    EXPECT_FALSE(components.Has(first));
}

TEST(TestEntitySystem, LowerBound)
{
    using ComponentSystem = vic::ecs::ComponentSystem<A>;
    ComponentSystem components{};

    for(std::size_t i = 1; i <= 10; ++i)
        if(i != 6)
            components.Add(i);

    auto it_5 = components.lower_bound(EntityId{5});
    EXPECT_EQ(it_5->first, EntityId{5});

    EXPECT_EQ(components.lower_bound(EntityId{6})->first, EntityId{7});

    EXPECT_EQ(components.lower_bound_with_hint(EntityId{5}, components.begin())->first, EntityId{5});
    EXPECT_EQ(components.lower_bound_with_hint(EntityId{5}, it_5)->first, EntityId{5});

    // find the first item with id 6 or higher (which is 7), start looking from 5
    EXPECT_EQ(components.lower_bound_with_hint(EntityId{6}, components.begin())->first, EntityId{7});
    EXPECT_EQ(components.lower_bound_with_hint(EntityId{6}, it_5)->first, EntityId{7});
}

TEST(TestEntitySystem, Iterate)
{
    using Ecs = ECS<A, B, C>;
    using Handle = Ecs::Handle;
    Ecs ecs;

    for(std::size_t i = 1; i < 100; ++i)
    {
        auto handle = ecs.NewEntity();
        const auto id = handle.Id();

        if((id % 2) == 0)
            handle.Add<A>();
        if((id % 3) == 0)
            handle.Add<B>();
        if((id % 4) == 0)
            handle.Add<C>();
    }

    const auto entities = std::vector<EntityId>{1, 2, 3, 4};

    const auto iterationA = ecs.Iterate<A>(entities.begin(), entities.end());

    EXPECT_EQ(iterationA.size(), 2u);
    for(const auto [entityId, aPtr] : iterationA)
        EXPECT_TRUE(ecs.Has<A>(entityId));

    const auto iterationB = ecs.Iterate<B>(entities.begin(), entities.end());
    EXPECT_EQ(iterationB.size(), 1u);
    for(const auto [entityId, bPtr] : iterationB)
        EXPECT_TRUE(ecs.Has<B>(entityId));

    const auto entitiesC = std::vector<EntityId>{4, 8, 12, 16, 20, 24, 28, 32};
    const auto iterationC = ecs.Iterate<C>(entitiesC.begin(), entitiesC.end());
    EXPECT_EQ(entitiesC.size(), iterationC.size());
    for(const auto [entityId, cPtr] : iterationC)
        EXPECT_TRUE(ecs.Has<C>(entityId));

    std::set<EntityId> entitySet{2, 4, 6};
    const auto setIteration = ecs.Iterate<A>(entitySet.begin(), entitySet.end());
    EXPECT_EQ(setIteration.size(), 3);
    for(const auto [entityId, aPtr] : setIteration)
        EXPECT_TRUE(ecs.Has<A>(entityId));
}

TEST(TestEntitySystem, Filter)
{
    using namespace vic::ecs::algorithms;

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

    for(const auto& [id, fizzPtr] : vic::ecs::algorithms::Filter<Fizz>(system))
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
    using namespace vic::ecs::algorithms;

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
    using ECS = vic::ecs::ECS<A, B, C, D>;

    ECS ecs;
    ecs.NewEntity().Add<A>();
    ecs.NewEntity().Add<A>().Add<B>();
    ecs.NewEntity().Add<A>().Add<B>().Add<C>();

    auto f1 = ecs.Filter<A>();
    EXPECT_EQ(f1.size(), 3u);
    auto f2 = ecs.Filter<A, B>();
    EXPECT_EQ(f2.size(), 2u);
    auto f3 = ecs.Filter<A, B, C>();
    EXPECT_EQ(f3.size(), 1u);
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
    struct Component
    {
        int val;
    };
    using System = vic::ecs::ECS<Component>;
    using Handle = System::Handle;

    System system;

    auto firstEnt = system.NewEntity();
    auto secondEnt = system.NewEntity();
    secondEnt.Add<Component>(5);

    if(auto nonexistantA = firstEnt.TryGet<Component>())
        ASSERT_TRUE(false); // should not be reachable

    if(auto existingA = secondEnt.TryGet<Component>())
        ASSERT_EQ(existingA->val, 5);
    else
        ASSERT_TRUE(false); // should not be reachable
}

TEST(TestEntitySystem, HasAny)
{
    struct ComponentA
    {
        int val;
    };
    struct ComponentB
    {
        int otherVal;
    };
    using System = vic::ecs::ECS<ComponentA, ComponentB>;
    System system;

    auto ent1 = system.NewEntity();
    ASSERT_FALSE(ent1.HasAny());
    ent1.Add<ComponentA>();
    ASSERT_TRUE(ent1.HasAny());
    ent1.Remove<ComponentA>();
    ASSERT_FALSE(ent1.HasAny());
    ent1.Add<ComponentB>();
    ASSERT_TRUE(ent1.HasAny());

    ASSERT_TRUE(System::ContainsComponent<ComponentA>());
    ASSERT_TRUE(System::ContainsComponent<ComponentB>());
    ASSERT_FALSE(System::ContainsComponent<int>());
    ASSERT_FALSE(System::ContainsComponent<double>());
}

TEST(TestEntitySystem, Relabel)
{
    const std::size_t count = 10;

    using Ecs = vic::ecs::ECS<A, B, C, D>;
    using Handle = Ecs::Handle;

    Ecs ecs;

    for(std::size_t i = 1; i <= count; ++i)
    {
        auto ent = ecs.NewEntity();
        if(i % 4 == 0)
            ent.Add<A>();

        if(i % 5 == 0)
            ent.Add<B>();

        if(i % 6 == 0)
            ent.Add<C>();

        if(i % 12 == 0)
            ent.Add<D>();
    }

    std::vector<vic::ecs::EntityId> existingIds;
    for(vic::ecs::EntityId i = 1; i <= count; ++i)
        if(ecs.HasAny(i))
            existingIds.push_back(i);

    const auto nItems = existingIds.size();

    EXPECT_EQ(ecs.ComponentSystem<B>::Minimum(), 5);
    EXPECT_EQ(ecs.ComponentSystem<B>::Maximum(), 10);

    EXPECT_EQ(ecs.Minimum(), 4u);
    EXPECT_EQ(ecs.Maximum(), 10u);

    ecs.Relabel();

    EXPECT_EQ(ecs.Minimum(), 1u);
    EXPECT_EQ(ecs.Maximum(), nItems);

    std::vector<vic::ecs::EntityId> newIds;
    for(vic::ecs::EntityId i = 1; i <= count; ++i)
        if(ecs.HasAny(i))
            newIds.push_back(i);

    EXPECT_EQ(existingIds.size(), newIds.size());

    for(vic::ecs::EntityId i = 1; i <= count; ++i)
        EXPECT_EQ(ecs.HasAny(i), i < nItems + 1) << "Entity " << i << (ecs.HasAny(i) ? " exists" : " does not exist");
}

TEST(TestEntitySystem, ChainAdds)
{
    struct ComponentA
    {
        int val;
    };
    struct ComponentB
    {
        int otherVal;
    };

    using MyEcs = vic::ecs::ECS<ComponentA, ComponentB>;
    MyEcs ecs;
    auto newEnt = ecs.NewEntity().Add<ComponentA>(1).Add<ComponentB>(2);

    EXPECT_TRUE(newEnt.Has<ComponentA>() && newEnt.Get<ComponentA>().val == 1);
    EXPECT_TRUE(newEnt.Has<ComponentB>() && newEnt.Get<ComponentB>().otherVal == 2);
}

TEST(TestEntitySystem, System)
{
    struct ComponentA
    {
        int val;
    };
    struct ComponentB
    {
        int otherVal;
    };
    using MyEcs = vic::ecs::ECS<ComponentA, ComponentB>;
    MyEcs ecs;

    // define a system that only requires a component A
    vic::ecs::System<MyEcs, ComponentA> system1(ecs);

    // define a system that requires both A and B
    vic::ecs::System<MyEcs, ComponentA, ComponentB> system2(ecs);

    // define a system that needs a const A, and a normal B
    vic::ecs::System<MyEcs, const ComponentA, ComponentB> system3(ecs);

    // failure:
    // vic::ecs::System<ECS, double> system_failure(ecs);
}

TEST(TestEntitySystem, CrossRef)
{
    // todo: make a CrossRef<TComponent, ...> type,
    // which is essentially just an EntityId,
    // but with component types assosiated to it.
    // in a debug build, Whenever the entity id is dereferenced,
    // it should be verified that the entity has the associated components
}

TEST(TestEntitySystem, Collection)
{
    struct Item
    {
        std::string description;
    };
    struct Backpack : public vic::ecs::Collection<Item>
    {
        using base = typename vic::ecs::Collection<Item>;
        using base::base;
    };

    static_assert(!vic::ecs::ConceptCollection<Item>);
    static_assert(vic::ecs::ConceptCollection<Backpack>);

    using MyEcs = vic::ecs::ECS<Item, Backpack>;
    MyEcs ecs;

    auto sword = ecs.NewEntity();
    sword.Add<Item>("sword");

    auto shield = ecs.NewEntity();
    shield.Add<Item>("shield");

    auto backpack = ecs.NewEntity();
    backpack.Add<Backpack>(sword, shield);

    EXPECT_TRUE(backpack.Get<Backpack>().Verify(ecs));

    // add an entity without the Item component, check that Verify fails
    auto rain = ecs.NewEntity();
    backpack.Get<Backpack>().Insert(rain);
    EXPECT_FALSE(backpack.Get<Backpack>().Verify(ecs));

    // remove the item, check that it succeeds again
    backpack.Get<Backpack>().Remove(rain);
    EXPECT_TRUE(backpack.Get<Backpack>().Verify(ecs));

    // todo: fast iteration over the sorted items in the collection
    std::set<vic::ecs::EntityId> ids;
    for(const auto& itemId : backpack.Get<Backpack>())
        ids.insert(itemId);
    const std::set<vic::ecs::EntityId> answer{sword.Id(), shield.Id()};
    EXPECT_EQ(ids, answer);

    // copy backpack into a vector of entity ids.
    const auto items = std::vector{backpack.Get<Backpack>().begin(), backpack.Get<Backpack>().end()};
    EXPECT_EQ(items.size(), 2u);
}

TEST(TestEntitySystem, ParallelSystems)
{
    using ECS = vic::ecs::ECS<A, B, C, D>;
    ECS ecs;

    using SystemConstA = vic::ecs::System<ECS, const A>;

    EXPECT_TRUE(SystemConstA::ContainsComponent<const A>());
    EXPECT_FALSE(SystemConstA::ContainsComponent<A>());

    using SystemNonConstA = vic::ecs::System<ECS, A>;

    EXPECT_FALSE(SystemNonConstA::ContainsComponent<const A>());
    EXPECT_TRUE(SystemNonConstA::ContainsComponent<A>());

    EXPECT_TRUE(SystemNonConstA::IsBlockedBy<SystemNonConstA>());
    EXPECT_TRUE(SystemConstA::IsBlockedBy<SystemNonConstA>());
    EXPECT_TRUE(SystemNonConstA::IsBlockedBy<SystemConstA>());
    EXPECT_FALSE(SystemConstA::IsBlockedBy<SystemConstA>());

    using System_cAB = vic::ecs::System<ECS, const A, B>;

    EXPECT_FALSE(System_cAB::IsBlockedBy<SystemConstA>());

    using System_AB = vic::ecs::System<ECS, A, B>;

    EXPECT_TRUE(System_AB::IsBlockedBy<SystemConstA>());
}

TEST(TestEntitySystem, ExecuteSystems)
{
    using ECS = vic::ecs::ECS<A, B, C, D, E, F, G>;
    ECS ecs;

    using System1 = vic::ecs::System<ECS, A, B, C, D, E, F, G>;
    System1 sys1{ecs};

    // system1 blocks all components

    using System2 = vic::ecs::System<ECS, const A, C>;
    System2 sys2{ecs};

    using System3 = vic::ecs::System<ECS, const B, D>;
    System3 sys3{ecs};

    // 2 and 3 can run simultaneously

    using System4 = vic::ecs::System<ECS, const C, const D, E>;
    System4 sys4{ecs};

    // 4 depends on 2 and 3 (and 1)

    using System5 = vic::ecs::System<ECS, const A, const B, const C, const D, const E, F, G>;
    System5 sys5{ecs};

    vic::ecs::SystemExecutor executor{&sys1, &sys2, &sys3, &sys4, &sys5};

    executor.Run();
}

TEST(TestEntitySystem, ForeachComponentType)
{
    using ECS = vic::ecs::ECS<A, B, C, D, E, F, G>;
    ECS ecs;

    std::size_t componentTypeCount = 0;

    auto lambda = [&]<typename T>() {
        componentTypeCount++;
        std::cout << typeid(T).name() << ": size = " << ecs.ComponentSystem<T>::Size() << std::endl;
    };

    ecs.ForeachComponentType(lambda);

    EXPECT_EQ(componentTypeCount, 7);
}