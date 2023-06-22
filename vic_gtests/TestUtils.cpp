
#include "gtest/gtest.h"

#include "test_base.h"
#include "vic/utils.h"
#include "vic/utils/counted.h"
#include "vic/utils/math.h"
#include "vic/utils/observable.h"
#include "vic/utils/ranges.h"
#include "vic/utils/serialize.h"
#include "vic/utils/statemachine.h"
#include "vic/utils/timing.h"
#include "vic/utils/unique.h"

#include "vic/memory/constexpr_map.h"

#include <random>
#include <thread>

using namespace vic;

TEST(TestUtils, TestPow)
{
    constexpr auto res = Pow<2>(3); // check constexpr

    EXPECT_EQ(Pow<0>(3), 1);
    EXPECT_EQ(Pow<1>(3), 3);
    EXPECT_EQ(Pow<2>(3), 9);

    EXPECT_EQ(Pow<0>(10), 1);
    EXPECT_EQ(Pow<1>(10), 10);
    EXPECT_EQ(Pow<2>(10), 100);
    EXPECT_EQ(Pow<3>(10), 1000);

    const double tol = 1E-14;
    const double pi = 3.1415;
    EXPECT_NEAR(Pow<0>(pi), 1., tol);
    EXPECT_NEAR(Pow<3>(pi), pi * pi * pi, tol);
    EXPECT_NEAR(Pow<6>(pi), pi * pi * pi * pi * pi * pi, tol);
}

TEST(TestUtils, TestTimer)
{
    CTimer timer;
    timer.Reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const double interval_ms = timer.GetTime<double, std::milli>().count();
    EXPECT_LT(9.99, interval_ms);
    EXPECT_LT(interval_ms, 50.); // large upper bound, we don't know when sleep returns
}

TEST(TestUtils, TestFinally)
{
    bool value = false;
    {
        Finally fin{[&]() { value = true; }};
    }
    EXPECT_TRUE(value);
}

TEST(TestUtils, TestCounted)
{
    struct S : public Counted<S>
    { };

    {
        // construct / destruct
        EXPECT_EQ(S::GetCount(), 0);
        {
            S s1{};
            EXPECT_EQ(S::GetCount(), 1);
        }
        EXPECT_EQ(S::GetCount(), 0);
        S s2{};
        S s3 = s2; // copy
        EXPECT_EQ(S::GetCount(), 2);
        S s4 = std::move(s2);
        EXPECT_EQ(S::GetCount(), 3);
    }
    EXPECT_EQ(S::GetCount(), 0);
}

TEST(TestUtils, TestToBase)
{
    std::vector<int> buffer{};
    ToBase<int>(8, 2, buffer);
    ExpectVectorsEqual(buffer, std::vector<int>{0, 0, 0, 1});
}

TEST(TestUtils, TestFromBase)
{
    // base 2
    const uint16_t baseTwo{2};
    EXPECT_EQ(0, FromBase<uint64_t>(std::vector<int>{}, baseTwo));
    EXPECT_EQ(0, FromBase<uint64_t>(std::vector<int>{0}, baseTwo));
    EXPECT_EQ(1, FromBase<uint64_t>(std::vector<int>{1}, baseTwo));
    EXPECT_EQ(2, FromBase<uint64_t>(std::vector<int>{0, 1}, baseTwo));
    EXPECT_EQ(4, FromBase<uint64_t>(std::vector<int>{0, 0, 1}, baseTwo));
    EXPECT_EQ(8, FromBase<uint64_t>(std::vector<int>{0, 0, 0, 1}, baseTwo));
    EXPECT_EQ(8, FromBase<uint64_t>(std::vector<int>{0, 0, 0, 1, 0, 0, 0}, baseTwo));

    // make sure we trigger an assert if we try to convert a number that is too big.
    // (e.g. the value 10 in base 10 is no longer 1 digit)
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{2}, 2), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{0, 3}, 3), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{4, 0}, 4), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{0, 5, 0}, 5), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{-1}, 2), "");
}

TEST(TestUtils, TestToFromBase)
{
    // pick a random value and base. convert value to that other base.
    // Then convert it back to normal representation
    using ValueType = uint64_t;
    using Basetype = uint16_t;

    std::default_random_engine generator;
    std::uniform_int_distribution<ValueType> valueDist(0, std::numeric_limits<ValueType>::max());
    std::uniform_int_distribution<Basetype> baseDist(2, std::numeric_limits<Basetype>::max());

    std::vector<Basetype> buffer{}; // used to write result to/from

    for(std::size_t i = 0; i < 10000; ++i)
    {
        const ValueType value = valueDist(generator);
        const Basetype base = baseDist(generator);

        ToBase(value, base, buffer);
        const auto fromBase = FromBase<ValueType>(buffer, base);
        EXPECT_EQ(value, fromBase);
    }
}

TEST(TestUtils, Linspace)
{
    const auto vec0 = Linspace(0., 1., 0);
    EXPECT_EQ(vec0.size(), 0);

    const auto vec1 = Linspace(0., 1., 1);
    EXPECT_EQ(vec1.size(), 1);
    EXPECT_DOUBLE_EQ(vec1.at(0), 0.);

    const auto vec2 = Linspace(0., 1., 2);
    EXPECT_EQ(vec2.size(), 2);
    EXPECT_DOUBLE_EQ(vec2.at(0), 0.);
    EXPECT_DOUBLE_EQ(vec2.at(1), 1.);

    const auto vec3 = Linspace(0., 1., 3);
    EXPECT_EQ(vec3.size(), 3);
    EXPECT_DOUBLE_EQ(vec3.at(0), 0.);
    EXPECT_DOUBLE_EQ(vec3.at(1), .5);
    EXPECT_DOUBLE_EQ(vec3.at(2), 1.);
}

TEST(TestUtils, TestRange)
{
    // test range
    std::vector<int> result;
    for(const auto& val : Range(10))
        result.push_back(val);
    std::vector<int> answer{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    ASSERT_EQ(result, answer);

    // test subrange
    std::vector<int> result2;
    for(const auto& val : Range(2, 5)) //
        result2.push_back(val);
    std::vector<int> answer2{2, 3, 4};
    ASSERT_EQ(result2, answer2);

    // test different stride
    std::vector<int> result3;
    for(const auto& val : Range(0, 10, 4)) //
        result3.push_back(val);
    std::vector<int> answer3{0, 4, 8};
    ASSERT_EQ(result3, answer3);

    // test backwards stride
    std::vector<int> result4;
    for(const auto& val : Range(10, 0, -2)) //
        result4.push_back(val);
    std::vector<int> answer4{10, 8, 6, 4, 2};
    ASSERT_EQ(result4, answer4);

    // check range in wrong direction
    std::vector<int> result5;
    for(const auto& val : Range(0, 4, -1)) //
        result5.push_back(val);
    std::vector<int> answer5{};
    ASSERT_EQ(result5, answer5);

    std::vector<int> result6;
    for(const auto& val : Range(4, 0, 1)) //
        result6.push_back(val);
    std::vector<int> answer6{};
    ASSERT_EQ(result6, answer6);
}

//TEST(TestUtils, TestZip)
//{
//    std::vector<int> a{0, 1, 2, 3};
//    std::vector<char> b{'a', 'b', 'c', 'd'};
//    std::vector<std::pair<int, char>> result;
//    for(const auto& pair : Zip(a, b))
//        result.push_back(pair); //
//
//    std::vector<std::pair<int, char>> answer{{0, 'a'}, {1, 'b'}, {2, 'c'}, {3, 'd'}};
//    ASSERT_EQ(result, answer);
//}

TEST(TestUtils, TestStateMachine)
{
    enum class State
    {
        One,
        Two,
        Three,
        Four,
        Five
    };

    static constexpr std::array<std::pair<State, State>, 8> validChanges //
        {std::pair{State::One, State::Two}, //
         std::pair{State::One, State::Four}, //
         std::pair{State::Two, State::Three}, //
         std::pair{State::Two, State::Four}, //
         std::pair{State::Three, State::Four}, //
         std::pair{State::Three, State::Five}, //
         std::pair{State::Four, State::Five}, //
         std::pair{State::Five, State::One}};

    //// TODO: change template definition such that this simply becomes:
    //// StateMachine<validChanges> stateMachine{State::Two};
    //StateMachine<State, validChanges.size(), validChanges> stateMachine(State::Two);

    //EXPECT_TRUE(stateMachine.HasState<State::Two>());
    //EXPECT_EQ(stateMachine.GetState(), State::Two);

    //stateMachine.SetState<State::Two, State::Four>();
    //EXPECT_TRUE(stateMachine.HasState<State::Four>());
    //// EXPECT_DEATH((stateMachine.SetState<State::Two, State::Four>()), ""); // valid change, but wrong fromState
    //// EXPECT_DEATH((stateMachine.SetState<State::Four, State::One>()), ""); // invalid change, correct fromState

    //// check at compile time that some state changes are valid or not
    //static_assert(stateMachine.IsValid<State::One, State::Two>());
    //static_assert(!stateMachine.IsValid<State::One, State::Five>());

    //// TODO: some kind of wrapper for calling a Tick function per current state.
    //// This way, we can even avoid checking the current state.
    //// and maybe even predict deadlock states
    ////stateMachine.Tick(); // calls: stateMachine.Tick<State::Two>();
}

TEST(TestUtils, TestNewStateMachine)
{
    enum class StateEnum
    {
        One,
        Two,
        Three,
        Four,
        Five,
        Unused // test that not every enum is automatically valid
    };

    using State1 = State<StateEnum::One>;
    using State2 = State<StateEnum::Two>;
    using State3 = State<StateEnum::Three>;
    using State4 = State<StateEnum::Four>;
    using State5 = State<StateEnum::Five>;
    using StateUnused = State<StateEnum::Unused>;

    using StateList = StateContainer<State1, State2, State3, State4, State5>;
    static_assert(StateList::Contains<State1>());
    static_assert(!StateList::Contains<StateUnused>());

    using Transition12 = Transition<State1, State2>;
    using Transition23 = Transition<State2, State3>;
    using Transition34 = Transition<State3, State4>;
    using Transition45 = Transition<State4, State5>;
    using Transition51 = Transition<State5, State1>;

    using TransitionUnused = Transition<State1, StateUnused>;

    using TransitionList = TransitionContainer<Transition12, Transition23, Transition34, Transition45, Transition51>;
    static_assert(TransitionList::Contains<Transition12>());
    static_assert(!TransitionList::Contains<TransitionUnused>());

    using Description = StateMachineDescription<StateList, TransitionList>;

    NewStateMachine<Description> newStateMachine;
    (void)newStateMachine;
}

TEST(TestUtils, TestObservable)
{
    struct TestObservable : public Observable<TestObservable>
    { };

    TestObservable object{};

    ASSERT_FALSE(object.IsObserved());
    {
        const auto handle = object.Observe();

        ASSERT_TRUE(object.IsObserved());

        {
            const auto handle2 = object.Observe();
            ASSERT_TRUE(object.IsObserved());
        }

        ASSERT_TRUE(object.IsObserved());
    }
    ASSERT_FALSE(object.IsObserved());
}

TEST(TestUtils, TestIsPowerOfTwo)
{
    // check constexpr
    static_assert(IsPowerOfTwo(int8_t{2}));
    static_assert(IsPowerOfTwo(int16_t{2}));
    static_assert(IsPowerOfTwo(int32_t{2}));
    static_assert(IsPowerOfTwo(int64_t{2}));
    static_assert(IsPowerOfTwo(uint8_t{2}));
    static_assert(IsPowerOfTwo(uint16_t{2}));
    static_assert(IsPowerOfTwo(uint32_t{2}));
    static_assert(IsPowerOfTwo(uint64_t{2}));

    static_assert(!IsPowerOfTwo(int8_t{3}));
    static_assert(!IsPowerOfTwo(int16_t{3}));
    static_assert(!IsPowerOfTwo(int32_t{3}));
    static_assert(!IsPowerOfTwo(int64_t{3}));
    static_assert(!IsPowerOfTwo(uint8_t{3}));
    static_assert(!IsPowerOfTwo(uint16_t{3}));
    static_assert(!IsPowerOfTwo(uint32_t{3}));
    static_assert(!IsPowerOfTwo(uint64_t{3}));

    // check that negative numbers are not powers of two
    for(int32_t i = 0; i < 100; ++i)
        ASSERT_FALSE(IsPowerOfTwo(-i));

    // check all powers of two in the range of 2^[2; 30]
    for(uint32_t i = 2; i < 31; ++i)
        ASSERT_TRUE(IsPowerOfTwo(Power<int32_t>(2, i))) << i;

    for(uint32_t i = 2; i < 31; ++i)
        ASSERT_FALSE(IsPowerOfTwo(Power<int32_t>(2, i) - 1)) << i;
}

TEST(TestUtils, Exp)
{
    std::default_random_engine rng;
    std::uniform_real_distribution dist(0.0001, 100.);

    for(int32_t i = 0; i < 1000000; ++i)
    {
        const double x = dist(rng);
        const double answer = std::exp(x);
        const double eps = answer * 1e-7;
        ASSERT_NEAR(answer, vic::math::exp(x), eps) << "x = " << x;
    }
}

TEST(TestUtils, TestUnique)
{
    struct SomeObject
    {
        int bla;
    };

    UniqueType<SomeObject> obj1;
    UniqueType<SomeObject> obj2;
    static_assert(!std::is_same_v<decltype(obj1), decltype(obj2)>);

    struct ObjectWithMutexes
    {
        using FirstMutex = UniqueType<std::mutex>;
        using SecondMutex = UniqueType<std::mutex>;
        static_assert(!std::is_same_v<FirstMutex, SecondMutex>);

        using FirstLock = std::unique_lock<FirstMutex>;
        using SecondLock = std::unique_lock<SecondMutex>;
        static_assert(!std::is_same_v<FirstLock, SecondLock>);

        FirstMutex m1;
        SecondMutex m2;

        void DoSomething1()
        {
            FirstLock lock1(m1);
            DoSomethingPrivate1(lock1);
        }

        void DoSomething2()
        {
            SecondLock lock2(m2);
            DoSomethingPrivate2(lock2);
        }

        void DoSomething1And2()
        {
            FirstLock lock1(m1, std::defer_lock);
            SecondLock lock2(m2, std::defer_lock);
            std::lock(lock1, lock2);

            DoSomethingPrivate1(lock1);
            DoSomethingPrivate2(lock2);
        }

    private:
        void DoSomethingPrivate1(const FirstLock& lock) { }
        void DoSomethingPrivate2(const SecondLock& lock) { }
    };

    ObjectWithMutexes object;
    object.DoSomething1();
    object.DoSomething2();
    object.DoSomething1And2();

    // make sure that two objects of the same type, which contain UniqueType<T>s,
    // are still the same object.
    ObjectWithMutexes object2;
    static_assert(std::is_same_v<decltype(object), decltype(object2)>);
}

TEST(TestUtils, Ranges)
{
    struct Type1
    { };
    struct Type2
    { };

    using Map1 = std::map<int, Type1>;
    using Map2 = std::map<int, Type2>;
    const Map1 map1{{1, {}}, {2, {}}, {3, {}}};
    Map2 map2{{3, {}}, {4, {}}, {5, {}}};

    static_assert(vic::ranges::view::MapConcept<std::map<int, float>>);
    static_assert(vic::ranges::view::MapConcept<Map1>);
    static_assert(vic::ranges::view::MapConcept<Map2>);
    // todo: flat map
    // todo: make sure that the iterators iterate in sorted order

    static_assert(std::same_as<Map1::key_type, Map2::key_type>);

    // auto mapOverlap = vic::ranges::view::map_intersection(map1, map2);

    //for(const auto& [key, type1, type2] : mapOverlap)
    //{
    //    // do stuff
    //}
}

TEST(TestUtils, Templates_Contains)
{
    static_assert(!templates::Contains<double>());

    static_assert(templates::Contains<double, double>());
    static_assert(templates::Contains<double, int, double>());
    static_assert(!templates::Contains<double, int>());
    static_assert(!templates::Contains<double, int, uint16_t>());
}

TEST(TestUtils, Templates_IsUnique)
{
    static_assert(templates::IsUnique<>());
    static_assert(templates::IsUnique<double>());
    static_assert(templates::IsUnique<double, int>());
    static_assert(templates::IsUnique<double, uint16_t, int>());
    static_assert(templates::IsUnique<double, float, uint16_t, int>());

    static_assert(!templates::IsUnique<double, double>());
    static_assert(!templates::IsUnique<double, uint16_t, double>());
    static_assert(!templates::IsUnique<double, uint16_t, float, double>());
    static_assert(!templates::IsUnique<double, double, uint16_t, float>());
    static_assert(!templates::IsUnique<double, double, double, double>());
    static_assert(!templates::IsUnique<int, double, uint16_t, double>());
}

TEST(TestUtils, Templates_Unique)
{
    //
    // static_assert(std::is_same_v<std::tuple<double>, typename templates::to_unique<double>::tuple_type>);
}

TEST(TestUtils, Serialize)
{
    //
}