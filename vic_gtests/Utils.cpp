
#include "gtest/gtest.h"

#include "test_base.h"
#include "vic/utils.h"
#include "vic/utils/algorithms.h"
#include "vic/utils/counted.h"
#include "vic/utils/crc32.h"
#include "vic/utils/decimal.h"
#include "vic/utils/heap.h"
#include "vic/utils/indexing.h"
#include "vic/utils/math.h"
#include "vic/utils/observable.h"
#include "vic/utils/permutations.h"
#include "vic/utils/ranges.h"
#include "vic/utils/rational.h"
#include "vic/utils/serialize.h"
#include "vic/utils/statemachine.h"
#include "vic/utils/string.h"
#include "vic/utils/timing.h"
#include "vic/utils/to_string.h"
#include "vic/utils/try.h"
#include "vic/utils/unique.h"

#include "vic/memory/constexpr_map.h"

#include <numeric>
#include <random>
#include <thread>

using namespace vic;

TEST(Utils, TestPow)
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

TEST(Utils, TestTimer)
{
    Timer timer;
    timer.Reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const double interval_ms = timer.GetTime<double, std::milli>().count();
    EXPECT_LT(9.99, interval_ms);
    EXPECT_LT(interval_ms, 50.); // large upper bound, we don't know when sleep returns
}

TEST(Utils, TestFinally)
{
    bool value = false;
    {
        Finally fin{[&]() { value = true; }};
    }
    EXPECT_TRUE(value);
}

TEST(Utils, TestCounted)
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

TEST(Utils, TestToBase)
{
    std::vector<int> buffer{};
    ToBase<int>(8, 2, buffer);
    ExpectVectorsEqual(buffer, std::vector<int>{0, 0, 0, 1});
}

TEST(Utils, TestFromBase)
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

TEST(Utils, TestToFromBase)
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

TEST(Utils, Linspace)
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

TEST(Utils, TestRange)
{
    // test range
    std::vector<int> result;
    for(const auto& val : Range(10))
        result.push_back(val);
    std::vector<int> answer{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    ASSERT_EQ(result, answer);

    // test subrange
    std::vector<int> result2;
    for(const auto& val : Range(2, 5))
        result2.push_back(val);
    std::vector<int> answer2{2, 3, 4};
    ASSERT_EQ(result2, answer2);

    // test different stride
    std::vector<int> result3;
    for(const auto& val : Range(0, 10, 4))
        result3.push_back(val);
    std::vector<int> answer3{0, 4, 8};
    ASSERT_EQ(result3, answer3);

    // test backwards stride
    std::vector<int> result4;
    for(const auto& val : Range(10, 0, -2))
        result4.push_back(val);
    std::vector<int> answer4{10, 8, 6, 4, 2};
    ASSERT_EQ(result4, answer4);

    // check range in wrong direction
    std::vector<int> result5;
    for(const auto& val : Range(0, 4, -1))
        result5.push_back(val);
    std::vector<int> answer5{};
    ASSERT_EQ(result5, answer5);

    std::vector<int> result6;
    for(const auto& val : Range(4, 0, 1))
        result6.push_back(val);
    std::vector<int> answer6{};
    ASSERT_EQ(result6, answer6);
}

//TEST(Utils, TestZip)
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

TEST(Utils, TestStateMachine)
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

TEST(Utils, TestNewStateMachine)
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

TEST(Utils, TestObservable)
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

TEST(Utils, TestIsPowerOfTwo)
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

TEST(Utils, Exp)
{
    //std::default_random_engine rng;
    //std::uniform_real_distribution dist(0.0001, 100.);

    //for(int32_t i = 0; i < 1000000; ++i)
    //{
    //    const double x = dist(rng);
    //    const double answer = std::exp(x);
    //    const double eps = answer * 1e-7;
    //    ASSERT_NEAR(answer, vic::math::exp(x), eps) << "x = " << x;
    //}
}

TEST(Utils, TestUnique)
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

TEST(Utils, Ranges)
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

TEST(Utils, Templates_Contains)
{
    static_assert(!templates::Contains<double>());

    static_assert(templates::Contains<double, double>());
    static_assert(templates::Contains<double, int, double>());
    static_assert(!templates::Contains<double, int>());
    static_assert(!templates::Contains<double, int, uint16_t>());
}

TEST(Utils, Templates_IsUnique)
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

TEST(Utils, ConstrainedPermutations)
{
    std::vector<uint8_t> items{1, 2, 3};
    std::vector<std::pair<uint8_t, uint8_t>> constraints{{1, 3}};

    const auto VerifyOrderLambda = [](const std::vector<uint8_t>& items, //
                                      const std::vector<std::pair<uint8_t, uint8_t>>& constraints,
                                      const std::vector<uint8_t>& permutation) {
        EXPECT_EQ(std::set(permutation.begin(), permutation.end()).size(), permutation.size()); // verify all items unique
    };

    VerifyOrderLambda(items, constraints, {1, 2, 3});

    std::vector<std::vector<uint8_t>> result;
    ConstrainedPermutations(items, //
                            constraints,
                            [&](const std::vector<uint8_t>& permutation) {
                                // result.push_back(permutation); //
                                VerifyOrderLambda(items, constraints, permutation);
                            });
}

TEST(Utils, Templates_Unique)
{
    //
    // static_assert(std::is_same_v<std::tuple<double>, typename templates::to_unique<double>::tuple_type>);
}

TEST(Utils, Serialize)
{
    //
}

TEST(Utils, RemoveDuplicates)
{
    const auto compareLambda = [](const auto& a, const auto& b) {
        return a == b; //
    };

    // empty
    auto vec = std::vector<int>{};
    vec.erase(vic::remove_duplicates(vec.begin(), vec.end(), compareLambda), vec.end());
    ASSERT_EQ(vec, (std::vector<int>{}));

    // purely unique
    vec = std::vector<int>{1, 2, 3, 4};
    vec.erase(vic::remove_duplicates(vec.begin(), vec.end(), compareLambda), vec.end());
    ASSERT_EQ(vec, (std::vector<int>{1, 2, 3, 4}));

    // purely duplicates
    vec = std::vector<int>{1, 1, 1, 1, 1, 1};
    vec.erase(vic::remove_duplicates(vec.begin(), vec.end(), compareLambda), vec.end());
    ASSERT_EQ(vec, (std::vector<int>{}));

    // mix of duplicates and unique values
    vec = std::vector<int>{1, 1, 1, 2, 2, 3, 4, 4, 4, 4, 5};
    vec.erase(vic::remove_duplicates(vec.begin(), vec.end(), compareLambda), vec.end());
    ASSERT_EQ(vec, (std::vector{3, 5}));

    vec = std::vector<int>{3, 1, 1, 4, 2, 2, 5};
    vec.erase(vic::remove_duplicates(vec.begin(), vec.end(), compareLambda), vec.end());
    ASSERT_EQ(vec, (std::vector{3, 4, 5}));
}

TEST(Utils, SortIndividual)
{
    {
        std::vector<int> vec{1, 4, 2, 3, 5};
        auto it = std::find(vec.begin(), vec.end(), 4);
        const auto newPositionIt = vic::sort_individual(vec.begin(), vec.end(), it);
        EXPECT_EQ(vec, (std::vector<int>{1, 2, 3, 4, 5}));
        EXPECT_EQ(*newPositionIt, 4);
    }

    {
        std::vector<int> vec{1, 3, 2, 5};
        auto it = std::find(vec.begin(), vec.end(), 2);
        const auto newPositionIt = vic::sort_individual(vec.begin(), vec.end(), it);
        EXPECT_EQ(vec, (std::vector<int>{1, 2, 3, 5}));
        EXPECT_EQ(*newPositionIt, 2);
    }

    {
        std::vector<int> vec{5, 1, 2, 3, 4};
        auto it = std::find(vec.begin(), vec.end(), 5);
        const auto newPositionIt = vic::sort_individual(vec.begin(), vec.end(), it);
        EXPECT_EQ(vec, (std::vector<int>{1, 2, 3, 4, 5}));
        EXPECT_EQ(*newPositionIt, 5);
    }

    {
        std::vector<int> vec{2, 3, 4, 5, 1};
        auto it = std::find(vec.begin(), vec.end(), 1);
        const auto newPositionIt = vic::sort_individual(vec.begin(), vec.end(), it);
        EXPECT_EQ(vec, (std::vector<int>{1, 2, 3, 4, 5}));
        EXPECT_EQ(*newPositionIt, 1);
    }
}

TEST(Utils, Indexing)
{
    // NOTE: this set of functions expects indices with the least significant digit in the last index of the array
    // translating them to reverse order should not be too complicated

    using namespace vic::indexing;

    // regardgess of shape, index [0,0,0,0,...] should always translate to flat index 0
    EXPECT_EQ(0, NdToFlatIndex<uint64_t>(std::vector{9, 4, 12, 576324}, std::vector{0, 0, 0, 0}));
    EXPECT_EQ(0, NdToFlatIndex<uint64_t>(std::vector{8, 7, 6, 5, 4, 3, 2, 1}, std::vector{0, 0, 0, 0, 0, 0, 0, 0}));

    const std::vector<std::size_t> shape = {2, 3, 4, 5};
    const uint64_t flatSize = FlatSize<uint64_t>(shape);

    ASSERT_EQ(0, NdToFlatIndex<uint64_t>(shape, std::vector{0, 0, 0, 0}));
    ASSERT_EQ(4, NdToFlatIndex<uint64_t>(shape, std::vector{0, 0, 0, 4}));
    ASSERT_EQ(5, NdToFlatIndex<uint64_t>(shape, std::vector{0, 0, 1, 0}));

    // check that each valid nd-index translates to a unique flat index
    std::set<uint64_t> indices;
    for(std::size_t i = 0; i < shape[0]; ++i)
        for(std::size_t j = 0; j < shape[1]; ++j)
            for(std::size_t k = 0; k < shape[2]; ++k)
                for(std::size_t l = 0; l < shape[3]; ++l)
                {
                    const auto ndIndex = std::vector{{i, j, k, l}};
                    const auto flatIdx = NdToFlatIndex<uint64_t>(shape, ndIndex);
                    indices.insert(flatIdx);
                    const auto reconstructedNdIndex = FlatToNdIndex<std::size_t>(shape, flatIdx);
                    EXPECT_EQ(ndIndex, reconstructedNdIndex);
                }
    ASSERT_EQ(indices.size(), flatSize);
    EXPECT_TRUE(*indices.rbegin() < flatSize);
}

TEST(Utils, UpdateHeap)
{
    EXPECT_EQ(LeftChild(0), 1);
    EXPECT_EQ(RightChild(0), 2);

    EXPECT_EQ(LeftChild(1), 3);
    EXPECT_EQ(RightChild(1), 4);

    EXPECT_EQ(LeftChild(2), 5);
    EXPECT_EQ(RightChild(2), 6);

    EXPECT_EQ(Parent(1), 0);
    EXPECT_EQ(Parent(2), 0);
    EXPECT_EQ(Parent(3), 1);
    EXPECT_EQ(Parent(4), 1);
    EXPECT_EQ(Parent(5), 2);
    EXPECT_EQ(Parent(6), 2);

    const auto compareLambda = [](const auto& a, const auto& b) { return a > b; };

    // decrease_key
    auto decrease = std::vector{1, 3, 4, 2, 5};

    auto itDecreased = std::find(decrease.begin(), decrease.end(), 2);
    auto it = decrease_key(decrease.begin(), itDecreased, compareLambda);

    EXPECT_EQ(decrease, (std::vector{1, 2, 4, 3, 5}));
    EXPECT_EQ(it, decrease.begin() + 1);
    EXPECT_EQ(*it, 2);
    EXPECT_TRUE(std::is_heap(decrease.begin(), decrease.end(), compareLambda));

    // increase_key
    auto increase = std::vector{1, 2, 3, 4, 5};
    EXPECT_TRUE(std::is_heap(increase.begin(), increase.end(), compareLambda));
    increase[0] = 6; // increase the value
    it = increase_key(increase.begin(), increase.end(), increase.begin() + 0, compareLambda);
    EXPECT_EQ(*it, 6);
    EXPECT_TRUE(std::is_heap(increase.begin(), increase.end(), compareLambda));

    std::default_random_engine rng;
    std::uniform_real_distribution dist(0.0001, 100.);

    const std::size_t heapSize = 100;
    std::uniform_int_distribution<std::size_t> randomIndex(0, heapSize - 1);

    for(auto run : Range(10))
    {
        // make a heap
        std::vector<double> vec;
        for(std::size_t i = 0; i < heapSize; ++i)
            vec.push_back(dist(rng));
        std::make_heap(vec.begin(), vec.end(), compareLambda);

        ASSERT_TRUE(std::is_heap(vec.begin(), vec.end(), compareLambda));

        for(auto i : Range(10))
        {
            // change some value in it
            const auto changeIndex = randomIndex(rng);
            const auto oldValue = vec[changeIndex];
            const auto newValue = dist(rng);
            vec[changeIndex] = newValue;

            if(newValue < oldValue)
                decrease_key(vec.begin(), vec.begin() + changeIndex, compareLambda);
            else
                increase_key(vec.begin(), vec.end(), vec.begin() + changeIndex, compareLambda);

            // check that the heap is valid again
            const auto isHeap = std::is_heap(vec.begin(), vec.end(), compareLambda);
            if(!isHeap)
                ASSERT_TRUE(isHeap);
        }
    }
}

TEST(Utils, SplitString)
{
    {
        const std::string testString = "a bc  def ";
        const auto substrings = Split(testString, " ");

        EXPECT_EQ(substrings.size(), 3);
        EXPECT_EQ(substrings.at(0), "a");
        EXPECT_EQ(substrings.at(1), "bc");
        EXPECT_EQ(substrings.at(2), "def");
    }

    {
        const auto substrings = Split("", " ");
        EXPECT_EQ(substrings.size(), 0);
    }

    {
        const auto substrings = Split(" ", " ");
        EXPECT_EQ(substrings.size(), 0);
    }

    {
        const auto substrings = Split("  ", " ");
        EXPECT_EQ(substrings.size(), 0);
    }

    {
        const auto substrings = Split("abc", " ");
        EXPECT_EQ(substrings.size(), 1);
        EXPECT_EQ(substrings.at(0), "abc");
    }
}

TEST(Utils, ToIntegral)
{

    EXPECT_EQ(ToIntegral<int>("1234"), 1234);
    EXPECT_EQ(ToIntegral<int>("-1234"), -1234);

    EXPECT_EQ(ToIntegral<long>("1234"), 1234);
    EXPECT_EQ(ToIntegral<long>(" 1234"), 1234);
    EXPECT_EQ(ToIntegral<long>("1234 "), 1234);
}

//TEST(Utils, Decimal)
//{
//
//    // initialize
//
//    EXPECT_EQ(dec0::FromIntegral(1).val, 1);
//    EXPECT_EQ(dec1::FromIntegral(1).val, 10);
//    EXPECT_EQ(dec2::FromIntegral(1).val, 100);
//
//    EXPECT_EQ(exp0::FromIntegral(1000).val, 1000);
//    EXPECT_EQ(exp1::FromIntegral(1000).val, 100);
//    EXPECT_EQ(exp2::FromIntegral(1000).val, 10);
//    EXPECT_EQ(exp3::FromIntegral(1000).val, 1);
//
//    EXPECT_EQ(dec1::FromFloat(1.).val, 10);
//    EXPECT_EQ(dec2::FromFloat(1.).val, 100);
//    EXPECT_EQ(dec3::FromFloat(1.).val, 1000);
//
//    EXPECT_EQ(exp0::FromFloat(1000.).val, 1000);
//    EXPECT_EQ(exp1::FromFloat(1000.).val, 100);
//    EXPECT_EQ(exp2::FromFloat(1000.).val, 10);
//    EXPECT_EQ(exp3::FromFloat(1000.).val, 1);
//
//    // ToUnit
//
//    EXPECT_EQ(dec0::FromIntegral(1).ToIntegral(), 1);
//    EXPECT_EQ(dec1::FromIntegral(1).ToIntegral(), 1);
//    EXPECT_EQ(dec2::FromIntegral(1).ToIntegral(), 1);
//
//    EXPECT_EQ(exp0::FromIntegral(1000).ToIntegral(), 1000);
//    EXPECT_EQ(exp1::FromIntegral(1000).ToIntegral(), 1000);
//    EXPECT_EQ(exp2::FromIntegral(1000).ToIntegral(), 1000);
//
//    // to
//
//    EXPECT_EQ(To<dec2>(dec1::FromIntegral(1)).val, 100);
//    EXPECT_EQ(To<dec3>(dec1::FromIntegral(1)).val, 1000);
//
//    const dec1 d1{5}; // 0.5
//    const dec1 d2{5}; // 0.5
//
//    EXPECT_EQ(To<dec1>(d1).val, 5);
//    EXPECT_EQ(To<dec2>(d1).val, 50);
//    EXPECT_EQ(To<dec3>(d1).val, 500);
//
//    const auto d3 = dec3::FromIntegral(5);
//
//    EXPECT_EQ(To<dec1>(d3).val, 50);
//    EXPECT_EQ(To<dec2>(d3).val, 500);
//    EXPECT_EQ(To<dec3>(d3).val, 5000);
//
//    // add
//
//    // NOTE: currently flooring, might not be what we want.
//    // we could calculate the sum in the highest common representation, and then convert to the actual representation
//    EXPECT_EQ(vic::Add<dec0>(d1, d2).val, 0);
//    EXPECT_EQ(vic::Add<dec1>(d1, d2).val, 10);
//    EXPECT_EQ(vic::Add<dec2>(d1, d2).val, 100);
//}

TEST(Utils, Try)
{
    vic::Try([]() { return true; });
}

TEST(Utils, Deadline)
{
    vic::Deadline([]() { return true; }, std::chrono::seconds(1));
}

TEST(Utils, Templates)
{
    // make sure the statement is constexpr
    static constexpr auto i1 = templates::GetIndex<double, double, int>();
    static constexpr auto i2 = templates::GetIndex<double, int, double>();

    EXPECT_EQ(i1, 0);
    EXPECT_EQ(i2, 1);
}

TEST(Utils, CRC32)
{
    const char* trivial = "";
    EXPECT_EQ(crc32(trivial, strlen(trivial)), 0x00000000);

    const char* text1 = "The quick brown fox jumps over the lazy dog";
    EXPECT_EQ(crc32(text1, strlen(text1)), 0x414FA339);

    const char* text2 = "various CRC algorithms input data";
    EXPECT_EQ(crc32(text2, strlen(text2)), 0x9BD366AE);

    const char* text3 = "Test vector from febooti.com";
    EXPECT_EQ(crc32(text3, strlen(text3)), 0x0C877F61);
}

TEST(Utils, Rational)
{
    using Kilo = Rational<int, 1000, 1>;
    using Hecto = Rational<int, 100, 1>;
    using Unit = Rational<int, 1, 1>;
    using Deci = Rational<int, 1, 10>;
    using Centi = Rational<int, 1, 100>;
    using Milli = Rational<int, 1, 1000>;
    using Micro = Rational<int, 1, 1000000>;

    using Half = Rational<int, 1, 2>;
    using Third = Rational<int, 1, 3>;
    using Quarter = Rational<int, 1, 4>;

    static_assert(ConceptRational<Deci> && ConceptRational<Centi> && ConceptRational<Milli>);
    static_assert(!ConceptRational<int> && !ConceptRational<float> && !ConceptRational<double>);

    static_assert(Numeric<int> && Numeric<float> && Numeric<double>);
    static_assert(!Numeric<Deci> && !Numeric<Centi> && !Numeric<Milli>);

    // simplify type
    static_assert(std::is_same_v<rational_simplify<Rational<int, 1, 1>>::type, Rational<int, 1, 1>>);
    static_assert(std::is_same_v<rational_simplify<Rational<int, 1, 123547>>::type, Rational<int, 1, 123547>>);
    static_assert(std::is_same_v<rational_simplify<Rational<int, 2, 2000>>::type, Milli>);
    static_assert(std::is_same_v<rational_simplify<Rational<int, 13, 13>>::type, Rational<int, 1, 1>>);
    static_assert(std::is_same_v<rational_simplify<Rational<int, 2000, 20>>::type, Rational<int, 100, 1>>);
    static_assert(std::is_same_v<rational_simplify<Rational<int, 5, 500>>::type, Rational<int, 1, 100>>);

    // addition type
    static_assert(std::is_same_v<rational_addition_t<Rational<int, 1, 2>, //
                                                     Rational<int, 1, 3>>, //
                                 Rational<int, 1, 6>>);
    static_assert(std::is_same_v<rational_addition_t<Rational<int, 1, 2>, //
                                                     Rational<int, 2, 3>>, //
                                 Rational<int, 1, 6>>);

    // multiply type
    static_assert(std::is_same_v<rational_multiplication_t<Rational<int, 1, 2>, //
                                                           Rational<int, 1, 2>>, //
                                 Rational<int, 1, 4>>);
    static_assert(std::is_same_v<rational_multiplication_t<Rational<int, 2, 3>, //
                                                           Rational<int, 3, 4>>, //
                                 Rational<int, 1, 2>>);

    // empty construction
    Milli empty;
    Milli empty2{};

    // construct a rational from normal value
    Milli m1(1000);
    Milli m2{2000};
    Milli m3 = 3000;

    EXPECT_EQ(m1.val, 1000);
    EXPECT_EQ(m2.val, 2000);
    EXPECT_EQ(m3.val, 3000);

    // check conversion to int
    EXPECT_EQ((int)m1, 1000);
    EXPECT_EQ((int)m2, 2000);
    EXPECT_EQ((int)m3, 3000);

    // test conversion to bool
    EXPECT_FALSE((bool)Milli{});
    EXPECT_TRUE((bool)Milli{1});
    EXPECT_FALSE(Milli{});
    EXPECT_TRUE(Milli{} == 0);
    EXPECT_TRUE(0 == Milli{});

    EXPECT_TRUE(Milli{999} < 1);
    EXPECT_FALSE(Milli{1000} < 1);
    EXPECT_TRUE(Milli{1000} <= 1);
    EXPECT_FALSE(Milli{1001} <= 1);

    EXPECT_TRUE(Quarter{1} < Third{1});
    EXPECT_TRUE(Half{1} == Quarter{2});

    // assigning value of one to the other
    m2 = m1;
    EXPECT_EQ(m2.val, 1000);
    m3 = 1000;
    EXPECT_EQ(m3.val, 1000);

    // to
    EXPECT_EQ(To<Deci>(Milli(1000)), Deci(10));
    EXPECT_EQ(To<Milli>(Deci(10)), Milli(1000));

    // addition
    EXPECT_EQ((int)vic::Add(Milli(1000), Milli(1000)), 2000);
    EXPECT_EQ((int)(Milli(1000) + Milli(1000)), 2000);

    EXPECT_EQ(Milli(1000) + 1, Milli(2000));

    // subtract
    EXPECT_EQ((int)vic::Subtract(Milli(1000), Milli(1000)), 0);

    // multiplication
    EXPECT_EQ(Multiply(Kilo{1}, Milli{1}), Unit{1});
    EXPECT_EQ(Multiply(1000, Milli{1}), Unit{1});
    EXPECT_EQ(Multiply(Milli{1}, 1000), Unit{1});

    // todo: roundoff causes devide by 0
    // division
    //EXPECT_EQ(Devide(Kilo{1}, Milli{1}), Unit{1000000});
    //EXPECT_EQ(Devide(1000, Milli{1}), Unit{1000000});
    //EXPECT_EQ(Devide(Micro{1000}, 1000), Micro{1});

    // equality
    EXPECT_EQ(Milli(1000), Centi(100));
    EXPECT_EQ(Deci(10), Centi(100));
    EXPECT_EQ(Deci(10), Milli(1000));

    // less than
}