
#include "gtest/gtest.h"

#include "test_base.h"
#include "vic/memory/flat_linked_list.h"
#include "vic/memory/flat_map.h"
#include "vic/memory/flat_set.h"
#include "vic/memory/merge_sort.h"
#include "vic/memory/refcounter.h"
#include "vic/memory/ring_buffer.h"

#include "vic/utils/counted.h"
#include "vic/utils/map_iterate.h"
#include "vic/utils/to_string.h"

#include <algorithm>
#include <deque>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

using namespace vic::memory;

TEST(Memory, ref)
{
    struct MyType
    {
        MyType(int x)
            : val(x)
        { }
        int val;
    };

    auto refptr = vic::make_refcounted<MyType>(1);
}

TEST(Memory, intrusive_ref)
{
    int destructorCalls = 0;
    struct IntrusiveTestType : public vic::intrusive_ref<IntrusiveTestType>
    {
        explicit IntrusiveTestType(int& destructorCalls)
            : mDestructorCalls(destructorCalls)
        { }
        ~IntrusiveTestType() { mDestructorCalls++; }

        int& mDestructorCalls;
    };

    vic::intrusive<IntrusiveTestType> intrusivePtr; // empty ptr
    EXPECT_TRUE(intrusivePtr.empty());
    EXPECT_EQ(intrusivePtr.count(), 0);
    EXPECT_FALSE(intrusivePtr);

    intrusivePtr = vic::make_intrusive<IntrusiveTestType>(destructorCalls);

    EXPECT_FALSE(intrusivePtr.empty());
    EXPECT_EQ(intrusivePtr.count(), 1);
    EXPECT_TRUE(intrusivePtr);

    {
        auto copy = intrusivePtr;
        EXPECT_FALSE(copy.empty());
        EXPECT_EQ(copy.count(), 2);
        EXPECT_FALSE(intrusivePtr.empty());
        EXPECT_EQ(intrusivePtr.count(), 2);
    }

    EXPECT_EQ(destructorCalls, 0); // copy destructor should not delete underlying data

    EXPECT_FALSE(intrusivePtr.empty());
    EXPECT_EQ(intrusivePtr.count(), 1);
    EXPECT_TRUE(intrusivePtr);

    auto moved = std::move(intrusivePtr);
    EXPECT_TRUE(intrusivePtr.empty());
    EXPECT_EQ(intrusivePtr.count(), 0);
    EXPECT_FALSE(intrusivePtr);
    EXPECT_FALSE(moved.empty());
    EXPECT_EQ(moved.count(), 1);
    EXPECT_TRUE(moved);

    moved.clear();
    EXPECT_TRUE(moved.empty());
    EXPECT_EQ(moved.count(), 0);
    EXPECT_EQ(destructorCalls, 1);
}

TEST(Memory, RefCounted)
{
    struct TestStruct
    {
        TestStruct(int* value)
            : mValue(value)
        { }
        ~TestStruct() { *mValue = 10; }
        int* mValue;

        void SetValue(const int val) { *mValue = val; }
        int GetValue() { return *mValue; }
    };

    int value = 1;
    {
        RefCounted<TestStruct> empty;
        EXPECT_EQ(empty.Get(), nullptr);
        EXPECT_EQ(empty.use_count(), 0);

        // construct a RefCounted object
        RefCounted<TestStruct> counted{&value};
        EXPECT_EQ(counted.use_count(), 1);

        // test dereferencing object
        counted->SetValue(2);
        EXPECT_EQ(value, 2);

        // check getting a reference to the contained object
        TestStruct* ptr = counted.Get();

        // copy the RefCounter, access data through it,
        // check that refcount is reset after it is destructed.
        {
            RefCounted<TestStruct> copy{counted};
            EXPECT_EQ(counted.use_count(), 2);
            EXPECT_EQ(copy.use_count(), 2);

            counted->SetValue(3);
            EXPECT_EQ(value, 3);

            copy->SetValue(4);
            EXPECT_EQ(value, 4);
        }
        EXPECT_EQ(counted.use_count(), 1);

        empty = counted; // assign counted ref to empty ref

        EXPECT_EQ(empty.use_count(), 2);
        EXPECT_EQ(counted.use_count(), 2);
    }

    // destructor sets value to 10
    EXPECT_EQ(value, 10);
}

using KeyType = std::size_t;
struct TestStruct
{
    KeyType id;
    friend std::ostream& operator<<(std::ostream& os, const TestStruct& item);
};

std::ostream& operator<<(std::ostream& os, const TestStruct& item)
{
    os << "TestStruct<id:" << std::to_string(item.id) << ">";
    return os;
}

struct MyType
{
    int val;
};

inline bool operator<(const MyType& a, const MyType& b) { return a.val < b.val; }

TEST(Memory, FlatMap)
{
    vic::memory::FlatMap<uint32_t, MyType> map{};

    ASSERT_TRUE(map.empty());
    ASSERT_EQ(map.size(), 0);
    ASSERT_EQ(map.count(0), 0);
    auto it = map.find(0);
    EXPECT_TRUE(it == map.end());

    map[5] = {1};
    ASSERT_FALSE(map.empty());
    ASSERT_EQ(map.size(), 1);
    it = map.find(0);
    EXPECT_TRUE(it == map.end());
    ASSERT_EQ(map.count(0), 0);
    ASSERT_EQ(map.count(5), 1);
    ASSERT_EQ(map.find(5), map.begin());
    ASSERT_NE(map.find(5), map.end());

    map[4] = {2};
    map[5] = {3};

    ASSERT_FALSE(map.empty());
    ASSERT_EQ(map.size(), 2);
    ASSERT_EQ(map.count(0), 0);
    ASSERT_EQ(map.count(5), 1);
    ASSERT_EQ(map.at(5).val, 3);

    map.erase(5);
    ASSERT_EQ(map.size(), 1);
    EXPECT_TRUE(map.find(5) == map.end());

    map[10] = {4};
    map[9] = {5};
    map[8] = {6};
    map[7] = {7};
    map[6] = {8};

    uint32_t key = map.begin()->first;
    for(auto it = map.begin() + 1; it < map.end(); ++it)
    {
        EXPECT_TRUE(key < it->first);
        key = it->first;
    }

    EXPECT_TRUE(map.Relabel(9, 11));
    EXPECT_EQ(std::prev(map.end())->first, 11);

    EXPECT_TRUE(map.Relabel(7, 3));
    EXPECT_EQ(map.begin()->first, 3);
    EXPECT_EQ(map.begin()->second.val, 7);

    EXPECT_FALSE(map.Relabel(4, 3)); // key 3 already exists
}

TEST(Memory, FlatMapConstructors)
{
    vic::memory::FlatMap<uint32_t, std::vector<int>> map{};

    map[4] = {4};
    EXPECT_EQ(map[4], std::vector<int>{4});
    map[3] = {3};
    EXPECT_EQ(map[3], std::vector<int>{3});
    EXPECT_EQ(map[4], std::vector<int>{4});
    EXPECT_EQ(map[5], std::vector<int>{});

    auto vec = std::vector<int>{6};
    map.emplace(6, std::move(vec));
    EXPECT_EQ(map[6], std::vector<int>{6});
    EXPECT_TRUE(vec.empty()); // todo: there is no good way to detect if a vector has been moved, the standards gives no guarantee

    struct NonDefaultConstructible
    {
        explicit NonDefaultConstructible(int x)
            : val(x)
        { }
        NonDefaultConstructible() = delete; // disable default construct
        NonDefaultConstructible& operator=(const NonDefaultConstructible& other) = delete;
        NonDefaultConstructible(const NonDefaultConstructible&) = delete; // disable copy construct

        NonDefaultConstructible& operator=(NonDefaultConstructible&& other) noexcept = default;
        NonDefaultConstructible(NonDefaultConstructible&&) noexcept = default; //  allow move
        int val;
    };
    NonDefaultConstructible a(1);
    NonDefaultConstructible b = std::move(a);
    // NonDefaultConstructible c = b; // should not work

    vic::memory::FlatMap<uint32_t, NonDefaultConstructible> map2{};
    map2.emplace(1, NonDefaultConstructible(1));
    map2.emplace(0, 0);

    EXPECT_EQ(map2.at(0).val, 0);
    EXPECT_EQ(map2.at(1).val, 1);

    auto movable = NonDefaultConstructible(2);
    map2.emplace(2, std::move(movable));

    EXPECT_EQ(map2.at(2).val, 2);
}

TEST(Memory, ArrayRingBuffer)
{
    struct TestStruct
    {
        int a;
        float b;
    };

    constexpr std::size_t capacity = 100;

    ArrayRingBuffer<TestStruct, capacity> buffer;

    // verify empty buffer
    ASSERT_EQ(buffer.Capacity(), capacity);
    ASSERT_EQ(buffer.Size(), 0);
    ASSERT_EQ(buffer.Pop(), std::nullopt);

    // put in 1 item, verify value
    buffer.TryPush({1, 1.});
    ASSERT_EQ(buffer.Size(), 1);
    auto item = buffer.Pop();
    ASSERT_NE(item, std::nullopt);
    EXPECT_EQ(item.value().a, 1);
    ASSERT_EQ(buffer.Size(), 0);

    // test with a separate push and pop thread
    constexpr std::size_t n = 1000000;

    int previous = -1;
    int count = 0;

    {
        std::jthread pushThread([&]() {
            // push n items into buffer, with increasing value of a
            for(int i = 0; i < n; ++i)
                while(!buffer.TryPush({i, 0.}))
                    std::this_thread::yield(); // yield, because the thread cannot continue right now
        });

        std::jthread popThread([&]() {
            // pop items, check that the value always increases
            while(true)
            {
                const auto item = buffer.Pop();
                if(item == std::nullopt)
                {
                    std::this_thread::yield();
                    continue;
                }
                if(item.value().a <= previous)
                    break; // count will not be correct
                count++;
                previous = item.value().a;
                if(previous == n - 1)
                    break; // done
            }
        });
    }

    EXPECT_EQ(buffer.Size(), 0);
    EXPECT_EQ(previous, n - 1);
    EXPECT_EQ(count, n);

    // todo: test with non-copy constructible type (unique_ptr?)

    // todo: test with multiple producers/consumers
}

TEST(Memory, RingBuffer)
{
    for(std::size_t ringSize : {4, 8, 16, 32})
    {
        for(std::size_t index : vic::Range<std::size_t>(0, ringSize))
        {
            const auto next = NextRingIndex(index, ringSize);
            const auto previousNext = PreviousRingIndex(next, ringSize);
            EXPECT_EQ(index, previousNext);

            const auto previous = PreviousRingIndex(index, ringSize);
            const auto nextPrevious = NextRingIndex(previous, ringSize);
            EXPECT_EQ(index, nextPrevious);
        }
    }

    struct MyType
    {
        int a;
    };
    RingBuffer<MyType> buffer; // <[]>
    EXPECT_EQ(buffer.size(), 0);
    EXPECT_EQ(buffer.try_pop_front(), std::nullopt);
    EXPECT_EQ(buffer.try_pop_back(), std::nullopt);

    buffer.push_back(MyType{1}); // <[1]>
    EXPECT_EQ(buffer.size(), 1);
    EXPECT_EQ(buffer.front().a, 1);
    EXPECT_EQ(buffer.back().a, 1);

    buffer.push_back(MyType{2}); // <[1, 2]>
    EXPECT_EQ(buffer.size(), 2);
    EXPECT_EQ(buffer.front().a, 1);
    EXPECT_EQ(buffer.back().a, 2);

    buffer.push_back(MyType{3}); // <[1, 2, 3]>
    EXPECT_EQ(buffer.size(), 3);
    EXPECT_EQ(buffer.front().a, 1);
    EXPECT_EQ(buffer.back().a, 3);

    MyType res = buffer.pop_front(); // <., [2, 3]>
    EXPECT_EQ(res.a, 1);
    EXPECT_EQ(buffer.size(), 2);
    EXPECT_EQ(buffer.front().a, 2);
    EXPECT_EQ(buffer.back().a, 3);

    res = buffer.pop_front(); // <., ., [3]>
    EXPECT_EQ(res.a, 2);
    EXPECT_EQ(buffer.size(), 1);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 3);

    buffer.push_back(MyType{4}); // <., ., [3, 4]>
    EXPECT_EQ(buffer.size(), 2);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 4);

    buffer.push_back(MyType{5}); // <5], ., [3, 4,>
    EXPECT_EQ(buffer.size(), 3);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 5);

    buffer.push_back(MyType{6}); // <5, 6], [3, 4,>
    EXPECT_EQ(buffer.size(), 4);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 6);

    // default size is 4, so next push_back will perform a reallocation
    buffer.push_back(MyType{7}); // <[3, 4, 5, 6, 7], ., ., .>
    EXPECT_EQ(buffer.size(), 5);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 7);

    // after reallocation, a push_front will add an item to the back of the list
    buffer.push_front(MyType{2}); // <3, 4, 5, 6, 7], ., ., [2, >
    EXPECT_EQ(buffer.size(), 6);
    EXPECT_EQ(buffer.front().a, 2);
    EXPECT_EQ(buffer.back().a, 7);

    // pop front when front is at the end
    res = buffer.pop_front(); // <[3, 4, 5, 6, 7], ., ., ., >
    EXPECT_EQ(res.a, 2);
    EXPECT_EQ(buffer.size(), 5);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 7);

    // pop
    res = buffer.pop_back();
    EXPECT_EQ(res.a, 7);
    EXPECT_EQ(buffer.size(), 4);
    EXPECT_EQ(buffer.front().a, 3);
    EXPECT_EQ(buffer.back().a, 6);

    //// todo: construct an std::deque. Perform the same operations on a ringbuffer and queue, check that they agree.
    //buffer.clear();
    //std::deque<MyType> verification;
    //for(std::size_t step = 0; step < 99; ++step)
    //{
    //    // todo: push some items to front or back

    //    // todo: pop some items from front or back

    //    // todo: verify that both lists agree
    //    ASSERT_EQ(buffer.size(), verification.size());
    //    for(std::size_t i = 0; i < buffer.size(); ++i)
    //        ASSERT_EQ(buffer.at(i).a, verification.at(i).a);
    //}
}

TEST(Memory, UnorderedFlatSet)
{
    std::set<uint32_t> stdSet;
    vic::memory::FlatSet<uint32_t> flatSet;
    vic::memory::UnorderedFlatSet<uint32_t> unorderedFlatSet;

    auto trueOrFalse = std::bind(std::uniform_int_distribution<>(0, 3), std::default_random_engine()); // bias towards inserting
    auto randomInt = std::bind(std::uniform_int_distribution<>(0, 100), std::default_random_engine());
    auto randomIndex = std::bind(std::uniform_int_distribution<>(0, 100000000), std::default_random_engine());

    for(std::size_t i = 0; i < 1000; ++i)
    {
        if(stdSet.empty() || (bool)trueOrFalse()) // insert
        {
            const uint32_t value = randomInt();
            stdSet.insert(value);
            flatSet.insert(value);
            unorderedFlatSet.insert(value);
        }
        else // remove
        {
            // grab a random value from the set
            const std::size_t index = randomIndex() % stdSet.size();
            const uint32_t removeValue = *std::next(stdSet.begin(), index);

            stdSet.erase(removeValue);
            flatSet.erase(removeValue);
            unorderedFlatSet.erase(removeValue);
        }

        EXPECT_EQ(stdSet.size(), flatSet.size());
        EXPECT_EQ(stdSet.size(), unorderedFlatSet.size());

        for(const auto value : stdSet)
        {
            // check that all sets contain the same items
            EXPECT_TRUE(flatSet.contains(value));
            EXPECT_TRUE(unorderedFlatSet.contains(value));
        }
    }
}

TEST(Memory, FlatLinkedList)
{
    vic::FlatLinkedList<double> linkedList;
    EXPECT_EQ(linkedList.size(), 0);
    EXPECT_TRUE(linkedList.empty());

    linkedList.PushBack(1.);
    double& one = linkedList.at(0);
    EXPECT_DOUBLE_EQ(one, 1.);
    EXPECT_EQ(linkedList.size(), 1);
    EXPECT_FALSE(linkedList.empty());

    linkedList.PushBack(2.);
    double& two = linkedList.at(1);
    EXPECT_DOUBLE_EQ(two, 2.);
    EXPECT_EQ(linkedList.size(), 2);

    linkedList.PushBack(3.); // 1, 2, 3
    double& three = linkedList.at(2);
    EXPECT_DOUBLE_EQ(three, 3.);
    EXPECT_EQ(linkedList.size(), 3);

    // now put a new item at the front of the linked list
    linkedList.PushFront(0.); // data: 1,2,3,0; linked list: 0,1,2,3
    double& zero = linkedList.at(0);
    EXPECT_DOUBLE_EQ(zero, 0.);
    EXPECT_EQ(linkedList.size(), 4);

    linkedList.PopBack();
    EXPECT_EQ(linkedList.size(), 3);
    EXPECT_DOUBLE_EQ(linkedList.at(0), 0.);
    EXPECT_DOUBLE_EQ(linkedList.at(2), 2.);

    linkedList.PopFront();
    EXPECT_EQ(linkedList.size(), 2);
    EXPECT_DOUBLE_EQ(linkedList.at(0), 1.);
    EXPECT_DOUBLE_EQ(linkedList.at(1), 2.);

    // Make sure that flush does not alter the data
    linkedList.Flush();
    EXPECT_EQ(linkedList.size(), 2);
    EXPECT_DOUBLE_EQ(linkedList.at(0), 1.);
    EXPECT_DOUBLE_EQ(linkedList.at(1), 2.);

    // operator[]
    EXPECT_DOUBLE_EQ(linkedList[0], 1.);
    EXPECT_DOUBLE_EQ(linkedList[1], 2.);

    // todo: iterator (forward/backward)
}

TEST(Memory, MapOverlap)
{
    using KeyType = int;
    struct A
    {
        int a;
    };
    struct B
    {
        int b;
    };

    std::map<KeyType, A> map1{{{1, {11}}, //
                               {3, {13}},
                               {5, {15}},
                               {7, {17}}}};
    std::map<KeyType, B> map2{{{2, {22}}, //
                               {3, {23}},
                               {4, {24}},
                               {7, {27}}}};

    // const
    std::set<KeyType> keys;

    for(const auto& [key, value1, value2] : vic::Overlap(map1, map2))
    {
        EXPECT_EQ(key, value1->a % 10);
        EXPECT_EQ(key, value2->b % 10);
        keys.insert(key);
    }
    EXPECT_EQ(keys, (std::set<KeyType>{3, 7}));

    //// non-const
    //for(auto& [key, first, second] : vic::Overlap(map1, map2))
    //{
    //    first.a += 1;
    //    second.b += 1;
    //}

    std::map<int, std::string> data = {{2, "bla"}, {1, "bla"}};

    auto it = data.begin();

    for(auto& [a, b] : data)
        std::cout << a << "; " << b << std::endl;

    // same problem:
    // https://stackoverflow.com/questions/49628401/structured-bindings-and-tuple-of-references
}

TEST(Memory, MoveMerge)
{
    // define some type that benefits from moving over copying
    using Item = std::vector<int>;

    std::vector<Item> a;
    for(int i = 0; i < 6; i++)
        a.push_back(Item{i});
    std::vector<Item> b;
    for(int i = 6; i < 10; i++)
        b.push_back(Item{i});

    std::vector<Item> c;
    vic::sorting::move_merge(a.begin(), //
                             a.end(),
                             b.begin(),
                             b.end(),
                             std::back_inserter(c));

    for(const auto& item : a)
        EXPECT_EQ(item.size(), 0);

    for(const auto& item : b)
        EXPECT_EQ(item.size(), 0);

    for(std::size_t i = 0; i < c.size() - 1; ++i)
        EXPECT_LT(c.at(i).at(0), c.at(i + 1).at(0));

    //  randomized test

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-100., 100.); // todo: log norm
    std::uniform_int_distribution<> size(0, 1000);

    for(auto iter : vic::Range(10))
    {
        std::vector<double> vec1;
        for(auto i : vic::Range(size(g)))
            vec1.push_back(dist(g));
        std::sort(vec1.begin(), vec1.end());

        std::vector<double> vec2;
        for(auto i : vic::Range(size(g)))
            vec2.push_back(dist(g));
        std::sort(vec2.begin(), vec2.end());

        std::vector<double> full;
        std::copy(vec1.begin(), vec1.end(), std::back_inserter(full));
        std::copy(vec2.begin(), vec2.end(), std::back_inserter(full));
        std::sort(full.begin(), full.end());

        std::vector<double> moveMerge;
        vic::sorting::move_merge(vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), std::back_inserter(moveMerge));

        ASSERT_EQ(full.size(), moveMerge.size());
        for(std::size_t i = 0; i < full.size(); ++i)
            ASSERT_DOUBLE_EQ(full.at(i), moveMerge.at(i)) << i;
    }
}