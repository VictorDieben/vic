
#include "gtest/gtest.h"

#include "test_base.h"
#include "vic/memory/flat_map.h"
#include "vic/memory/flat_set.h"
#include "vic/memory/merge_sort.h"
#include "vic/memory/refcounter.h"
#include "vic/memory/ring_buffer.h"

#include "vic/utils/map_iterate.h"
#include "vic/utils/to_string.h"

#include <algorithm>
#include <deque>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

using namespace vic::memory;

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

    map[4] = {2};
    map[5] = {3};

    ASSERT_FALSE(map.empty());
    ASSERT_EQ(map.size(), 2);
    ASSERT_EQ(map.count(0), 0);
    ASSERT_EQ(map.count(5), 1);

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

    EXPECT_FALSE(map.Relabel(4, 3)); // key 3 already exists
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

    std::thread pushThread([&]() {
        // push n items into buffer, with increasing value of a
        for(int i = 0; i < n; ++i)
            while(!buffer.TryPush({i, 0.}))
                std::this_thread::yield(); // yield, because the thread cannot continue right now
    });

    int previous = -1;
    int count = 0;
    std::thread popThread([&]() {
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

    pushThread.join();
    popThread.join();
    EXPECT_EQ(buffer.Size(), 0);
    EXPECT_EQ(previous, n - 1);
    EXPECT_EQ(count, n);

    // todo: test with non-copy constructible type (unique_ptr?)

    // todo: test with multiple producers/consumers
}

TEST(Memory, RingBuffer)
{
    struct MyType
    {
        int a;
    };
    RingBuffer<MyType> buffer; // <[]>
    EXPECT_EQ(buffer.size(), 0);

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

    buffer.pop_back(); // <., [2, 3]>
    EXPECT_EQ(buffer.size(), 2);
    EXPECT_EQ(buffer.front().a, 2);
    EXPECT_EQ(buffer.back().a, 3);

    buffer.pop_back(); // <., ., [3]>
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

    // todo: construct an std::deque. Perform the same operations on a ringbuffer and queue, check that they agree.
    buffer.clear();
    std::deque<MyType> verification;
    for(std::size_t step = 0; step < 99; ++step)
    {
        // todo: push some items to front or back

        // todo: pop some items from front or back

        // todo: verify that both lists agree
        ASSERT_EQ(buffer.size(), verification.size());
        for(std::size_t i = 0; i < buffer.size(); ++i)
            ASSERT_EQ(buffer.at(i).a, verification.at(i).a);
    }
}

TEST(Memory, MergeSort)
{
    // construct a vector with increasing numbers
    std::vector<int> answer;
    answer.resize(10);
    std::iota(answer.begin(), answer.end(), 0);

    // make a shuffled copy
    auto values = answer;
    std::random_device rd;
    std::mt19937 g(1234);
    std::shuffle(values.begin(), values.end(), g);

    // sort two halves of the vector separately
    auto midpoint = values.begin() + 6;
    std::sort(values.begin(), midpoint);
    std::sort(midpoint, values.end());

    // now combine the two sorted halves
    vic::sorting::merge_sort(values.begin(), midpoint, values.end());

    // check values
    EXPECT_EQ(values, answer);
}

TEST(Memory, MergeSort2)
{
    using namespace vic;

    std::vector<int> vec = {1, 3, 7, 9, 11, 2, 4, 8, 10, 12}; //
    std::cout << vec << std::endl;

    vic::sorting::merge_sort(vec.begin(), vec.begin() + 5, vec.end());
    std::cout << vec << std::endl;

    const std::vector<int> answer = {1, 2, 3, 4, 7, 8, 9, 10, 11, 12};

    EXPECT_EQ(vec, answer);
}

TEST(Memory, MergeSortEdgeCases)
{
    // sort an empty list, should work
    std::vector<int> vec = {};
    std::vector<int> answer = {};
    vic::sorting::merge_sort(vec.begin(), vec.begin(), vec.end());
    EXPECT_EQ(vec, answer);

    // sort a list with only 1 item in it, for both pivot points
    vec = {1};
    answer = {1};

    vic::sorting::merge_sort(vec.begin(), vec.begin(), vec.end());
    EXPECT_EQ(vec, answer);

    vic::sorting::merge_sort(vec.begin(), vec.begin() + 1, vec.end());
    EXPECT_EQ(vec, answer);

    // sort the smallest list where work could be done
    vec = {2, 1};
    answer = {1, 2};
    vic::sorting::merge_sort(vec.begin(), vec.begin() + 1, vec.end());
    EXPECT_EQ(vec, answer);

    // make sure that sorting a sorted array returns the same vector, regardless of pivot
    vec.resize(100);
    std::iota(vec.begin(), vec.end(), 0);
    for(std::size_t i = 0; i < 100; i++)
    {
        auto copy = vec;
        vic::sorting::merge_sort(copy.begin(), copy.begin() + i, copy.end());
        EXPECT_EQ(copy, vec);
    }
}

TEST(Memory, MergeSortFailure)
{
    // check that merge sorting a vector that does not comply does not get stuck in an infinite loop
    // note: not really needed, I just wanted to see what the output would be.

    // sooo.. it turns out that the sorting algorithm still works, even if the subvectors are not sorted.
    // It is not going to be efficient though.

    std::vector<int> vec;
    vec.resize(10);
    std::iota(vec.begin(), vec.end(), 0);

    auto reversed = vec;
    std::reverse(reversed.begin(), reversed.end());
    for(std::size_t i = 0; i < 10; i++)
    {
        auto copy = reversed;
        vic::sorting::merge_sort(copy.begin(), copy.begin() + i, copy.end());
    }
}

TEST(Memory, MergeSortRotate)
{
    // test the case where we need to rotate
    std::vector<int> vec;
    vec.resize(10);
    std::iota(vec.begin(), vec.end(), 0);

    for(std::size_t i = 0; i < 10; i++)
    {
        auto copy = vec;

        // rotate to the right.
        std::rotate(copy.rbegin(), copy.rbegin() + i, copy.rend());

        vic::sorting::merge_sort(copy.begin(), copy.begin() + i, copy.end());

        EXPECT_EQ(copy, vec);
    }
}

TEST(Memory, MergeSortFunctor)
{
    // test merge_sort with a custom comparison operator
}

TEST(Memory, MergeSortBackInserter)
{
    // test merge_sort with back insertion into another vector

    std::vector<int> answer;
    answer.resize(10);
    std::iota(answer.begin(), answer.end(), 0);

    // make a shuffled copy
    auto values = answer;
    std::random_device rd;
    std::mt19937 g(1234);
    std::shuffle(values.begin(), values.end(), g);

    // sort two halves of the vector separately
    auto midpoint = values.begin() + 6;
    std::sort(values.begin(), midpoint);
    std::sort(midpoint, values.end());

    // use back insertion version of merge_sort
    std::vector<int> result;
    vic::sorting::merge_sort_back_insertion(values.begin(), midpoint, values.end(), std::back_inserter(result));
    EXPECT_EQ(result, answer);

    std::vector<int> result2;
    vic::sorting::merge_sort_back_insertion(answer.begin(), answer.begin(), answer.end(), std::back_inserter(result2));
    EXPECT_EQ(result2, answer);

    std::vector<int> result3;
    vic::sorting::merge_sort_back_insertion(answer.begin(), answer.end(), answer.end(), std::back_inserter(result3));
    EXPECT_EQ(result3, answer);
}

TEST(Memory, UnorderedFlatSet)
{
    std::set<uint32_t> stdSet;
    vic::memory::FlatSet<uint32_t> flatSet;
    vic::memory::UnorderedFlatSet<uint32_t> unorderedFlatSet;

    std::random_device rd;
    std::mt19937 g(1234);

    auto trueOrFalse = std::bind(std::uniform_int_distribution<>(0, 3), std::default_random_engine()); // bias towards inserting
    auto randomInt = std::bind(std::uniform_int_distribution<>(0, 1000000), std::default_random_engine());

    for(std::size_t i = 0; i < 1000; ++i)
    {
        const bool insertOrRemove = trueOrFalse();
        if(insertOrRemove) // insert
        {
            const uint32_t value = randomInt();
            stdSet.insert(value);
            flatSet.insert(value);
            unorderedFlatSet.insert(value);
        }
        else // remove
        {
            if(stdSet.empty())
                continue;
            // grab a random value from the set
            const std::size_t index = randomInt() % stdSet.size();
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

TEST(Memory, MapOverlap)
{
    using KeyType = uint32_t;
    struct A
    {
        KeyType a;
    };
    struct B
    {
        KeyType b;
    };

    std::map<KeyType, A> map1{{{1, {1}}, //
                               {3, {3}},
                               {5, {5}},
                               {7, {7}}}};
    std::map<KeyType, B> map2{{{2, {2}}, //
                               {3, {3}},
                               {4, {4}},
                               {7, {7}}}};

    std::set<KeyType> keys;
    for(const auto& [key, first, second] : vic::Overlap(map1, map2))
    {
        EXPECT_EQ(key, first.a);
        EXPECT_EQ(key, second.b);
        keys.insert(key);
    }

    EXPECT_EQ(keys, (std::set<KeyType>{3, 7}));
}