
#include "pch.h"
#include "test_base.h"
#include "vic/memory/dense_map.h"
#include "vic/memory/flat_map.h"
#include "vic/memory/merge_sort.h"
#include "vic/memory/refcounter.h"
#include "vic/memory/ring_buffer.h"

#include <algorithm>
#include <random>
#include <thread>
#include <vector>

namespace vic
{
namespace memory
{

TEST(TestMemory, RefCounted)
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

        // copy the RefCounter, acces data through it,
        // check that refcount is reset after it is destructed.
        {
            memory::RefCounted<TestStruct> copy{counted};
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

TEST(TestMemory, DenseMap)
{

    vic::memory::DenseMap<KeyType, TestStruct> map;

    map.Write();

    map.Insert(1, TestStruct{1});

    std::cout << "=== after insert: ===" << std::endl;

    map.Write();

    map.Insert(2, TestStruct{2});

    std::cout << "=== after insert 2: ===" << std::endl;
    map.Write();

    map.Insert(0, TestStruct{0});

    std::cout << "=== after insert 3: ===" << std::endl;
    map.Write();

    std::cout << "=== removing key 1 ===" << std::endl;
    // map.Remove(1);

    map.Write();

    ASSERT_TRUE(false);
}

TEST(TestMemory, FlatMap)
{
    struct MyType
    {
        int val;
    };
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
}

TEST(TestMemory, RingBuffer)
{
    struct TestStruct
    {
        int a;
        float b;
    };

    constexpr std::size_t capacity = 100;

    // verify empty buffer
    RingBuffer<TestStruct, capacity> buffer;
    ASSERT_EQ(buffer.Capacity(), capacity);
    ASSERT_EQ(buffer.Size(), 0);
    ASSERT_EQ(buffer.Pop(), std::nullopt);

    // put in 1 item, verify value
    buffer.Push({1, 1.});
    ASSERT_EQ(buffer.Size(), 1);
    auto item = buffer.Pop();
    ASSERT_NE(item, std::nullopt);
    EXPECT_EQ(item.value().a, 1);
    ASSERT_EQ(buffer.Size(), 0);

    // test with a separate push and pop thread
    const std::size_t n = 100000;

    std::thread pushThread([&]() {
        // push n items into buffer
        for(int i = 0; i < n; ++i)
            while(!buffer.Push({i, 0.}))
                std::this_thread::yield();
    });

    int previous = -1;
    std::thread popThread([&]() {
        // pop items, check that the index of a always increases
        while(true)
        {
            auto item = buffer.Pop();
            if(item == std::nullopt)
            {
                std::this_thread::yield();
                continue;
            }
            ASSERT_TRUE(item.value().a > previous);
            previous = item.value().a;
            if(previous == n - 1)
                break;

            std::this_thread::yield();
        }
    });

    pushThread.join();
    popThread.join();
    EXPECT_EQ(buffer.Size(), 0);
    EXPECT_EQ(previous, n - 1);
}

TEST(TestMemory, MergeSort)
{
    // construct a vector with increasing numbers
    std::vector<int> answer;
    for(int i = 0; i < 10; ++i)
        answer.push_back(i);

    // make a shuffled copy
    auto values = answer;
    std::random_device rd;
    std::mt19937 g(1234);
    std::shuffle(values.begin(), values.end(), g);

    // sort two halves of the vector separately
    auto midpoint = values.begin() + 6;
    std::sort(values.begin(), midpoint);
    std::sort(midpoint, values.end());

    for(const auto& val : values)
        std::cout << val << "; ";
    std::cout << std::endl;

    // now combine the two sorted halves
    vic::sorting::merge_sort(values.begin(), midpoint, values.end());

    // check values
    EXPECT_EQ(values, answer);
}

} // namespace memory
} // namespace vic