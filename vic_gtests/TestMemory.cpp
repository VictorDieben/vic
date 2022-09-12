
#include "pch.h"
#include "test_base.h"
#include "vic/memory/dense_map.h"
#include "vic/memory/flat_map.h"
#include "vic/memory/merge_sort.h"
#include "vic/memory/refcounter.h"

#include <algorithm>
#include <random>
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
    //ASSERT_EQ(map.count(0), 0);
    //ASSERT_EQ(map.count(5), 1);
    map[4] = {2};

    map[5] = {3};
    ASSERT_FALSE(map.empty());
    ASSERT_EQ(map.size(), 2);
    ASSERT_EQ(map.count(0), 0);
    ASSERT_EQ(map.count(5), 1);

    map.erase(5);
    ASSERT_EQ(map.size(), 1);


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