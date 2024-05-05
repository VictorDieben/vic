#include <vector>

#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/serialize/serialize.h"

using namespace vic::serialize;

// define some types for serialization/deserialization

enum class MyEnum : int
{
    First,
    Second
};

struct MyStruct
{
    std::string name;
    int age;
};
bool operator==(const MyStruct& a, const MyStruct& b) { return (a.name == b.name) && (a.age == b.age); }

struct MyOtherStruct
{
    int a;
    double b;
};
bool operator==(const MyOtherStruct& a, const MyOtherStruct& b) { return (a.a == b.a) && (a.b == b.b); }

struct MyNestedStruct
{
    MyStruct m1;
    MyOtherStruct m2;
};

template <typename T>
const auto serializeDeserialize = [](const T& item) -> bool {
    std::vector<std::byte> buffer(1024); // one KiB
    auto span = std::span<std::byte>{buffer};
    if(!Serialize(span, item))
        return false;
    T newItem;
    if(!Deserialize(std::span<const std::byte>{buffer}, newItem))
        return false;
    return item == newItem;
};

TEST(Serialize, Vector)
{
    EXPECT_TRUE(serializeDeserialize<std::vector<int>>({}));
    EXPECT_TRUE(serializeDeserialize<std::vector<int>>({1, 2, 3, 4, 5, 6}));
}

TEST(Serialize, String)
{
    EXPECT_TRUE(serializeDeserialize<std::string>(""));
    EXPECT_TRUE(serializeDeserialize<std::string>("Hello World!"));
    EXPECT_TRUE(serializeDeserialize<std::string>("`1234567890-+abcdefghijklmnopqrstuvwxyz ,./\n\t[]{}"));
}

TEST(Serialize, Map)
{
    //std::map<char, int> myMap{{{'a', 1}, {'b', 2}}};
    //EXPECT_TRUE(serializeDeserialize<decltype(myMap)>(myMap));
}

TEST(Serialize, Set)
{
    //std::set<int> mySet{1, 2, 4, 8};
    //EXPECT_TRUE(serializeDeserialize<decltype(mySet)>(mySet));
}

TEST(Serialize, Custom)
{
    MyOtherStruct s{1, 2};
    EXPECT_TRUE(serializeDeserialize<MyOtherStruct>(s)); //

    //MyStruct s1{"name", 1};
    //EXPECT_TRUE(serializeDeserialize<MyStruct>(s1)); //
}

TEST(Serialize, Many)
{
    std::vector<std::byte> buffer(1024); // one KiB

    const int a = 1;
    const float b = 2.f;
    const double c = 3.;
    // const MyEnum d{MyEnum::Second};

    int na;
    float nb;
    double nc;
    // MyEnum nd;

    auto span = std::span<std::byte>{buffer};
    EXPECT_TRUE(Serialize(span, a, b, c));

    EXPECT_TRUE(Deserialize(std::span<const std::byte>{buffer}, na, nb, nc));

    EXPECT_EQ(a, na);
    EXPECT_EQ(b, nb);
    EXPECT_EQ(c, nc);
    // EXPECT_EQ(d, nd);
}