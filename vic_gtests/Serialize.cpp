#include <vector>

#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/serialize/serialize.h"

#include "SerializeHelpers.h"

using namespace vic::serialize;

TEST(Serialize, Vector)
{
    EXPECT_TRUE(SerializeDeserialize(std::vector<int>{}));
    EXPECT_TRUE(SerializeDeserialize(std::vector<int>{1, 2, 3, 4, 5, 6}));

    // todo: check non-trivially copyable inner type
    static_assert(!std::is_trivially_copyable_v<std::string>);
    static_assert(!std::is_trivial_v<std::string>);
    EXPECT_TRUE(SerializeDeserialize(std::vector<std::string>{"abc", "def"}));
}

TEST(Serialize, String)
{
    EXPECT_TRUE(SerializeDeserialize(std::string{""}));
    EXPECT_TRUE(SerializeDeserialize(std::string{"Hello World!"}));
    EXPECT_TRUE(SerializeDeserialize(std::string{"`1234567890-+abcdefghijklmnopqrstuvwxyz ,./\n\t[]{}"}));
}

TEST(Serialize, Map)
{
    //std::map<char, int> myMap{{{'a', 1}, {'b', 2}}};
    //EXPECT_TRUE(SerializeDeserialize<decltype(myMap)>(myMap));
}

TEST(Serialize, Set)
{
    //std::set<int> mySet{1, 2, 4, 8};
    //EXPECT_TRUE(SerializeDeserialize<decltype(mySet)>(mySet));
}

TEST(Serialize, Optional)
{
    std::optional<int> optInt = std::nullopt;
    EXPECT_TRUE(SerializeDeserialize(optInt));

    std::optional<int> optInt2 = 1;
    EXPECT_TRUE(SerializeDeserialize(optInt2));
}

TEST(Serialize, Variant)
{
    //std::set<int> mySet{1, 2, 4, 8};
    //EXPECT_TRUE(SerializeDeserialize<decltype(mySet)>(mySet));
}

TEST(Serialize, Expected)
{
    //std::set<int> mySet{1, 2, 4, 8};
    //EXPECT_TRUE(SerializeDeserialize<decltype(mySet)>(mySet));
}

TEST(Serialize, Custom)
{
    EXPECT_TRUE(SerializeDeserialize(MyOtherStruct{1, 2})); // <- will be copied in 1 go
    EXPECT_TRUE(SerializeDeserialize(MyStruct{"name", 1})); // <- agregage
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