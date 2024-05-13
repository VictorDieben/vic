#include <vector>

#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/serialize/serialize.h"

#include "SerializeHelpers.h"

using namespace vic::serialize;

TEST(Serialize, Array)
{
    // trivial:
    EXPECT_TRUE(SerializeDeserialize(std::array<int, 0>{}));
    EXPECT_TRUE(SerializeDeserialize(std::array<int, 6>{1, 2, 3, 4, 5, 6}));

    // non-trivial
    static_assert(!ConceptResizeableRange<std::array<int, 2>>);
    static_assert(!ConceptResizeableRange<std::array<std::string, 2>>);
    static_assert(ConceptResizeableRange<std::string>);
    static_assert(ConceptResizeableRange<std::vector<int>>);

    EXPECT_TRUE(SerializeDeserialize(std::array<std::string, 2>{"abc", "def"}));
}

TEST(Serialize, Vector)
{
    EXPECT_TRUE(SerializeDeserialize(std::vector<int>{}));
    EXPECT_TRUE(SerializeDeserialize(std::vector<int>{1, 2, 3, 4, 5, 6}));

    // Check non-trivially copyable inner type
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
    static_assert(!std::is_trivially_copyable_v<std::map<char, int>>);
    static_assert(!std::is_trivial_v<std::map<char, int>>);
    static_assert(!std::contiguous_iterator<std::map<char, int>::iterator>);

    static_assert(!ConceptDataTrivial<std::map<char, int>>);

    EXPECT_TRUE(SerializeDeserialize(std::map<char, int>{{{'a', 1}, {'b', 2}}}));
    EXPECT_TRUE(SerializeDeserialize(std::map<int, MyStruct>{{{1, MyStruct{"name1", 1}}, //
                                                              {2, MyStruct{"name2", 2}}}}));
}

TEST(Serialize, DataTrivial)
{
    static_assert(ConceptDataTrivial<std::string>);
    static_assert(ConceptDataTrivial<std::vector<int>>);

    static_assert(!std::is_trivial_v<MyStruct>);
    static_assert(!ConceptDataTrivial<std::vector<MyStruct>>);

    static_assert(!ConceptDataTrivial<std::map<char, int>>);
}

TEST(Serialize, Set)
{
    EXPECT_TRUE(SerializeDeserialize(std::set<int>{}));

    EXPECT_TRUE(SerializeDeserialize(std::set<int>{1, 2, 4, 8}));

    EXPECT_TRUE(SerializeDeserialize(std::set<std::string>{"abc", "def", "ghi"}));
}

TEST(Serialize, Optional)
{
    std::optional<int> optInt = std::nullopt;
    EXPECT_TRUE(SerializeDeserialize(optInt));

    std::optional<int> optInt2 = 1;
    EXPECT_TRUE(SerializeDeserialize(optInt2));
}

TEST(Serialize, Pair)
{
    static_assert(vic::tuple_like<std::pair<char, int>>);
    static_assert(vic::tuple_like<std::pair<const char, int>>);
    static_assert(vic::tuple_like<std::pair<char, const int>>);
    static_assert(vic::tuple_like<std::pair<const char, const int>>);
    static_assert(vic::tuple_like<const std::pair<char, int>>);

    EXPECT_TRUE(SerializeDeserialize(std::pair{'a', 1}));
    EXPECT_TRUE(SerializeDeserialize(std::pair{std::string{"abc"}, std::string{"def"}}));
}

TEST(Serialize, Tuple)
{
    EXPECT_TRUE(SerializeDeserialize(std::tuple{'a', 1, 1.}));
    EXPECT_TRUE(SerializeDeserialize(std::tuple{std::string{"abc"}, std::string{"def"}, std::string{"ghi"}}));
}

TEST(Serialize, Variant)
{
    using MyVariant = std::variant<int, float>;
    static_assert(vic::ConceptVariant<MyVariant>);
    MyVariant myVariant = 1;
    EXPECT_TRUE(SerializeDeserialize(myVariant));

    MyVariant myVariant2 = 2.f;
    EXPECT_TRUE(SerializeDeserialize(myVariant2));
}

TEST(Serialize, Expected)
{
    using MyExpected = std::expected<int, std::string>;
    static_assert(vic::serialize::ConceptExpected<MyExpected>);
    MyExpected e1 = 1;
    EXPECT_TRUE(SerializeDeserialize(e1));
    MyExpected e2 = std::unexpected{"error"};
    EXPECT_TRUE(SerializeDeserialize(e2));
}

TEST(Serialize, Custom)
{
    EXPECT_TRUE(SerializeDeserialize(MyOtherStruct{1, 2})); // <- will be copied in 1 go
    EXPECT_TRUE(SerializeDeserialize(MyStruct{"name", 1})); // <- agregage
}

TEST(Serialize, Enum)
{
    EXPECT_TRUE(SerializeDeserialize(MyEnum::First)); //
}

TEST(Serialize, Many)
{
    std::vector<std::byte> buffer(1024); // one KiB

    const int a = 1;
    const float b = 2.f;
    const double c = 3.;
    const MyEnum d{MyEnum::Second};

    int na;
    float nb;
    double nc;
    MyEnum nd;

    auto span = std::span<std::byte>{buffer};
    EXPECT_TRUE(SerializeMany(span, a, b, c, d));

    EXPECT_TRUE(DeserializeMany(std::span<const std::byte>{buffer}, na, nb, nc, nd));

    EXPECT_EQ(a, na);
    EXPECT_EQ(b, nb);
    EXPECT_EQ(c, nc);
    EXPECT_EQ(d, nd);
}