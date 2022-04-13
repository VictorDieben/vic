
#include "pch.h"
#include "test_base.h"

#include "vic/units/units.h"

namespace vic
{
namespace units
{

TEST(TestUnits, HappyFlow)
{
    Length l1{1.};
    auto l2 = Length{2.};

    Length l3 = l1 + l2;
    // static_assert(std::is_same_v<decltype(l3), Length<double>>);

    Area a3 = l1 * l3;
    EXPECT_DOUBLE_EQ(a3.Get(), 3.);
    // static_assert(std::is_same_v<decltype(a3), Area<double>>);

    Volume v6 = a3 * l2;
    EXPECT_DOUBLE_EQ(v6.Get(), 6.);
    // static_assert(std::is_same_v<decltype(v6), Volume<double>>);
}

TEST(TestUnits, Nested)
{
    constexpr auto volume = Length{1.} * Length{2.} * Length{3.};
    EXPECT_DOUBLE_EQ(volume.Get(), 6.);
    // static_assert(std::is_same_v<decltype(volume), Volume<double>>);
}

TEST(TestUnits, Division)
{
    auto value = Area{5.} / Length{2.};
    EXPECT_DOUBLE_EQ(value.Get(), 2.5);
    // static_assert(std::is_same_v<decltype(value), Length<double>>);
}

TEST(TestUnits, MixTypes)
{
    // test adding different primitive types
    Length<double> d1{2.};
    Length<int> i1{2};

    auto sum = d1 + i1;
    // static_assert(std::is_same_v<decltype(sum), Length<double>>);

    auto division = d1 / i1;
    // static_assert(std::is_same_v<decltype(division), Unitless<double>>);
}

} // namespace units
} // namespace vic