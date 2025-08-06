
#include <gtest/gtest.h>

#include "test_base.h"

#include "vic/units/units.h"

using namespace vic::units;

template <typename T, typename U>
constexpr bool is_decayed_same_v = std::is_same_v<std::decay_t<T>, std::decay_t<U>>;

TEST(Units, HappyFlow)
{
    Length l1{1.};
    auto l2 = Length{2.};
    Length l5 = 5.;

    auto l3 = (l1 + l2); // todo: ideally i want to write Length l3 = ...; But gcc cannot deduce the type somehow
    l3 = 3.;
    l3 = -Length<double>{-3.};

    Area<double> a3 = l1 * l3;
    EXPECT_EQ((a3.Get()), 3.);

    Volume<double> v6 = a3 * l2;
    EXPECT_EQ(v6.Get(), 6.);

    using TInt = Length<int>;
    using TDouble = TInt::template SameType<double>;

    TDouble doubleLength{1.};
}

TEST(Units, Nested)
{
    constexpr auto volume = Length{1.} * Length{2.} * Length{3.};
    EXPECT_DOUBLE_EQ(volume.Get(), 6.);
    static_assert(is_decayed_same_v<decltype(volume), Volume<double>>);
}

TEST(Units, Multiplication)
{
    auto area = Length(2.) * Length(2.);
    EXPECT_DOUBLE_EQ(area.Get(), 4.);
    static_assert(is_decayed_same_v<decltype(area), Area<double>>);

    auto twiceDistance = Length{2.} * 2.;
    EXPECT_DOUBLE_EQ(twiceDistance.Get(), 4.);
    static_assert(is_decayed_same_v<decltype(twiceDistance), Length<double>>);
}

TEST(Units, Division)
{
    auto length = Area{5.} / Length{2.};
    EXPECT_DOUBLE_EQ(length.Get(), 2.5);
    static_assert(is_decayed_same_v<decltype(length), Length<double>>);

    auto halfArea = Area{5.} / 2.;
    EXPECT_DOUBLE_EQ(halfArea.Get(), 2.5);
    static_assert(is_decayed_same_v<decltype(halfArea), Area<double>>);

    // Division + Remainder. Remainder has different BPI unit than divisor
    const Area<int> area{5};
    const Length<int> len{2};

    const auto divisor = area / len;
    static_assert(is_decayed_same_v<decltype(divisor), Length<int>>);
    EXPECT_EQ(divisor.Get(), 2);

    const auto remainder = area % len; // Area{5} - (Length{2} * Lenght{2}) = Area{1};
    static_assert(is_decayed_same_v<decltype(remainder), Area<int>>);

    EXPECT_EQ(remainder.Get(), 1);
}

TEST(Units, Sqrt)
{
    Length<int> l2 = std::sqrt(Area(4));
    EXPECT_EQ(l2.Get(), 2);
}

TEST(Units, Cbrt)
{
    Length<int> length = std::cbrt(Volume{8});
    EXPECT_EQ(length.Get(), 2);
}

TEST(Units, Addition)
{
    Length<double> length = Length(2.) + 2.;
    EXPECT_DOUBLE_EQ(length.Get(), 4.);
    static_assert(is_decayed_same_v<decltype(length), Length<double>>);

    Area<double> area = Area(2.) + 2.;
    EXPECT_DOUBLE_EQ(area.Get(), 4.);
    static_assert(is_decayed_same_v<decltype(area), Area<double>>);
}

TEST(Units, TypeName)
{
    // constexpr auto name = Unitless<double>::TypeName(); // todo
    EXPECT_EQ(Length<double>::TypeName(), "BPI<0, 1, 0>");
    EXPECT_EQ(Area<double>::TypeName(), "BPI<0, 2, 0>");
    EXPECT_EQ(Volume<double>::TypeName(), "BPI<0, 3, 0>");
}

TEST(Units, MixTypes)
{
    // test adding different primitive types
    Length<double> d1{2.};
    Length<int> i1{2};

    Length<double> sum = d1 + i1;
    EXPECT_TRUE((std::is_same_v<decltype(sum.Get()), double>));

    Force<int> force = Mass{1} * Acceleration{1};
    Density<int> density = Mass{1} / Volume{1};
    MassFlow<int> massflow = Mass{1} / Time{1};
    Frequency<double> freq = 1. / Time{1};

    // s = 0.5 * a * t^2
    Length<double> distance = 0.5 * Acceleration{3.} * Time{2.} * Time{2.};
}

TEST(Units, Representations)
{
    // + operator
    Length<int> intPlusInt = Length<int>{} + Length<int>{};

    Length<double> intPlusDouble = Length<int>{} + Length<double>{};
    Length<double> doublePlusInt = Length<double>{} + Length<int>{};

    Length<long> longInt = Length<long>{} + Length<int>{};
    Length<long> intLong = Length<int>{} + Length<long>{};

    Length<int> intShort = Length<int>{} + Length<short>{};
    Length<int> shortInt = Length<short>{} + Length<int>{};
}

TEST(Units, ShouldNotComile)
{
    // a bunch of statements that should not compile

    // double freq = 1. / Time{1};
}

// // todo: verify linear algebra axioms