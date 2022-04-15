
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
    Length<double> l5 = 5.;

    Length l3 = l1 + l2;
    l3 = 3.; // test assigning a value
    l3 = -Length<double>{-3.};

    Area a3 = l1 * l3;
    EXPECT_DOUBLE_EQ(a3.Get(), 3.);

    Volume v6 = a3 * l2;
    EXPECT_DOUBLE_EQ(v6.Get(), 6.);
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

TEST(TestUnits, TypeName)
{
    // constexpr auto name = Unitless<double>::TypeName(); // todo
    EXPECT_EQ(Length<double>::TypeName(), "BPI<0, 1, 0>");
    EXPECT_EQ(Area<double>::TypeName(), "BPI<0, 2, 0>");
    EXPECT_EQ(Volume<double>::TypeName(), "BPI<0, 3, 0>");
}

TEST(TestUnits, MixTypes)
{
    // test adding different primitive types
    Length<double> d1{2.};
    Length<int> i1{2};

    auto sum = d1 + i1;
    // static_assert(std::is_same_v<decltype(sum), Length<double>>);

    Force<double> force = Mass<double>{} * Acceleration<double>{};
    Density<double> density = Mass<double>{} / Volume<double>{};
    MassFlow<double> massflow = Mass<double>{} / Time<double>{};
    Frequency<double> freq = Unitless<double>{} / Time<double>{};

    Length<double> distance = Unitless<double>{0.5} * Acceleration<double>{3.} * Time<double>{2.} * Time<double>{2.};
}

TEST(TestUnits, Representations)
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

template <typename T>
using PendulumPhisics = BPIFundamentalPhysics<Time<T>, Mass<T>, Length<T>, Acceleration<T>>;

TEST(TestUnits, BPIPendulum)
{
    // https://en.wikipedia.org/wiki/Buckingham_%CF%80_theorem
    PendulumPhisics<double> physics{};
}

} // namespace units
} // namespace vic