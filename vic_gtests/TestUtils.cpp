#include "pch.h"

#include "test_base.h"
#include "vic/utils.h"
#include <limits>
#include <random>
#include <thread>

using namespace vic;

TEST(TestUtils, TestPow)
{
    constexpr auto res = Pow<2>(3); // check constexpr

    EXPECT_EQ(Pow<0>(3), 1);
    EXPECT_EQ(Pow<1>(3), 3);
    EXPECT_EQ(Pow<2>(3), 9);

    EXPECT_EQ(Pow<0>(10), 1);
    EXPECT_EQ(Pow<1>(10), 10);
    EXPECT_EQ(Pow<2>(10), 100);
    EXPECT_EQ(Pow<3>(10), 1000);

    const double tol = 1E-14;
    const double pi = 3.1415;
    EXPECT_NEAR(Pow<0>(pi), 1., tol);
    EXPECT_NEAR(Pow<3>(pi), pi * pi * pi, tol);
    EXPECT_NEAR(Pow<6>(pi), pi * pi * pi * pi * pi * pi, tol);
}

TEST(TestUtils, TestTimer)
{
    CTimer timer;
    timer.Reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const double interval_ms = timer.GetTime<double, std::milli>().count();
    EXPECT_LT(9., interval_ms);
    EXPECT_LT(interval_ms, 50.); // large upper bound, we don't know when thread returns
}

TEST(TestUtils, TestCounted)
{
    struct S : public Counted<S>
    { };

    {
        // construct / destruct
        EXPECT_EQ(S::GetCount(), 0);
        {
            S s1{};
            EXPECT_EQ(S::GetCount(), 1);
        }
        EXPECT_EQ(S::GetCount(), 0);
        S s2{};
        S s3 = s2; // copy
        EXPECT_EQ(S::GetCount(), 2);
        S s4 = std::move(s2);
        EXPECT_EQ(S::GetCount(), 3);
    }
    EXPECT_EQ(S::GetCount(), 0);
}

TEST(TestUtils, TestToBase)
{
    std::vector<int> buffer{};
    ToBase<int>(8, 2, buffer);
    ExpectVectorsEqual(buffer, std::vector<int>{0, 0, 0, 1});
}

TEST(TestUtils, TestFromBase)
{
    // base 2
    const uint16_t baseTwo{2};
    EXPECT_EQ(0, FromBase<uint64_t>(std::vector<int>{}, baseTwo));
    EXPECT_EQ(0, FromBase<uint64_t>(std::vector<int>{0}, baseTwo));
    EXPECT_EQ(1, FromBase<uint64_t>(std::vector<int>{1}, baseTwo));
    EXPECT_EQ(2, FromBase<uint64_t>(std::vector<int>{0, 1}, baseTwo));
    EXPECT_EQ(4, FromBase<uint64_t>(std::vector<int>{0, 0, 1}, baseTwo));
    EXPECT_EQ(8, FromBase<uint64_t>(std::vector<int>{0, 0, 0, 1}, baseTwo));
    EXPECT_EQ(8, FromBase<uint64_t>(std::vector<int>{0, 0, 0, 1, 0, 0, 0}, baseTwo));

    // make sure we trigger an assert if we try to convert a number that is too big.
    // (e.g. the value 10 in base 10 is no longer 1 digit)
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{2}, baseTwo), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{3}, 3), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{4}, 4), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{5}, 5), "");
    ASSERT_DEATH(FromBase<uint64_t>(std::vector<int>{6}, 6), "");
}

TEST(TestUtils, TestToFromBase)
{
    // pick a random value and base. convert value to that other base.
    // Then convert it back to normal representation
    using ValueType = uint64_t;
    using Basetype = uint16_t;

    std::default_random_engine generator;
    std::uniform_int_distribution<ValueType> valueDist(0, std::numeric_limits<ValueType>::max());
    std::uniform_int_distribution<Basetype> baseDist(2, std::numeric_limits<Basetype>::max());

    std::vector<Basetype> buffer{}; // used to write result to/from

    for(std::size_t i = 0; i < 10000; ++i)
    {
        const ValueType value = valueDist(generator);
        const Basetype base = baseDist(generator);

        ToBase(value, base, buffer);
        const auto fromBase = FromBase<ValueType>(buffer, base);
        EXPECT_EQ(value, fromBase);
    }
}