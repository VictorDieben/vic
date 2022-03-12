#include "pch.h"

#include "vic/utils.h"

using namespace vic;

TEST(TestUtils, Startup)
{
    constexpr auto res = Pow<2>(3);
    EXPECT_EQ(res, 9);
}

TEST(TestUtils, TestTimer)
{
    CTimer timer;
    timer.Reset();
    // std::chrono::duration interval = timer.GetTime();
}

TEST(TestUtils, TestToBase)
{
    auto res1 = ToBase<int, int, int>(8, 2); // 0, 0, 0, 1
    auto res2 = ToBase<int, int, int>(9, 3); // 0, 1
}

TEST(TestUtils, TestFromBase)
{
    // base 2
    EXPECT_EQ(0, (FromBase<size_t, size_t, size_t>({0}, 2)));
    EXPECT_EQ(1, (FromBase<size_t, size_t, size_t>({1}, 2)));
    EXPECT_EQ(2, (FromBase<size_t, size_t, size_t>({0, 1}, 2)));
    EXPECT_EQ(4, (FromBase<size_t, size_t, size_t>({0, 0, 1}, 2)));
    EXPECT_EQ(8, (FromBase<size_t, size_t, size_t>({0, 0, 0, 1}, 2)));
    EXPECT_EQ(8, (FromBase<size_t, size_t, size_t>({0, 0, 0, 1, 0, 0, 0}, 2)));

    // TODO(vicdie): some other bases
}

TEST(TestUtils, TestToFromBase)
{
    for(std::size_t i = 0; i < 1000; ++i)
    {
        // value = rnd()
        // base = rnd()

        // toBase = ToBase(value, base)
        // fromBase = FromBase(toBase, base)
        // EXPECT_EQ(value, fromBase);
    }
}