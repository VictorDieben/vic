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
    std::chrono::duration interval = timer.GetTime();
}