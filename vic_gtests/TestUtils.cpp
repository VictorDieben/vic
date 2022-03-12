#include "pch.h"

#include "vic/utils.h"

using namespace vic;

TEST(TestUtils, Startup)
{
    constexpr auto res = Pow<2>(3);
    EXPECT_EQ(res, 9);
}