
#include "gtest/gtest.h"

#include "vic/utils/math.h"

#include <array>
#include <random>

using namespace vic;
using namespace vic::math;

TEST(Math, SetAssignment)
{
    const std::array<uint8_t, 4> setAssignment{0, 1, 2, 3};
    const auto value = SetAssingmentsToInteger<uint64_t>(setAssignment);

    std::array<uint8_t, 4> resultBuffer{};
    IntegerToSetAssingment<decltype(resultBuffer)>(value, resultBuffer);

    EXPECT_EQ(setAssignment, resultBuffer);
}