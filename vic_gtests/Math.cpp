
#include "gtest/gtest.h"

#include "vic/utils/math.h"

#include "test_base.h"

#include <array>
#include <numeric>
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

TEST(Math, Stirling)
{
    // check end conditions
    EXPECT_EQ(Stirling(1, 1), 1);
    EXPECT_EQ(Stirling(10, 1), 1);
    EXPECT_EQ(Stirling(100, 1), 1);

    EXPECT_EQ(Stirling(1, 1), 1);
    EXPECT_EQ(Stirling(10, 10), 1);
    EXPECT_EQ(Stirling(100, 100), 1);

    // check some random values
    EXPECT_EQ(Stirling(5, 3), 25);
    EXPECT_EQ(Stirling(7, 5), 140);
    EXPECT_EQ(Stirling(10, 5), 42525);

    // todo: this function can be greatly sped up, by storing the last list of n
}

TEST(Math, Partition)
{
    static constexpr auto stirling_3_1 = Stirling(3, 1);
    static constexpr auto stirling_3_2 = Stirling(3, 2);
    static constexpr auto stirling_3_3 = Stirling(3, 3);

    static constexpr auto nPartitions3 = NumberOfPermutations<3>(); // todo: change name to Partitions

    EXPECT_EQ(stirling_3_1 + stirling_3_2 + stirling_3_3, nPartitions3);

    static constexpr auto partitions_5 = ConstructPartitions<5>();

    static constexpr uint64_t numberOfPermutations = NumberOfPermutations<10>();

    static constexpr auto permutations_8 = ConstructPartitions<8>();
    for(const auto& permutation : permutations_8)
        std::cout << permutation << std::endl;

    std::cout << "permutations(8): " << permutations_8.size() << std::endl;
}

TEST(Math, GCD_lookup)
{
    // Greatest common denominator
    // should be constexpr map lookup up to a point

    EXPECT_EQ(std::gcd(1, 1), 1);

    EXPECT_EQ(std::gcd(100, 10), 10);
    EXPECT_EQ(std::gcd(10, 100), 10);

    EXPECT_EQ(std::gcd(3, 9), 3);
    EXPECT_EQ(std::gcd(9, 3), 3);

    GCDLookup<uint64_t, 50, 50> lookup;

    EXPECT_EQ(lookup.GCD(9, 3), 3);
    EXPECT_EQ(lookup.GCD(3, 9), 3);
}

TEST(Math, LookupTable)
{
    LookupTable table([](const double x) -> double { return x * x; });
}