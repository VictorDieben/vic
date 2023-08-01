
#include "gtest/gtest.h"

#include "../test_base.h"

#include "vic/linalg/linalg.h"
#include "vic/linalg/tools.h"

#include <array>

#include <random>

using namespace vic::linalg;

TEST(Linalg, RowSplit)
{
    static constexpr const Vector4<double> vec4{{1, 2, 3, 4}};

    // static constexpr const auto part1_part2 = RowSplit<2, 2>(vec4);

    {
        const auto [part1, part2] = RowSplit<2, 2>(vec4);
        EXPECT_TRUE(IsEqual(part1, Vector2<double>{1, 2}));
        EXPECT_TRUE(IsEqual(part2, Vector2<double>{3, 4}));
    }

    {
        const auto [part1, part2] = RowSplit<UnknownSize, 2>(vec4);
        EXPECT_TRUE(IsEqual(part1, Vector2<double>{1, 2}));
        EXPECT_TRUE(IsEqual(part2, Vector2<double>{3, 4}));
    }

    {
        const auto [part1, part2] = RowSplit<2, UnknownSize>(vec4);
        EXPECT_TRUE(IsEqual(part1, Vector2<double>{1, 2}));
        EXPECT_TRUE(IsEqual(part2, Vector2<double>{3, 4}));
    }
}