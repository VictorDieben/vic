#include "../test_base.h"
#include "pch.h"

#include "vic/linalg/linalg.h"

#include <random>

namespace vic
{
namespace linalg
{

TEST(TestLinalg, TestInverseDiagonal)
{
    constexpr auto diag1 = Diagonal3<double>({1, 2, 3});
    constexpr Diagonal3<double> diagInv1 = Inverse(diag1);
    EXPECT_TRUE(IsEqual(Matmul(diag1, diagInv1), Identity3<double>{}));
}

TEST(TestLinalg, TestInverseRandom)
{
    // NOTE: these numbers are inside gtest context, including construction of random matrix etc.
    // Not representative of actual performance

    // 50x50; 100 iters;    766 [ms]
    // 75x75; 100 iters;    2.7 [s]
    // 100x100; 100 iters;  6.8 [s]
    // 500x500; 1 iter;     12.5 [s]
    // 600x600; 1 iter;     16.5 [s]

    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(-.1, 0.1);
    std::uniform_real_distribution<double> diagDist(1., 5.);

    constexpr std::size_t n = 100;
    constexpr auto identity = IdentityN<double, n>{};

    // test a bunch of random matrices
    for(std::size_t i = 0; i < 100; ++i)
    {
        MatrixMxN<double, n, n> matrix{};
        for(Row r = 0; r < n; ++r)
            for(Col c = 0; c < n; ++c)
                matrix.At(r, c) = (r == c) ? diagDist(g) + dist(g) : dist(g);

        const auto inverse = InverseHotellingBodewig(matrix, 1E-10);
        const auto result = Matmul(inverse, matrix);

        EXPECT_TRUE(IsEqual(result, identity, 1E-8)); // A^-1 * A == I
    }
}

} // namespace linalg
} // namespace vic