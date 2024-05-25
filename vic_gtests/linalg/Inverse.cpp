

#include "gtest/gtest.h"

#include "../test_base.h"

#include "vic/linalg/linalg.h"

#include <random>

using namespace vic::linalg;

TEST(Linalg, InverseDiagonal)
{
    constexpr auto diag1 = Diagonal3<double>({1, 2, 3});
    constexpr Diagonal3<double> diagInv1 = Inverse(diag1);
    EXPECT_TRUE(IsEqual(Matmul(diag1, diagInv1), Identity3<double>{}));
}

TEST(Linalg, InverseRandom)
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

#ifdef _DEBUG
    constexpr std::size_t n = 25;
#else
    constexpr std::size_t n = 100;
#endif

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

TEST(Linalg, Inverse2x2)
{
    static constexpr Matrix22d mat2x2{1, 2, 3, 4}; //
    static constexpr Matrix22d inv2x2 = Inverse2x2(mat2x2);

    EXPECT_TRUE(IsEqual(Matmul(mat2x2, inv2x2), Identity2d{}));
}

TEST(Linalg, Inverse3x3)
{
    static constexpr Matrix33d mat3x3{1, 2, -1, 2, 1, 2, -1, 2, 1}; //

    static constexpr double invdet = -1. / 16.;
    static constexpr Matrix33d inverseSolution{-3. * invdet, -4. * invdet, 5. * invdet, -4. * invdet, 0, -4. * invdet, 5. * invdet, -4. * invdet, -3. * invdet};

    const auto sol = Matmul(inverseSolution, mat3x3);
    EXPECT_TRUE(IsEqual(sol, Identity3d{}));

    static constexpr Matrix33d inv3x3 = Inverse3x3(mat3x3);

    EXPECT_TRUE(IsEqual(inv3x3, inverseSolution));

    static constexpr auto tmp = Matmul(mat3x3, inv3x3);
    EXPECT_TRUE(IsEqual(tmp, Identity3d{}));
}

TEST(Linalg, Inverse4x4)
{
    static constexpr Identity4d identity4{};

    static constexpr auto inverseIdentity = Inverse4x4(identity4);
    EXPECT_TRUE(IsEqual(identity4, inverseIdentity));

    static constexpr Matrix44d mat4x4{1, 2, 3, 4, 0, 6, 7, 8, 0, 0, 11, 12, 0, 0, 0, 16};
    static constexpr auto inverseMat4x4 = Inverse4x4(mat4x4);
    EXPECT_TRUE(IsEqual(identity4, Matmul(mat4x4, inverseMat4x4)));

    static constexpr Matrix44d mat4x4_2{1, 0, 0, 0, 5, 6, 0, 0, 9, 10, 11, 0, 13, 14, 15, 16};
    static constexpr auto inverseMat4x4_2 = Inverse4x4(mat4x4_2);
    EXPECT_TRUE(IsEqual(identity4, Matmul(mat4x4_2, inverseMat4x4_2)));
}