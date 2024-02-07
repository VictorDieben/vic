
#include "gtest/gtest.h"

#include "../test_base.h"

#include "vic/linalg/linalg.h"

using namespace vic;
using namespace vic::linalg;

TEST(Linalg, Transpose)
{
    // Test the Shape, Constexpr-ness and value of Transpose()
    constexpr Identity2<double> identity2 = Transpose(Identity2<double>{});
    EXPECT_TRUE(IsEqual(identity2, Identity2<double>{}));

    constexpr auto diag = DiagonalMxN<double, 2, 2>(std::array{1., 2.});
    constexpr DiagonalMxN<double, 2, 2> diagT = Transpose(diag);
    EXPECT_TRUE(IsEqual(diag, diagT));

    constexpr MatrixMxN<double, 2, 2> mat = MatrixMxN<double, 2, 2>({1., 2., 3., 4.});
    constexpr MatrixMxN<double, 2, 2> matT = Transpose(mat);
    EXPECT_TRUE(IsEqual(matT, MatrixMxN<double, 2, 2>({1., 3., 2., 4.})));

    // test that Transpose can successfully deduce shape
    using RowConstType = detail::MatrixRowConst<double, Shape<2, UnknownSize>>;
    using ColConstType = detail::MatrixColConst<double, Shape<UnknownSize, 2>>;

    const RowConstType rowConst1 = To<RowConstType>(mat);
    const ColConstType colConst1 = Transpose(rowConst1);
    EXPECT_TRUE(IsEqual(colConst1, matT));

    const ColConstType colConst2 = To<ColConstType>(mat);
    const RowConstType rowConst2 = Transpose(colConst2);
    EXPECT_TRUE(IsEqual(rowConst2, matT));
}

TEST(Linalg, TransposeInverseDiag)
{
    constexpr auto diag = DiagonalMxN<double, 3, 3>({1., 2., 3.});
    constexpr DiagonalMxN<double, 3, 3> diagInv = InverseDiagonal(diag);
    EXPECT_TRUE(IsEqual(Matmul(diag, diagInv), Identity3<double>{}));
}