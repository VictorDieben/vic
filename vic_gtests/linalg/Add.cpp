

#include "gtest/gtest.h"

#include "vic/utils.h"

#include "vic/linalg/linalg.h"

#include "../test_base.h"

using namespace vic::linalg;

TEST(Linalg, AddSimple)
{
    constexpr const Identity2<double> i2{};
    constexpr const Diagonal2<double> d2 = Add(i2, i2);
    EXPECT_TRUE(IsEqual(d2, Matrix2<double>{{2., 0., 0., 2.}}));

    constexpr const Matrix2<double> m2{{1., 2., 3., 4.}};
    constexpr const Matrix2<double> res2 = Add(m2, m2);
    EXPECT_TRUE(IsEqual(res2, Matrix2<double>{{2., 4., 6., 8.}}));

    EXPECT_TRUE(IsEqual(Add(i2, 1.), Matrix2<double>{{2., 1., 1., 2.}}));

    EXPECT_TRUE(IsEqual(Add(m2, 1.), Matrix2<double>{{2., 3., 4., 5.}}));

    // verify mixed row-const/col-const
    const Matrix<double, Shape<2, UnknownSize>> c1{2, 2};
    const Matrix<double, Shape<UnknownSize, 2>> c2{2, 2};
    const Matrix<double, Shape<UnknownSize, UnknownSize>> d{2, 2};

    // only verify size, not values
    const Matrix2<double> c12 = Add(c1, c2);
    const Matrix2<double> c21 = Add(c2, c1);
    const Matrix<double, Shape<2, UnknownSize>> c1d = Add(c1, d);
    const Matrix<double, Shape<UnknownSize, 2>> c2d = Add(c2, d);
    const Matrix<double> dd = Add(d, d);

    // todo: sparse
}

TEST(Linalg, AddDiagonal)
{
    constexpr const Matrix2<double> m2{{1., 2., 3., 4.}};
    constexpr const Diagonal2<double> d2 = AddDiagonal(m2, m2); // note: only adds the diagonals
    EXPECT_TRUE(IsEqual(d2, Matrix2<double>{{2., 0., 0., 8.}}));
}
