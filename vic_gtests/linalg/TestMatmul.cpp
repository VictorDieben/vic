#include "../pch.h"
#include "../test_base.h"

#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"
#include "vic/utils.h"

#include "vic/linalg/tools.h"

namespace vic
{
namespace linalg
{

TEST(TestLinalg, TestMatmulScalar)
{
    constexpr Diagonal<double, 2, 2> diag2 = Matmul(Identity<double, 2>{}, 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(0, 0), 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(1, 1), 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(0, 1), 0.);
    EXPECT_DOUBLE_EQ(diag2.Get(1, 0), 0.);
}

TEST(TestLinalg, TestMatmul)
{
    EXPECT_DOUBLE_EQ(6., Matmul(2., 3.));

    constexpr const auto res1 = Matmul(Identity<double, 3>{}, Identity<double, 3>{});
    ExpectMatrixEqual(res1, Identity<double, 3>{});

    constexpr const auto diag = Diagonal<double, 3, 3>({1, 2, 3});
    constexpr const auto res2 = Matmul(diag, Identity<double, 3>{});
    constexpr const auto res3 = Matmul(Identity<double, 3>{}, diag);
    ExpectMatrixEqual(res2, diag);
    ExpectMatrixEqual(res3, diag);

    constexpr const auto mat1 = Matrix<double, 2, 2>({1, 2, 3, 4});
    constexpr const auto mat2 = Matrix<double, 2, 2>({5, 6, 7, 8});
    constexpr const auto res4 = Matmul(mat1, mat2);
    ExpectMatrixEqual(res4, Matrix<double, 2, 2>({19, 22, 43, 50}));

    constexpr const auto diag1 = Diagonal<double, 2, 2>{mat1};
    constexpr const auto diag2 = Diagonal<double, 2, 2>{mat2};
    constexpr const auto res5 = Matmul(diag1, diag2);
    ExpectMatrixEqual(res5, Matrix<double, 2, 2>({5, 0, 0, 32}));

    constexpr const auto res6 = Matmul(diag1, mat2);
    ExpectMatrixEqual(res6, Matrix<double, 2, 2>({5, 6, 28, 32}));
    constexpr const auto res7 = Matmul(mat1, diag2);
    ExpectMatrixEqual(res7, Matrix<double, 2, 2>({5, 16, 15, 32}));

    constexpr const auto scalar1 = Matmul(mat1, 2.0);
    ExpectMatrixEqual(scalar1, Matrix<double, 2, 2>({2, 4, 6, 8}));
    constexpr const auto scalar2 = Matmul(3., mat2);
    ExpectMatrixEqual(scalar2, Matrix<double, 2, 2>({15, 18, 21, 24}));

    // verify types
    EXPECT_TRUE((std::is_same_v<double, decltype(Matmul(Identity<double, 1>{}, Identity<double, 1>{}))::DataType>));
    EXPECT_TRUE((std::is_same_v<double, decltype(Matmul(Identity<double, 1>{}, Identity<float, 1>{}))::DataType>));
    EXPECT_TRUE((std::is_same_v<double, decltype(Matmul(Identity<double, 1>{}, Identity<int, 1>{}))::DataType>));
    EXPECT_TRUE((std::is_same_v<int, decltype(Matmul(Identity<int, 1>{}, Identity<int, 1>{}))::DataType>));
}

TEST(TestLinalg, TestMatmulDynamic)
{
    // initialize
    const MatrixDynamic<double> dyn1{2, 3};
    const MatrixDynamic<double> dyn2{3, 4};
    const MatrixDynamic<double> dyn3{4, 5};

    // check sizes
    const auto matmul1 = Matmul(dyn1, dyn2);
    ASSERT_TRUE((matmul1.GetRows() == 2 && matmul1.GetColumns() == 4));
    const auto matmul2 = Matmul(dyn2, dyn3);
    ASSERT_TRUE(matmul2.GetRows() == 3 && matmul2.GetColumns() == 5);
    const auto matmul12_3 = Matmul(matmul1, dyn3);
    ASSERT_TRUE(matmul12_3.GetRows() == 2 && matmul12_3.GetColumns() == 5);
    const auto matmul12_23 = Matmul(dyn1, matmul2);
    ASSERT_TRUE(matmul12_23.GetRows() == 2 && matmul12_23.GetColumns() == 5);

    // try to evaluate some indices
    EXPECT_EQ(0, dyn1.Get(0, 0));
    EXPECT_EQ(0, dyn1.Get(1, 0));
    EXPECT_EQ(0, dyn1.Get(1, 2));
    EXPECT_DEATH(dyn1.Get(1, 3), "invalid index");
    EXPECT_DEATH(dyn1.Get(2, 2), "invalid index");

    // todo: verify values
}

TEST(TestLinalg, TestMatmulMixed)
{
    // test several series of multiplications, make sure the result is static size if it can be
    const auto mat2x3 = Matrix<double, 2, 3>{};
    const auto dyn3x4 = MatrixDynamic<double>{3, 4};
    const auto mat4x5 = Matrix<double, 4, 5>{};

    //const MatrixRowConst<double, 2> mixed2x4d = Matmul(mat2x3, dyn3x4);
    //const MatrixColConst<double, 2> mixed2x4d = Matmul(mat2x3, dyn3x4);

    auto mat2x5_1 = Matmul(Matmul(mat2x3, dyn3x4), mat4x5);
    auto mat2x5_2 = Matmul(mat2x3, Matmul(dyn3x4, mat4x5));
    // TODO(vicdie): the two mat2x5 matrices should be static size

    ExpectMatrixEqual(mat2x5_1, mat2x5_2); // TODO(vicdie): just zeros now

    // check that if the inbetween dimension is variable, but the two outside edges are static,
    // the resulting shape is still static
    const MatrixRowConst<double, 3> rowconst3x100(100);
    const MatrixColConst<double, 5> colconst100x5(100);
    const Matrix<double, 3, 5> static3x5 = Matmul(rowconst3x100, colconst100x5);
}

TEST(TestLinalg, TestMultivariate)
{
    // test multivariate
    constexpr auto M = DiagonalConstant<double, 3>(2);
    constexpr auto ans3 = Matmul(M, M, M);
    constexpr auto ans4 = Matmul(M, M, M, M);
    constexpr auto ans5 = Matmul(M, M, M, M, M);
    constexpr auto ans6 = Matmul(M, M, M, M, M, M);
    constexpr auto ans7 = Matmul(M, M, M, M, M, M, M);
    ExpectMatrixEqual(ans7, DiagonalConstant<double, 3>(Pow<7>(2.)));
}
} // namespace linalg
} // namespace vic