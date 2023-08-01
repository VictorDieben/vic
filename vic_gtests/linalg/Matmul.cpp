#include "gtest/gtest.h"

#include "../test_base.h"

#include "vic/utils.h"

#include "vic/linalg/linalg.h"

using namespace vic::linalg;

TEST(Matmul, TestMatmulScalar)
{
    constexpr Diagonal2<double> diag2 = Matmul(Identity2<double>{}, 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(0, 0), 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(0, 0), 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(1, 1), 2.);
    EXPECT_DOUBLE_EQ(diag2.Get(0, 1), 0.);
    EXPECT_DOUBLE_EQ(diag2.Get(1, 0), 0.);

    constexpr Diagonal2<double> diag2b = Matmul(2., Identity2<double>{});
    EXPECT_DOUBLE_EQ(diag2b.Get(0, 0), 2.);
    EXPECT_DOUBLE_EQ(diag2b.Get(1, 1), 2.);
    EXPECT_DOUBLE_EQ(diag2b.Get(0, 1), 0.);
    EXPECT_DOUBLE_EQ(diag2b.Get(1, 0), 0.);
}

TEST(Matmul, TestMatmul)
{
    EXPECT_DOUBLE_EQ(6., Matmul(2., 3.));

    constexpr const auto res1 = Matmul(Identity3<double>{}, Identity3<double>{});
    ExpectMatrixEqual(res1, Identity3<double>{});

    constexpr const auto diag = Diagonal3<double>({1, 2, 3});
    constexpr const auto res2 = Matmul(diag, Identity3<double>{});
    constexpr const auto res3 = Matmul(Identity3<double>{}, diag);
    ExpectMatrixEqual(res2, diag);
    ExpectMatrixEqual(res3, diag);

    constexpr const auto mat1 = Matrix2<double>({1, 2, 3, 4});
    constexpr const auto mat2 = Matrix2<double>({5, 6, 7, 8});
    constexpr const auto res4 = Matmul(mat1, mat2);
    ExpectMatrixEqual(res4, Matrix2<double>({19, 22, 43, 50}));

    constexpr const auto diag1 = ToDiagonal(mat1);
    constexpr const auto diag2 = ToDiagonal(mat2);
    constexpr const auto res5 = Matmul(diag1, diag2);
    ExpectMatrixEqual(res5, Matrix2<double>({5, 0, 0, 32}));

    constexpr const auto res6 = Matmul(diag1, mat2);
    ExpectMatrixEqual(res6, Matrix2<double>({5, 6, 28, 32}));
    constexpr const auto res7 = Matmul(mat1, diag2);
    ExpectMatrixEqual(res7, Matrix2<double>({5, 16, 15, 32}));

    constexpr const auto scalar1 = Matmul(mat1, 2.0);
    ExpectMatrixEqual(scalar1, Matrix2<double>({2, 4, 6, 8}));
    constexpr const auto scalar2 = Matmul(3., mat2);
    ExpectMatrixEqual(scalar2, Matrix2<double>({15, 18, 21, 24}));

    // verify DataTypes
    EXPECT_TRUE((std::is_same_v<double, decltype(Matmul(Identity1<double>{}, Identity1<double>{}))::DataType>));
    EXPECT_TRUE((std::is_same_v<double, decltype(Matmul(Identity1<double>{}, Identity1<float>{}))::DataType>));
    EXPECT_TRUE((std::is_same_v<double, decltype(Matmul(Identity1<double>{}, Identity1<int>{}))::DataType>));
    EXPECT_TRUE((std::is_same_v<int, decltype(Matmul(Identity1<int>{}, Identity1<int>{}))::DataType>));
}

TEST(Matmul, TestMatmulDynamic)
{
    // initialize
    const Matrix<double, UnknownShape> dyn1{2, 3};
    const Matrix<double, UnknownShape> dyn2{3, 4};
    const Matrix<double, UnknownShape> dyn3{4, 5};

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

    // todo: verify values

    // verify that the dynamic matmul can be called with static matrices

    using Tstatic = Matrix3<double>;
    const auto mat3x3 = Tstatic{};
    auto res = Matmul(mat3x3, mat3x3);
    (void)res;

    const auto dyn3x3 = Matrix<double, UnknownShape>{3, 3};
    auto res2 = Matmul(dyn3x3, dyn3x3);
}

TEST(Matmul, TestMatmulMixed)
{
    // test several series of multiplications, make sure the result is static size if it can be
    const auto mat2x3 = Matrix<double, Shape<2, 3>>{};
    const auto dyn3x4 = Matrix<double, UnknownShape>{3, 4};
    const auto mat4x5 = Matrix<double, Shape<4, 5>>{};

    const Matrix<double, Shape<2, UnknownSize>> mixed2x4d_1 = Matmul(mat2x3, dyn3x4);
    const Matrix<double, Shape<UnknownSize, 5>> mixed2x4d_2 = Matmul(dyn3x4, mat4x5);

    const Matrix<double, Shape<2, 5>> mat2x5_1 = Matmul(Matmul(mat2x3, dyn3x4), mat4x5);
    const Matrix<double, Shape<2, 5>> mat2x5_2 = Matmul(mat2x3, Matmul(dyn3x4, mat4x5));
    // TODO: the two mat2x5 matrices should be static size

    ExpectMatrixEqual(mat2x5_1, mat2x5_2); // TODO: just zeros now

    // check that if the inbetween dimension is variable, but the two outside edges are static,
    // the resulting shape is still static
    const Matrix<double, Shape<3, UnknownSize>> rowconst3x100(3, 100);
    const Matrix<double, Shape<UnknownSize, 5>> colconst100x5(100, 5);
    const Matrix<double, Shape<3, 5>> static3x5 = Matmul(rowconst3x100, colconst100x5);
}

TEST(Matmul, TestMultivariate)
{
    // test multivariate
    constexpr auto M = Add(Identity3<double>{}, Identity3<double>{});
    constexpr auto ans3 = Matmul(M, M, M);
    constexpr auto ans4 = Matmul(M, M, M, M);
    constexpr auto ans5 = Matmul(M, M, M, M, M);
    constexpr auto ans6 = Matmul(M, M, M, M, M, M);
    constexpr auto ans7 = Matmul(M, M, M, M, M, M, M);
    ExpectMatrixEqual(ans7, Matmul(Pow<7>(2.), Identity3<double>{}));
}

TEST(Matmul, TestMatmulSparse)
{
    // todo: specialize Matmul for sparse*vec
    Matrix2<double> mat{{0, 1, 2, 3}};
    const auto sparse2x2 = ToSparse(mat);

    ExpectMatrixEqual(mat, sparse2x2);

    const Vector2<double> vec{{5, 6}};

    const auto res1 = Matmul(mat, vec);
    const auto res2 = Matmul(sparse2x2, vec);

    ExpectMatrixEqual(res1, res2);
}

TEST(Matmul, TestMatmulStack)
{
    for(uint32_t i = 0; i < 10; ++i)
    {
        auto mat1 = Matrix3<double>{};
        auto mat2 = Matrix3<double>{};
        auto vec3 = Vector3<double>{};
        auto vec6 = Vector6<double>{};

        RandomFill(mat1);
        RandomFill(mat2);
        RandomFill(vec3);
        RandomFill(vec6);

        { // test rowstack
            const auto rowstack = ToRowStack(mat1, mat2);
            const auto res1 = MatmulFull(rowstack, vec3);
            const auto res2 = MatmulRowStack(rowstack, vec3);
            EXPECT_TRUE(IsEqual(res1, res2));
        }

        { // test colstack
            const auto colstack = ToColStack(mat1, mat2);
            const auto res1 = MatmulFull(colstack, vec6);
            const auto res2 = MatmulColStack(colstack, vec6);
            EXPECT_TRUE(IsEqual(res1, res2));
        }
    }
}

TEST(Matmul, TestMatmulTriDiagonal)
{
    constexpr auto test = MatmulTriDiagVector(ToTriDiagonal(Identity6<double>{}), Vector6<double>{});

    auto mat6 = Matrix6<double>{};
    RandomFill(mat6);

    // convert matrix to tri-diag, then back to full to extract tri-diagonal
    auto tri6 = ToTriDiagonal(mat6);
    auto mat6_tri = ToFull(tri6);

    EXPECT_TRUE(IsEqual(tri6, mat6_tri));

    auto vec6 = Vector6<double>{};
    RandomFill(vec6);

    const auto res1 = MatmulTriDiagVector(tri6, vec6);
    const auto res2 = MatmulFull(mat6_tri, vec6);
    EXPECT_TRUE(IsEqual(res1, res2));
}