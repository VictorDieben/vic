#include "../pch.h"
#include "../test_base.h"

#include "vic/linalg/inverse.h"
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/traits.h"

#include "vic/linalg/tools.h"

#include <random>
#include <utility>

namespace vic
{
namespace linalg
{

TEST(TestLinalg, HappyFlow)
{
    constexpr Matrix<double, 4, 4> mat1{};
    constexpr Identity<double, 4> identity1{};
    constexpr Diagonal<double, 4, 4> diagonal1{};

    EXPECT_TRUE((HasSameType<decltype(mat1), decltype(identity1)>::value));
    EXPECT_TRUE((HasSameType<decltype(mat1), decltype(diagonal1)>::value));
    EXPECT_TRUE((HasSameType<decltype(diagonal1), decltype(identity1)>::value));

    EXPECT_FALSE((HasSameType<Matrix<float, 4, 4>, Matrix<double, 4, 4>>::value));
    EXPECT_FALSE((HasSameType<Identity<float, 4>, Identity<double, 4>>::value));
    EXPECT_FALSE((HasSameType<Diagonal<float, 4, 4>, Diagonal<double, 4, 4>>::value));
    EXPECT_FALSE((HasSameType<Matrix<float, 4, 4>, Identity<double, 4>>::value));
    EXPECT_FALSE((HasSameType<Matrix<float, 4, 4>, Diagonal<double, 4, 4>>::value));
}

TEST(TestLinalg, TestZeros)
{
    constexpr Zeros<double, 4, 4> zeros4x4{};
    for(std::size_t i = 0; i < 4; ++i)
        for(std::size_t j = 0; j < 4; ++j)
            EXPECT_DOUBLE_EQ(zeros4x4.Get(i, j), 0.);
    EXPECT_DEATH(zeros4x4.Get(4, 0), "");
    EXPECT_DEATH(zeros4x4.Get(0, 4), "");
}

TEST(TestLinalg, TestIdentity)
{
    constexpr Identity<double, 4> identity4{};

    // make sure we can evaluate identity as constexpr
    constexpr double get0x0 = identity4.Get(0, 0);
    EXPECT_DOUBLE_EQ(get0x0, 1.);
    constexpr double get1x0 = identity4.Get(1, 0);
    EXPECT_DOUBLE_EQ(get1x0, 0.);

    for(std::size_t i = 0; i < 4; ++i)
        for(std::size_t j = 0; j < 4; ++j)
            EXPECT_DOUBLE_EQ(identity4.Get(i, j), i == j ? 1. : 0.);

    EXPECT_DEATH(identity4.Get(4, 0), "");
    EXPECT_DEATH(identity4.Get(0, 4), "");
}

TEST(TestLinalg, TestMatrix)
{
    constexpr const Matrix<double, 3, 3> mat3({0, 1, 2, 3, 4, 5, 6, 7, 8});

    for(std::size_t i = 0; i < 3; ++i)
        for(std::size_t j = 0; j < 3; ++j)
            EXPECT_DOUBLE_EQ(mat3.Get(i, j), double(RowColToIndex<3>(i, j)));

    EXPECT_DEATH(mat3.Get(3, 0), "");
    EXPECT_DEATH(mat3.Get(0, 3), "");

    // initialize matrix from other types, make sure it is constexpr
    constexpr auto mat1 = Matrix<double, 4, 4>(Identity<double, 4>{});
    constexpr auto mat2 = Matrix<double, 4, 4>(Zeros<double, 4, 4>{});
    constexpr auto mat3b = Matrix<double, 4, 4>(Diagonal<double, 4, 4>({1, 2, 3, 4}));
}

TEST(TestLinalg, TestDiagonal)
{
    EXPECT_TRUE((ConceptSquareMatrix<Diagonal<double, 4, 4>>));

    // test default constructor (all 0)
    constexpr const Diagonal<double, 4, 4> zeros4{};
    for(std::size_t i = 0; i < 4; ++i)
        for(std::size_t j = 0; j < 4; ++j)
            EXPECT_DOUBLE_EQ(zeros4.Get(i, j), 0.);

    // test constructing from value
    constexpr const Diagonal<double, 4, 4> twos{2.};
    for(std::size_t i = 0; i < 4; ++i)
        for(std::size_t j = 0; j < 4; ++j)
            EXPECT_DOUBLE_EQ(twos.Get(i, j), i == j ? 2. : 0.);

    // test constructing from array
    constexpr const Diagonal<double, 5, 5> diag5({0, 1, 2, 3, 4});
    for(std::size_t i = 0; i < 5; ++i)
        for(std::size_t j = 0; j < 5; ++j)
            EXPECT_DOUBLE_EQ(diag5.Get(i, j), i == j ? double(i) : 0.);

    // test constructing from identity
    constexpr const Identity<double, 6> identity6{};
    constexpr const Diagonal<double, 6, 6> diag6{identity6};
    for(std::size_t i = 0; i < 6; ++i)
        for(std::size_t j = 0; j < 6; ++j)
            EXPECT_DOUBLE_EQ(diag6.Get(i, j), i == j ? 1. : 0.);

    // test constructing from full matrix (extract diagonal)
    constexpr const Matrix<double, 3, 3> mat3({0, 1, 2, 3, 4, 5, 6, 7, 8});
    constexpr const Diagonal<double, 3, 3> diag3(mat3);
    EXPECT_DOUBLE_EQ(diag3.Get(0, 0), 0.);
    EXPECT_DOUBLE_EQ(diag3.Get(1, 1), 4.);
    EXPECT_DOUBLE_EQ(diag3.Get(2, 2), 8.);
    for(std::size_t i = 0; i < 3; ++i)
        for(std::size_t j = 0; j < 3; ++j)
            if(i != j)
                EXPECT_DOUBLE_EQ(diag3.Get(i, j), 0.);
}

TEST(TestLinalg, TestAddMatrix)
{
    constexpr double value = 1.0;

    constexpr auto res1 = Add(Identity<double, 4>{}, value);
    constexpr auto res2 = Add(value, Identity<double, 4>{});
    for(std::size_t i = 0; i < 4; ++i)
    {
        for(std::size_t j = 0; j < 4; ++j)
        {
            EXPECT_DOUBLE_EQ(res1.Get(i, j), i == j ? 2. : 1.);
            EXPECT_DOUBLE_EQ(res2.Get(i, j), i == j ? 2. : 1.);
        }
    }

    constexpr const auto mat1 = Matrix<double, 2, 2>({1, 2, 3, 4});
    constexpr const auto mat2 = Matrix<double, 2, 2>({5, 6, 7, 8});
    constexpr const auto res = Add(mat1, mat2);
    ExpectMatrixEqual(res, Matrix<double, 2, 2>({6, 8, 10, 12}));
}

TEST(TestLinalg, TestAddConstant)
{
    constexpr Matrix<double, 4, 4> res1 = Add(Identity<double, 4>{}, Identity<double, 4>{});
    for(std::size_t i = 0; i < 4; ++i)
        for(std::size_t j = 0; j < 4; ++j)
            EXPECT_DOUBLE_EQ(res1.Get(i, j), i == j ? 2. : 0.);
}

TEST(TestLinalg, TestDeterminant)
{
    constexpr auto identityDetD = Determinant(Identity<double, 100>{});
    EXPECT_EQ(identityDetD, 1.0);
    EXPECT_TRUE((std::is_same_v<decltype(identityDetD), const double>));

    constexpr auto identityDetI = Determinant(Identity<int, 1000>{});
    EXPECT_EQ(identityDetI, 1);
    EXPECT_TRUE((std::is_same_v<decltype(identityDetI), const int>));

    constexpr Diagonal<double, 5, 5> diag5({1, 2, 3, 4, 5});
    constexpr auto diag5Det = Determinant(diag5);
    EXPECT_EQ(diag5Det, 1 * 2 * 3 * 4 * 5);

    constexpr Matrix<int, 3, 3> mat3({1, 2, 3, 4, 5, 6, 7, 8, 9});
    constexpr auto mat3Det = Determinant(mat3);
    EXPECT_TRUE((std::is_same_v<decltype(mat3Det), const int>));
    //EXPECT_EQ(mat3Det, ...);

    // should not compile:
    // Determinant(Matrix<int, 3, 4>{});
}

TEST(TestLinalg, TestInverseTranspose)
{
    constexpr Identity<double, 5> identity{};
    constexpr auto identityTranspose = Transpose(identity);
    ExpectMatrixEqual(identityTranspose, identity);
    constexpr auto identityInverse = Inverse(identity);
    ExpectMatrixEqual(identityInverse, identity);

    constexpr Diagonal<double, 5, 5> diag({1, 2, 3, 4, 5});
    constexpr auto transposeDiag = Transpose(diag);
    ExpectMatrixEqual(transposeDiag, diag);
    constexpr auto inverseDiag = Inverse(diag);
    constexpr Diagonal<double, 5, 5> inverseDiagAnswer({1., 1. / 2., 1. / 3., 1. / 4, 1. / 5.});
    ExpectMatrixEqual(inverseDiag, inverseDiagAnswer);

    constexpr Matrix<double, 3, 3> matrix3({1, 2, 3, 4, 5, 6, 7, 8, 9});
    constexpr auto inverseMat3 = Inverse(matrix3);
    constexpr Matrix<double, 3, 3> inverseMatrix3Answer({1, 2, 3, 4, 5, 6, 7, 8, 9});
    // ExpectMatrixEqual(inverseMat3, inverseMatrix3Answer);
    constexpr auto transposeMat3 = Transpose(matrix3);
    constexpr Matrix<double, 3, 3> transposeMatrix3Answer({1, 4, 7, 2, 5, 8, 3, 6, 9});
    ExpectMatrixEqual(transposeMat3, transposeMatrix3Answer);
}

TEST(TestLinalg, TestInverse)
{
    constexpr auto diag1 = Diagonal<double, 3, 3>({1, 2, 3});
    constexpr auto diagInv1 = InverseDiagonal(diag1);
    auto diagInv2 = InverseStatic(diag1);
    ExpectMatrixEqual(diagInv1, diagInv2);
}

TEST(TestLinalg, TestInverseRandom)
{
    std::default_random_engine g;
    std::uniform_real_distribution<double> dist(0.01, 100.);
    constexpr auto identity = Identity<double, 10>{};

    // test a bunch of random matrices
    for(std::size_t i = 0; i < 1000; ++i)
    {
        Matrix<double, 10, 10> matrix{};
        for(std::size_t r = 0; r < 10; ++r)
            for(std::size_t c = 0; c < 10; ++c)
                matrix.At(r, c) = dist(g);

        const auto inverse = InverseStatic(matrix);
        const auto result = Matmul(inverse, matrix);

        ExpectMatrixEqual(result, identity, 1E-6); // A^-1 * A == I
    }
}

TEST(TestLinalg, TestBracket3)
{
    constexpr Vector3<double> vec({1, 2, 3});
    constexpr auto bracket = Bracket3(vec);
    constexpr Matrix<double, 3, 3> answer({0, -3, 2, 3, 0, -1, -2, 1, 0});
    ExpectMatrixEqual(bracket, answer);
}

TEST(TestLinalg, TestBracket6)
{
    // TODO
}

TEST(TestLinalg, TestLieBracket)
{
    // TODO
}

} // namespace linalg
} // namespace vic