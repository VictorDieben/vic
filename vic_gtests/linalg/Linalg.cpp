
#include "gtest/gtest.h"

#include "../test_base.h"

#include "vic/linalg/algorithms/add.h"
#include "vic/linalg/algorithms/matmul.h"
#include "vic/linalg/definitions.h"
#include "vic/linalg/index.h"
#include "vic/linalg/traits.h"

// #include "vic/linalg/tools.h"

#include <random>

using namespace vic::linalg;

template <typename T, typename TShape>
using TestTypeSelector = TypeSelector<TShape, //
                                      int,
                                      uint32_t,
                                      float,
                                      double>;

TEST(Linalg, Definitions)
{
    EXPECT_EQ(1u, Row{1u});
    EXPECT_EQ(1u, Col{1u});

    EXPECT_TRUE((std::is_same_v<TestTypeSelector<double, Shape<3, 3>>, int>));
    EXPECT_TRUE((std::is_same_v<TestTypeSelector<double, Shape<3, UnknownSize>>, uint32_t>));
    EXPECT_TRUE((std::is_same_v<TestTypeSelector<double, Shape<UnknownSize, 3>>, float>));
    EXPECT_TRUE((std::is_same_v<TestTypeSelector<double, Shape<UnknownSize, UnknownSize>>, double>));
}

TEST(Linalg, Indices)
{
    std::default_random_engine g;
    std::uniform_int_distribution<> randomInt(0, 100);

    for(std::size_t n = 0; n < 100; ++n)
    {
        const auto i = randomInt(g);
        const auto j = randomInt(g);
        // test row mayor
        const auto c = Col{randomInt(g) + 100u};
        const auto idx = RowMayorRowColToIndex(i, j, c);
        const auto [ni, nj] = RowMayorIndexToRowCol(idx, c);
        EXPECT_EQ(i, ni);
        EXPECT_EQ(j, nj);
        // test col mayor
    }
}

//TEST(Linalg, HappyFlow)
//{
//    constexpr Matrix<double, 4, 4> mat1{};
//    constexpr Identity<double, 4> identity1{};
//    constexpr Diagonal<double, 4, 4> diagonal1{};
//
//    EXPECT_TRUE((is_same_type_v<decltype(mat1), decltype(identity1)>));
//    EXPECT_TRUE((is_same_type_v<decltype(mat1), decltype(diagonal1)>));
//    EXPECT_TRUE((is_same_type_v<decltype(diagonal1), decltype(identity1)>));
//
//    EXPECT_FALSE((is_same_type_v<Matrix<float, 4, 4>, Matrix<double, 4, 4>>));
//    EXPECT_FALSE((is_same_type_v<Identity<float, 4>, Identity<double, 4>>));
//    EXPECT_FALSE((is_same_type_v<Diagonal<float, 4, 4>, Diagonal<double, 4, 4>>));
//    EXPECT_FALSE((is_same_type_v<Matrix<float, 4, 4>, Identity<double, 4>>));
//    EXPECT_FALSE((is_same_type_v<Matrix<float, 4, 4>, Diagonal<double, 4, 4>>));
//}
//
//TEST(Linalg, TestZeros)
//{
//    constexpr Zeros<double, 4, 4> zeros4x4{};
//    for(std::size_t i = 0; i < 4; ++i)
//        for(std::size_t j = 0; j < 4; ++j)
//            EXPECT_DOUBLE_EQ(zeros4x4.Get(i, j), 0.);
//    EXPECT_DEATH(zeros4x4.Get(4, 0), "");
//    EXPECT_DEATH(zeros4x4.Get(0, 4), "");
//}
//
//TEST(Linalg, TestIdentity)
//{
//    static_assert(ConceptIdentity<Identity<double, 1>>);
//    static_assert(ConceptIdentity<Identity<double, 4>>);
//    static_assert(ConceptIdentity<Identity<double, 5>>);
//
//    constexpr Identity<double, 4> identity4{};
//
//    // make sure we can evaluate identity as constexpr
//    constexpr double get0x0 = identity4.Get(0, 0);
//    EXPECT_DOUBLE_EQ(get0x0, 1.);
//    constexpr double get1x0 = identity4.Get(1, 0);
//    EXPECT_DOUBLE_EQ(get1x0, 0.);
//
//    for(std::size_t i = 0; i < 4; ++i)
//        for(std::size_t j = 0; j < 4; ++j)
//            EXPECT_DOUBLE_EQ(identity4.Get(i, j), i == j ? 1. : 0.);
//
//    EXPECT_DEATH(identity4.Get(4, 0), "");
//    EXPECT_DEATH(identity4.Get(0, 4), "");
//}
//
//TEST(Linalg, TestMatrix)
//{
//    constexpr const Matrix<double, 3, 3> all2s(2.);
//    for(std::size_t i = 0; i < 3; ++i)
//        for(std::size_t j = 0; j < 3; ++j)
//            EXPECT_DOUBLE_EQ(all2s.Get(i, j), 2.);
//
//    constexpr const Matrix<double, 3, 3> mat3({0, 1, 2, 3, 4, 5, 6, 7, 8});
//
//    EXPECT_DOUBLE_EQ(mat3.Get(0, 2), 2.);
//    for(std::size_t i = 0; i < 3; ++i)
//        for(std::size_t j = 0; j < 3; ++j)
//            EXPECT_DOUBLE_EQ(mat3.Get(i, j), double(RowColToIndex<3>(i, j)));
//
//    EXPECT_DEATH(mat3.Get(3, 0), "");
//    EXPECT_DEATH(mat3.Get(0, 3), "");
//
//    // initialize matrix from other types, make sure it is constexpr
//    constexpr auto mat1 = Matrix<double, 4, 4>(Identity<double, 4>{});
//    constexpr auto mat2 = Matrix<double, 4, 4>(Zeros<double, 4, 4>{});
//    constexpr auto mat3b = Matrix<double, 4, 4>(Diagonal<double, 4, 4>({1, 2, 3, 4}));
//}
//
//TEST(Linalg, TestDiagonal)
//{
//    EXPECT_TRUE((ConceptSquareMatrix<Diagonal<double, 4, 4>>));
//
//    // test default constructor (all 0)
//    constexpr const Diagonal<double, 4, 4> zeros4{};
//    for(std::size_t i = 0; i < 4; ++i)
//        for(std::size_t j = 0; j < 4; ++j)
//            EXPECT_DOUBLE_EQ(zeros4.Get(i, j), 0.);
//
//    // test constructing from value
//    constexpr const Diagonal<double, 4, 4> twos{2.};
//    for(std::size_t i = 0; i < 4; ++i)
//        for(std::size_t j = 0; j < 4; ++j)
//            EXPECT_DOUBLE_EQ(twos.Get(i, j), i == j ? 2. : 0.);
//
//    // test constructing from array
//    constexpr const Diagonal<double, 5, 5> diag5({0, 1, 2, 3, 4});
//    for(std::size_t i = 0; i < 5; ++i)
//        for(std::size_t j = 0; j < 5; ++j)
//            EXPECT_DOUBLE_EQ(diag5.Get(i, j), i == j ? double(i) : 0.);
//
//    // test constructing from identity
//    constexpr const Identity<double, 6> identity6{};
//    constexpr const Diagonal<double, 6, 6> diag6{identity6};
//    for(std::size_t i = 0; i < 6; ++i)
//        for(std::size_t j = 0; j < 6; ++j)
//            EXPECT_DOUBLE_EQ(diag6.Get(i, j), i == j ? 1. : 0.);
//
//    // test constructing from full matrix (extract diagonal)
//    constexpr const Matrix<double, 3, 3> mat3({0, 1, 2, 3, 4, 5, 6, 7, 8});
//    constexpr const Diagonal<double, 3, 3> diag3(mat3);
//    EXPECT_DOUBLE_EQ(diag3.Get(0, 0), 0.);
//    EXPECT_DOUBLE_EQ(diag3.Get(1, 1), 4.);
//    EXPECT_DOUBLE_EQ(diag3.Get(2, 2), 8.);
//    for(std::size_t i = 0; i < 3; ++i)
//        for(std::size_t j = 0; j < 3; ++j)
//            if(i != j)
//                EXPECT_DOUBLE_EQ(diag3.Get(i, j), 0.);
//}
//
//TEST(Linalg, TestAddDefaultType)
//{
//    using value_type = double;
//    static constexpr std::size_t rows = 3;
//    static constexpr std::size_t cols = 2;
//
//    const std::vector<EDistribution> distributions = {EDistribution::Full, //
//                                                      EDistribution::Sparse, //
//                                                      EDistribution::UpperTriangular, //
//                                                      EDistribution::LowerTriangular, //
//                                                      EDistribution::Diagonal, //
//                                                      EDistribution::StrictUpperTriangular, //
//                                                      EDistribution::StrictLowerTriangular};
//
//    for(auto it = distributions.begin(); it < distributions.end(); ++it)
//    {
//        for(auto it2 = it; it2 < distributions.end(); ++it2)
//        {
//            const auto e1 = AdditionDistribution(*it, *it2);
//            const auto e2 = AdditionDistribution(*it2, *it);
//            ASSERT_EQ(e1, e2); // a + b == b + a
//            ASSERT_NE(e1, EDistribution::Unknown);
//            ASSERT_NE(e2, EDistribution::Unknown);
//        }
//    }
//
//    using MatConst = Matrix<value_type, rows, cols>;
//    using MatRowConst = MatrixRowConst<value_type, rows>;
//    using MatColConst = MatrixColConst<value_type, cols>;
//    using MatDynamic = MatrixDynamic<value_type>;
//
//    static_assert(std::is_same_v<MatConst, default_add_t<MatConst, MatConst>>);
//    static_assert(std::is_same_v<MatRowConst, default_add_t<MatRowConst, MatRowConst>>);
//    static_assert(std::is_same_v<MatColConst, default_add_t<MatColConst, MatColConst>>);
//    static_assert(std::is_same_v<MatDynamic, default_add_t<MatDynamic, MatDynamic>>);
//
//    static_assert(std::is_same_v<MatConst, default_add_t<MatRowConst, MatConst>>);
//    static_assert(std::is_same_v<MatConst, default_add_t<MatConst, MatRowConst>>);
//    static_assert(std::is_same_v<MatConst, default_add_t<MatColConst, MatConst>>);
//    static_assert(std::is_same_v<MatConst, default_add_t<MatConst, MatColConst>>);
//    static_assert(std::is_same_v<MatConst, default_add_t<MatRowConst, MatColConst>>);
//    static_assert(std::is_same_v<MatConst, default_add_t<MatColConst, MatRowConst>>);
//}
//
//TEST(Linalg, TestAddMatrix)
//{
//    constexpr double value = 1.0;
//
//    constexpr auto res1 = Add(Identity<double, 4>{}, value);
//    constexpr auto res2 = Add(value, Identity<double, 4>{});
//    for(std::size_t i = 0; i < 4; ++i)
//    {
//        for(std::size_t j = 0; j < 4; ++j)
//        {
//            EXPECT_DOUBLE_EQ(res1.Get(i, j), i == j ? 2. : 1.);
//            EXPECT_DOUBLE_EQ(res2.Get(i, j), i == j ? 2. : 1.);
//        }
//    }
//
//    constexpr const auto mat1 = Matrix<double, 2, 2>({1, 2, 3, 4});
//    constexpr const auto mat2 = Matrix<double, 2, 2>({5, 6, 7, 8});
//    constexpr const auto res = Add(mat1, mat2);
//    ExpectMatrixEqual(res, Matrix<double, 2, 2>({6, 8, 10, 12}));
//}
//
//TEST(Linalg, TestAddConstant)
//{
//    constexpr Matrix<double, 4, 4> res1 = Add(Identity<double, 4>{}, Identity<double, 4>{});
//    for(std::size_t i = 0; i < 4; ++i)
//        for(std::size_t j = 0; j < 4; ++j)
//            EXPECT_DOUBLE_EQ(res1.Get(i, j), i == j ? 2. : 0.);
//}
//
//TEST(Linalg, TestAddMultivariate)
//{
//    constexpr auto I4 = Identity<double, 4>{};
//    constexpr auto add2 = Add(I4, I4);
//    constexpr auto add3 = Add(I4, I4, I4);
//    constexpr auto add4 = Add(I4, I4, I4, I4);
//    constexpr auto add5 = Add(I4, I4, I4, I4, I4);
//    constexpr auto add6 = Add(I4, I4, I4, I4, I4, I4);
//    constexpr auto add7 = Add(I4, I4, I4, I4, I4, I4, I4);
//    ExpectMatrixEqual(add7, DiagonalConstant<double, 4>{7});
//}
//
//TEST(Linalg, TestDeterminant)
//{
//    constexpr auto identityDetD = Determinant(Identity<double, 100>{});
//    EXPECT_EQ(identityDetD, 1.0);
//    EXPECT_TRUE((std::is_same_v<decltype(identityDetD), const double>));
//
//    constexpr auto identityDetI = Determinant(Identity<int, 1000>{});
//    EXPECT_EQ(identityDetI, 1);
//    EXPECT_TRUE((std::is_same_v<decltype(identityDetI), const int>));
//
//    constexpr Diagonal<double, 5, 5> diag5({1, 2, 3, 4, 5});
//    constexpr auto diag5Det = Determinant(diag5);
//    EXPECT_EQ(diag5Det, 1 * 2 * 3 * 4 * 5);
//
//    constexpr Matrix<int, 3, 3> mat3({1, 2, 3, 4, 5, 6, 7, 8, 9});
//    constexpr auto mat3Det = Determinant(mat3);
//    EXPECT_TRUE((std::is_same_v<decltype(mat3Det), const int>));
//    //EXPECT_EQ(mat3Det, ...);
//
//    // should not compile:
//    // Determinant(Matrix<int, 3, 4>{});
//}
//
//TEST(Linalg, TestInverseTranspose)
//{
//    constexpr Identity<double, 5> identity{};
//    constexpr auto identityTranspose = Transpose(identity);
//    EXPECT_TRUE(IsEqual(identityTranspose, identity));
//    constexpr auto identityInverse = Inverse(identity);
//    EXPECT_TRUE(IsEqual(identityInverse, identity));
//
//    constexpr Diagonal<double, 5, 5> diag({1, 2, 3, 4, 5});
//    constexpr auto transposeDiag = Transpose(diag);
//    EXPECT_TRUE(IsEqual(transposeDiag, diag));
//    constexpr auto inverseDiag = Inverse(diag);
//    constexpr Diagonal<double, 5, 5> inverseDiagAnswer({1., 1. / 2., 1. / 3., 1. / 4, 1. / 5.});
//    EXPECT_TRUE(IsEqual(inverseDiag, inverseDiagAnswer));
//
//    constexpr Matrix<double, 3, 3> matrix3({1, 2, 3, 4, 5, 6, 7, 8, 9});
//    constexpr auto inverseMat3 = Inverse(matrix3);
//    EXPECT_TRUE(IsEqual(Matmul(matrix3, inverseMat3), Identity<double, 3>{}));
//
//    constexpr auto transposeMat3 = Transpose(matrix3);
//    constexpr Matrix<double, 3, 3> transposeMatrix3Answer({1, 4, 7, 2, 5, 8, 3, 6, 9});
//    EXPECT_TRUE(IsEqual(transposeMat3, transposeMatrix3Answer));
//
//    // TODO: check that a matrix is rotation, and can be transposed instead of inversed
//
//    // TODO: Check block diagonal matrix inverse is inverse per block
//}
//
//TEST(Linalg, TestInverseDiagonal)
//{
//    constexpr auto diag1 = Diagonal<double, 3, 3>({1, 2, 3});
//    constexpr Diagonal<double, 3, 3> diagInv1 = InverseDiagonal(diag1);
//    constexpr Diagonal<double, 3, 3> diagInv2 = Inverse(diag1);
//    EXPECT_TRUE(IsEqual(Matmul(diag1, diagInv1), Identity<double, 3>{}));
//    EXPECT_TRUE(IsEqual(Matmul(diag1, diagInv2), Identity<double, 3>{}));
//}
//
//TEST(Linalg, TestInverseRandom)
//{
//    // NOTE: these numbers are inside gtest context, including construction of random matrix etc.
//    // Not representative of actual performance
//
//    // 50x50; 100 iters;    766 [ms]
//    // 75x75; 100 iters;    2.7 [s]
//    // 100x100; 100 iters;  6.8 [s]
//    // 500x500; 1 iter;     12.5 [s]
//    // 600x600; 1 iter;     16.5 [s]
//
//    std::default_random_engine g;
//    std::uniform_real_distribution<double> dist(0.01, 100.); // todo: log norm
//
//    constexpr std::size_t n = 50;
//    constexpr auto identity = Identity<double, n>{};
//
//    // test a bunch of random matrices
//    for(std::size_t i = 0; i < 100; ++i)
//    {
//        Matrix<double, n, n> matrix{};
//        for(std::size_t r = 0; r < n; ++r)
//            for(std::size_t c = 0; c < n; ++c)
//                matrix.At(r, c) = dist(g);
//
//        const auto inverse = InverseHotellingBodewig(matrix, 1E-10);
//        const auto result = Matmul(inverse, matrix);
//
//        ExpectMatrixEqual(result, identity, 1E-8); // A^-1 * A == I
//    }
//}
//
//TEST(Linalg, TestBracket3)
//{
//    constexpr Vector3<double> vec({1, 2, 3});
//    constexpr auto bracket = Bracket3(vec);
//    constexpr Matrix<double, 3, 3> answer({0, -3, 2, 3, 0, -1, -2, 1, 0});
//    ExpectMatrixEqual(bracket, answer);
//
//    constexpr auto bSquared = Matmul(bracket, bracket);
//}
//
//TEST(Linalg, TestBracket6)
//{
//    // TODO
//}
//
//TEST(Linalg, TestLieBracket)
//{
//    // TODO
//}
//
//TEST(Linalg, TestLambdaMatrix)
//{
//    constexpr auto lambda = [](const std::size_t i, const std::size_t j) {
//        return (i == j) ? 1. : 0.; //
//    };
//    constexpr LambdaMatrix<decltype(lambda), 3, 3> mat{lambda};
//
//    ExpectMatrixEqual(mat, Identity<double, 3>{}, 1E-14);
//}
//
//TEST(Linalg, TestMatmul4x4Perf)
//{
//    std::default_random_engine g;
//    std::uniform_real_distribution<double> r(0.01, 100.);
//
//#ifdef _DEBUG
//    constexpr uint32_t iters = 100;
//#else
//    constexpr uint32_t iters = 100000;
//#endif
//
//    // release: ~5e7 matmuls/sec (with static volatile)
//    // debug:   matmuls/sec
//    for(const auto i : Range(1000))
//    {
//        const Matrix<double, 4, 4> mat1{{r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g)}};
//        const Matrix<double, 4, 4> mat2{{r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g), r(g)}};
//
//        for(const auto j : Range(iters))
//        {
//            const auto res = Matmul(mat1, mat2);
//            static volatile auto copy = res; // avoid optimizations
//        }
//    }
//}
//
//TEST(Linalg, TestMatmulDefaultType)
//{
//    using value_type = double;
//    static constexpr std::size_t rows = 3;
//    static constexpr std::size_t cols = 2;
//
//    using MatConst = Matrix<value_type, rows, cols>;
//    using MatRowConst = MatrixRowConst<value_type, rows>;
//    using MatColConst = MatrixColConst<value_type, cols>;
//    using MatDynamic = MatrixDynamic<value_type>;
//
//    static_assert(std::is_same_v<MatConst, default_matmul_t<MatConst, MatConst>>);
//    static_assert(std::is_same_v<MatRowConst, default_matmul_t<MatRowConst, MatRowConst>>);
//    static_assert(std::is_same_v<MatColConst, default_matmul_t<MatColConst, MatColConst>>);
//    static_assert(std::is_same_v<MatDynamic, default_matmul_t<MatDynamic, MatDynamic>>);
//
//    static_assert(std::is_same_v<MatConst, default_matmul_t<MatRowConst, MatConst>>);
//    static_assert(std::is_same_v<MatRowConst, default_matmul_t<MatConst, MatRowConst>>);
//    static_assert(std::is_same_v<MatColConst, default_matmul_t<MatColConst, MatConst>>);
//    static_assert(std::is_same_v<MatConst, default_matmul_t<MatConst, MatColConst>>);
//    static_assert(std::is_same_v<MatConst, default_matmul_t<MatRowConst, MatColConst>>);
//    static_assert(std::is_same_v<MatDynamic, default_matmul_t<MatColConst, MatRowConst>>);
//}
//
//TEST(Linalg, TestSparse)
//{
//    static_assert(is_sparse_v<MatrixSparse<double>>);
//    static_assert(is_sparse_v<MatrixSparse<int>>);
//    static_assert(ConceptSparse<MatrixSparse<double>>);
//    static_assert(ConceptSparse<MatrixSparse<int>>);
//
//    // try constructing with explicit size
//    MatrixSparse<double> sparse(100, 100);
//    EXPECT_EQ(sparse.GetRows(), 100);
//    EXPECT_EQ(sparse.GetColumns(), 100);
//
//    // try constructing from existing matrix
//    Identity<double, 100> identity100{};
//    MatrixSparse<double> sparseFromIdentiy(identity100);
//    ExpectMatrixEqual(identity100, sparseFromIdentiy);
//
//    constexpr const Matrix<double, 3, 3> mat3({0, 1, 2, 3, 4, 5, 6, 7, 8});
//    MatrixSparse<double> sparseFromMat3(mat3);
//    ExpectMatrixEqual(mat3, sparseFromMat3);
//
//    // todo: test/specialize solve
//}
