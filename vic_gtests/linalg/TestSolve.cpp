#include "../test_base.h"
#include "gtest/gtest.h"

#include "vic/linalg/linalg.h"

#include <random>

namespace vic
{
namespace linalg
{

TEST(TestSolve, JacobiSimple)
{
    constexpr std::size_t n = 3;
    const auto A = ToDiagonal(std::array{4., 5., 6.});
    Vector3<double> b{{1., 2., 3.}};

    const auto ans = SolveJacobiMethod(A, b);
    const auto b2 = Matmul(A, ans);
    ExpectMatrixEqual(b, b2, 1E-10);
}

TEST(TestSolve, JacobiRandom)
{
    constexpr std::size_t n = 100;

    std::default_random_engine g;
    std::uniform_real_distribution<double> distDiagonal(1., 10.);
    std::uniform_real_distribution<double> distOffDiagonal(0.001, 0.01);

    for(std::size_t i = 0; i < 1; ++i)
    {
        MatrixMxN<double, n, 1> vector{};
        MatrixMxN<double, n, n> matrix{};
        for(MatrixSize r = 0; r < n; ++r)
        {
            vector.At(r, 0) = distDiagonal(g);
            for(MatrixSize c = 0; c < n; ++c)
            {
                matrix.At(r, c) = (r == c) ? distDiagonal(g) : distOffDiagonal(g);
            }
        }

        const auto matrix2 = Matmul(Transpose(matrix), matrix);

        const auto ans = SolveConjugateGradient(matrix2, vector);

        // const auto ans = SolveJacobiMethod(matrix2, vector);

        const auto vector2 = Matmul(matrix2, ans);
        EXPECT_TRUE(IsEqual(vector, vector2, 1e-4));
    }
}

TEST(TestSolve, JacobiLargeSparse)
{
    std::cout << "init" << std::endl;

#ifdef _DEBUG
    constexpr MatrixSize n = 100;
#else
    constexpr MatrixSize n = 1500;
#endif

    constexpr std::size_t nFill = 5;
    static constexpr IdentityN<double, n> identity10000{};
    Sparse<double, Shape<UnknownSize, UnknownSize>> largeSparse{n, n};

    std::default_random_engine g;
    std::uniform_real_distribution<double> randomValue(0., 1. / nFill);
    std::uniform_int_distribution<MatrixSize> randomIndex(0, (n * n) - 1u);

    std::cout << "start constructing random indices" << std::endl;

    std::vector<MatrixSize> indices;
    indices.reserve(5 * n);
    for(MatrixSize i = 0; i < (nFill - 1) * n; ++i)
        indices.push_back(randomIndex(g));
    for(MatrixSize i = 0; i < n; ++i)
        indices.push_back(RowMayorRowColToIndex(i, i, n));

    // std::sort(indices.begin(), indices.end());
    std::set uniqueIndices(indices.begin(), indices.end());

    std::cout << "start constructing sparse mat" << std::endl;

    // fill some random indices with random values
    for(const auto& index : uniqueIndices)
    {
        const auto [i, j] = RowMayorIndexToRowCol(index, n);
        if(i == j)
            largeSparse.At(i, j) = 1. + (nFill * randomValue(g));
        else
            largeSparse.At(i, j) = randomValue(g);
    }

    Matrix<double, Shape<UnknownSize, 1>> vector{n, 1};
    std::uniform_real_distribution<double> bDist(-10., 10.);
    for(Row r = 0; r < n; ++r)
        vector.At(r, 0) = bDist(g);

    std::cout << "start solving" << std::endl;

    // const auto x = SolveJacobiMethod(largeSparse, vector);
    const auto x = SolveConjugateGradient(largeSparse, vector);

    std::cout << "done solving" << std::endl;

    const auto vector2 = Matmul(largeSparse, x);
    ExpectMatrixEqual(vector, vector2, 1.E-10);

    std::cout << "done" << std::endl;
}

TEST(TestSolve, SimpleConjugateGradient)
{
    const Matrix2<double> A{{4, 1, 1, 3}};
    const Vector2<double> b{{1, 2}};

    const Vector2<double> solution{{0.0909, 0.6364}};

    const Vector2<double> x = SolveConjugateGradient(A, b);

    EXPECT_TRUE(IsEqual(solution, x, 1e-4));

    EXPECT_TRUE(IsEqual(b, Matmul(A, x), 1e-10));
}

TEST(TestSolve, UpperTriangular)
{
    // note: compile time matrix solving \o/
    static constexpr const Matrix3<double> A{{1, 2, 3, 0, 4, 5, 0, 0, 6}};
    static constexpr const Vector3<double> b{{1, 2, 3}};

    static constexpr const auto x = SolveUpperTriangular(A, b);

    static constexpr const auto tmp = Matmul(A, x);
    EXPECT_TRUE(IsEqual(b, tmp));
}

TEST(TestSolve, LowerTriangular)
{
    static constexpr const Matrix3<double> A{{1, 0, 0, 2, 3, 0, 4, 5, 6}};
    static constexpr const Vector3<double> b{{1, 2, 3}};

    static constexpr const auto x = SolveLowerTriangular(A, b);

    static constexpr const auto tmp = Matmul(A, x);
    EXPECT_TRUE(IsEqual(b, tmp));
}

} // namespace linalg
} // namespace vic