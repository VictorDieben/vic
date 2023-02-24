#include "../pch.h"
#include "../test_base.h"

#include "vic/linalg/inverse.h"
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/matrices_sparse.h"
#include "vic/linalg/solve.h"
#include "vic/linalg/traits.h"
#include "vic/linalg/transpose.h"

#include "vic/linalg/tools.h"
#include <random>

namespace vic
{
namespace linalg
{
TEST(TestSolve, JacobiSimple)
{
    constexpr std::size_t n = 3;
    Diagonal<double, n, n> A{{4., 5., 6.}};
    Matrix<double, n, 1> b{{1., 2., 3.}};

    const auto ans = SolveJacobiMethod(A, b);
    const auto b2 = Matmul(A, ans);
    ExpectMatrixEqual(b, b2, 1E-6);
}

TEST(TestSolve, JacobiRandom)
{
    constexpr std::size_t n = 100;

    std::default_random_engine g;
    std::uniform_real_distribution<double> distDiagonal(1., 10.);
    std::uniform_real_distribution<double> distOffDiagonal(0.001, 0.01);

    for(std::size_t i = 0; i < 1; ++i)
    {
        Matrix<double, n, 1> vector{};
        Matrix<double, n, n> matrix{};
        for(std::size_t r = 0; r < n; ++r)
        {
            vector.At(r, 0) = distDiagonal(g);
            for(std::size_t c = 0; c < n; ++c)
            {
                matrix.At(r, c) = (r == c) ? distDiagonal(g) : distOffDiagonal(g);
            }
        }

        const auto ans = SolveJacobiMethod(matrix, vector);
        const auto vector2 = Matmul(matrix, ans);
        ExpectMatrixEqual(vector, vector2, 1.E-6);
    }
}

TEST(TestSolve, JacobiLargeSparse)
{
    std::cout << "init" << std::endl;

#ifdef _DEBUG
    constexpr std::size_t n = 500;
#else
    constexpr std::size_t n = 30000;
#endif

    constexpr std::size_t nFill = 5;
    static constexpr Identity<double, n> identity10000{};
    MatrixSparse<double> largeSparse{identity10000};

    std::default_random_engine g;
    std::uniform_real_distribution<double> randomValue(0., 1. / (n * nFill));
    std::uniform_int_distribution<> randomIndex(0, (n * n) - 1);

    std::cout << "start constructing random indices" << std::endl;

    std::vector<std::size_t> indices;
    for(std::size_t i = 0; i < 5 * n; ++i)
        indices.push_back(randomIndex(g));
    std::sort(indices.begin(), indices.end());
    std::set uniqueIndices(indices.begin(), indices.end());

    std::cout << "start constructing sparse mat" << std::endl;

    // fill some random indices with random values
    for(const auto& index : uniqueIndices)
    {
        const auto [i, j] = IndexToRowCol(index, n);
        largeSparse.At(i, j) = randomValue(g);
    }

    Matrix<double, n, 1> vector{};
    for(std::size_t r = 0; r < n; ++r)
        vector.At(r, 0) = randomValue(g);

    std::cout << "start solving" << std::endl;

    const auto x = SolveJacobiMethod(largeSparse, vector);

    std::cout << "done solving" << std::endl;

    const auto vector2 = Matmul(largeSparse, x);
    ExpectMatrixEqual(vector, vector2, 1.E-10);

    std::cout << "done" << std::endl;
}
} // namespace linalg
} // namespace vic