#include "../pch.h"
#include "../test_base.h"

#include "vic/linalg/inverse.h"
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/linalg/solve.h"
#include "vic/linalg/traits.h"
#include "vic/linalg/transpose.h"

#include "vic/linalg/tools.h"

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

    constexpr std::size_t n = 3;

    std::default_random_engine g;
    std::uniform_real_distribution<double> distDiagonal(0.01, 100.);
    std::uniform_real_distribution<double> distOffDiagonal(0.01, 1);

    for(std::size_t i = 0; i < 100; ++i)
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
        ExpectMatrixEqual(vector, vector2, 1E-6);
    }
}
} // namespace linalg
} // namespace vic