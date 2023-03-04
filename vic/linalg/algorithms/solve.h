#pragma once

#include "vic/linalg/algorithms/inverse.h"
#include "vic/linalg/linalg.h"
#include "vic/utils.h"

#include <algorithm>

namespace vic
{
namespace linalg
{
// Solving a system means solving the equation A*x = b, without calculating A^-1.
// Generally, this means calculating a partial inverse (the inverse of the diagonal for instance)

// https://www.robots.ox.ac.uk/~sjrob/Teaching/EngComp/linAlg34.pdf

template <typename TMatrix, typename TVector>
requires ConceptMatrix<TMatrix> && ConceptVector<TVector>
auto SolveJacobiMethod(const TMatrix& matrix, const TVector& vector, const double eps = 1E-12)
{
    const MatrixSize n = matrix.GetRows();
    const auto D = ToDiagonal(matrix);
    const auto Dinv = InverseDiagonal(D);

    // const auto lPlusU = Add(matrix, Matmul(-1., D)); // (L+U) == matrix - diagonal
    auto lPlusU = matrix;
    for(MatrixSize i = 0; i < n; ++i)
        lPlusU.At(i, i) = 0.;
    if constexpr(ConceptSparse<decltype(lPlusU)>)
        lPlusU.Prune();

    // make an initial estimate by inversing diagonal
    //auto x = Matmul(0.1, Matmul(Dinv, vector));
    auto x = Matmul(Dinv, vector);

    // TODO: verify that matrix is diagonally dominant

    // x_i+1 = D^-1 * (b - (L+U)*x)
    for(std::size_t i = 0; i < 1000; ++i)
    {
        // const auto lPlusU_x = Matmul(lPlusU, x);

        const auto a = Matmul(matrix, x);
        const auto b = Matmul(-1., Matmul(D, x));
        const auto lPlusU_x = Add(a, b);

        const auto neg_lPlusU_x = Matmul(-1., lPlusU_x);
        const auto sum = Add(vector, neg_lPlusU_x);
        auto x_new = Matmul(Dinv, sum);

        double largestDiff = 0.;
        for(MatrixSize i = 0; i < n; ++i)
            largestDiff = Max(largestDiff, Abs(x.Get(i, 0) - x_new.Get(i, 0)));
        x = x_new;

        std::cout << "SolveJacobiMethod(): i = " << i << "; diff = " << largestDiff << std::endl;
        if(largestDiff < eps)
        {
            std::cout << "SolveJacobiMethod(): i = " << i << "; Done!" << std::endl;
            break;
        }
    }

    return x;
}

// Selector for Inverse algorithm
template <typename TMatrix, typename TVector>
requires ConceptSquareMatrix<TMatrix>
constexpr auto Solve(const TMatrix& matrix, const TVector& vector)
{
    return SolveJacobiMethod(matrix, vector); //
}

} // namespace linalg
} // namespace vic