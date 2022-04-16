#pragma once

#include "linalg.h"
#include "traits.h"
#include "vic/linalg/matrices.h"
#include "vic/utils.h"
#include <algorithm>

namespace vic
{
namespace linalg
{
// Solving a matrix equation means solving the equation A*x = b, without calculating A^-1.
// Generally, this means calculating a partial inverse (the inverse of the diagonal for instance)

// https://www.robots.ox.ac.uk/~sjrob/Teaching/EngComp/linAlg34.pdf
template <typename TMatrix, typename TVector>
requires ConceptSquareMatrix<TMatrix>
auto SolveJacobiMethod(const TMatrix& matrix, const TVector& vector, const double eps = 1E-10)
{
    constexpr const std::size_t n = TMatrix::GetRows();
    const Diagonal<typename TMatrix::DataType, n, n> D(matrix);
    const auto Dinv = InverseDiagonal(D);

    const auto lPlusU = Add(matrix, Matmul(-1., D)); // (L+U) == matrix - diagonal

    // TODO: if matrix is very large, do not precompute this matrix
    // just calculate the list of matrix-vector multiplications separately
    const auto DInvLPlusU = Matmul(Dinv, lPlusU);

    const auto DInvB = Matmul(Dinv, vector);

    // make an initial estimate by inversing diagonal
    auto u = Matmul(Dinv, vector);

    for(std::size_t i = 0; i < 1000; ++i)
    {
        const auto firstTerm = Matmul(-1., DInvLPlusU, u);
        u = Add(firstTerm, DInvB);
    }
    return u;
}

// Selector for Inverse algorithm
template <typename TMatrix, typename TVector>
requires ConceptSquareMatrix<TMatrix>
constexpr auto Solve(const TMatrix& matrix, const TVector& vector)
{ //
    return TVector{};
}

} // namespace linalg
} // namespace vic