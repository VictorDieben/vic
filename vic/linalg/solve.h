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
// https://www.robots.ox.ac.uk/~sjrob/Teaching/EngComp/linAlg34.pdf
template <typename TMatrix, typename TVector>
requires ConceptSquareMatrix<TMatrix>
auto SolveJacobiMethod(const TMatrix& matrix, const TVector& vector, const double eps = 1E-10)
{
    constexpr const std::size_t n = TMatrix::GetRows();
    const Diagonal<typename TMatrix::DataType, n, n> D(matrix);
    const auto Dinv = InverseDiagonal(D);

    const auto lPlusU = Add(matrix, Matmul(-1., D)); // (L+U) == matrix - diagonal

    // TODO(vicdie): if matrix is very large, do not precompute this matrix
    // just calculate the list of matrix-vector multiplications separately
    const auto DInvLPlusU = Matmul(Dinv, lPlusU);

    const auto DInvB = Matmul(Dinv, vector);

    auto u = Matmul(Dinv, vector); // initialize by inversing diagonal

    for(std::size_t i = 0; i < 1000; ++i)
    {
        const auto firstTerm = Matmul(-1., DInvLPlusU, u);
        const auto utmp = Add(firstTerm, DInvB);
        u = utmp;
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