#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/traits.h"

#include "vic/linalg/algorithms/matmul.h"
#include "vic/linalg/algorithms/solve.h"
#include "vic/linalg/algorithms/transpose.h"

#include <cassert>

namespace vic
{
namespace linalg
{

// todo: make both A.T * A and A * A.T
template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
constexpr ConceptMatrix auto GramMatrix(const TMatrix& A)
{
    const auto At = Transpose(A);
    return Matmul(At, A);
}

template <typename TMatrix, typename TVector>
    requires ConceptMatrix<TMatrix> && ConceptMatrix<TVector>
constexpr auto LeastSquares(const TMatrix& A, const TVector& b)
{
    // todo: what if b is not a vector, but a list of n-dimensional measurements?
    assert(A.GetRows() >= A.GetColumns());

    // todo: make (At * A) a separate function, can be optimized i think
    // https://en.wikipedia.org/wiki/Gram_matrix
    const auto At = Transpose(A);
    const auto AtA = Matmul(At, A);

    const auto bHat = Matmul(At, b);

    const auto xHat = SolveConjugateGradient(AtA, bHat);
    // const auto xHat = SolveJacobiMethod(AtA, bHat);

    return xHat;
}

} // namespace linalg
} // namespace vic