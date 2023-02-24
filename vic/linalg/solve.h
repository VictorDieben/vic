#pragma once

#include "linalg.h"
#include "traits.h"
#include "vic/linalg/add.h"
#include "vic/linalg/matrices.h"
#include "vic/linalg/matrices_dynamic.h"
#include "vic/utils.h"
#include <algorithm>

namespace vic
{
namespace linalg
{
// Solving a matrix equation means solving the equation A*x = b, without calculating A^-1.
// Generally, this means calculating a partial inverse (the inverse of the diagonal for instance)

// todo: matrix view that returns the original matrix, except for the values on the diagonal.
// Those are inverted (1/val)
//template <typename TMat>
//class JacobiPseudoInverseView
//{
//public:
//    JacobiPseudoInverseView(const TMat& mat) { }
//
//private:
//};

// https://www.robots.ox.ac.uk/~sjrob/Teaching/EngComp/linAlg34.pdf
template <typename TMatrix, typename TVector> // requires ConceptSquareMatrix<TMatrix>
auto SolveJacobiMethod(const TMatrix& matrix, const TVector& vector, const double eps = 1E-10)
{
    const std::size_t n = matrix.GetRows();
    const DiagonalDynamic<typename TMatrix::DataType> D{matrix};
    const auto Dinv = InverseDiagonal(D);

    // const auto lPlusU = Add(matrix, Matmul(-1., D)); // (L+U) == matrix - diagonal
    auto lPlusU = matrix;
    for(std::size_t i = 0; i < n; ++i)
        lPlusU.At(i, i) = 0.;
    if constexpr(ConceptSparse<decltype(lPlusU)>)
        lPlusU.Prune();

    // make an initial estimate by inversing diagonal
    auto x = Matmul(0.1, Matmul(Dinv, vector));

    // TODO: verify that matrix is diagonally dominant

    // x_i+1 = D^-1 * (b - (L+U)*x)
    for(std::size_t i = 0; i < 1000; ++i)
    {
        const auto lPlusU_x = Matmul(lPlusU, x);
        const auto neg_lPlusU_x = Matmul(-1., lPlusU_x);
        const auto sum = Add(vector, neg_lPlusU_x);
        auto x_new = Matmul(Dinv, sum);

        double largestDiff = 0.;
        for(std::size_t i = 0; i < n; ++i)
            largestDiff = Max(largestDiff, Abs(x.Get(i, 0) - x_new.Get(i, 0)));
        x = x_new;

        if(largestDiff < 1.e-12)
        {
            std::cout << "SolveJacobiMethod(): i = " << i << std::endl;
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