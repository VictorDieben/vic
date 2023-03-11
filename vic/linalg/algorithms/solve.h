#pragma once

#include "vic/linalg/matrices/ones.h"
#include "vic/linalg/matrices/zeros.h"

#include "vic/linalg/algorithms/inverse.h"
#include "vic/linalg/linalg.h"
#include "vic/linalg/tools.h"
#include "vic/utils.h"

#include <algorithm>

namespace vic
{
namespace linalg
{
// Solving a system means solving the equation A*x = b, without calculating A^-1.
// Generally, this means calculating a partial inverse (the inverse of the diagonal for instance)

template <typename TMatrix, typename TVector>
requires ConceptMatrix<TMatrix> && ConceptVector<TVector>
auto SolveConjugateGradient(const TMatrix& A, const TVector& b, const double eps = 1E-12)
{
    // CG will only work if matrix is symmetric (A.T == A), and positive definite (output is positive for any positive vector)

    // todo: verify at runtime?
    // assert(ConceptSymmetric<TMatrix>);
    // assert(ConceptPositiveDefinite<TMatrix>);

    assert(A.GetRows() == A.GetColumns() && //
           A.GetColumns() == b.GetRows() && //
           b.GetColumns() == 1);

    using DataType = typename TMatrix::DataType;

    // only valid matrix shape is square, if we know either row or col size, we know both
    using MatrixShape = SquareShape<typename TMatrix::ShapeType>;
    using ResultShape = Shape<MatrixShape::rows, 1>;

    auto x = To<Matrix<DataType, ResultShape>>(Matmul(0.1, ToOnes(b)));

    auto bHat = Matmul(A, x);
    auto r = Subtract(b, bHat); // residual
    auto p = r; // search direction

    DataType rsold = Matmul(Transpose(r), r).Get(0, 0);

    uint32_t i = 0;
    for(; i < Min(A.GetRows(), A.GetColumns()); ++i)
    {
        const auto Ap = Matmul(A, p);
        const DataType alpha = rsold / Matmul(Transpose(p), Ap).Get(0, 0);
        const auto xTemp = Add(x, Matmul(alpha, p));
        x = xTemp; // avoid aliasing issues
        const auto rn = Subtract(r, Matmul(alpha, Ap));
        auto rsnew = Matmul(Transpose(rn), rn).Get(0, 0);

        if(Sqrt(rsnew) < eps)
            break;

        const auto pTemp = Add(rn, Matmul(rsnew / rsold, p));
        p = pTemp;
        rsold = rsnew;
    }

    return x;
}

// https://www.robots.ox.ac.uk/~sjrob/Teaching/EngComp/linAlg34.pdf

template <typename TMatrix, typename TVector>
requires ConceptMatrix<TMatrix> && ConceptVector<TVector>
auto SolveJacobiMethod(const TMatrix& matrix, const TVector& vector, const double eps = 1E-12)
{
    assert(matrix.GetRows() == matrix.GetColumns());
    // TODO: verify that matrix is diagonally dominant / positive definite

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
        x = To<decltype(x)>(x_new);

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
requires ConceptMatrix<TMatrix> && ConceptVector<TVector>
constexpr auto Solve(const TMatrix& matrix, const TVector& vector)
{
    return SolveJacobiMethod(matrix, vector); //
}

} // namespace linalg
} // namespace vic