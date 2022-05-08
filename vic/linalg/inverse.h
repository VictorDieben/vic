#pragma once

#include "vic/linalg/linalg.h"
#include "vic/linalg/matrix_view.h"
#include "vic/linalg/traits.h"
#include "vic/utils.h"

#include <algorithm>

namespace vic
{
namespace linalg
{
// inverse of diagonal is 1/<diag value> (for square matrices)
template <typename T, std::size_t size>
constexpr Diagonal<T, size, size> InverseDiagonal(const typename Diagonal<T, size, size>& matrix)
{
    Diagonal<T, size, size> inverse{};
    for(std::size_t i = 0; i < size; ++i)
    {
        inverse.At(i, i) = 1. / matrix.Get(i, i);
    }
    return inverse;
}

// Inverse of rotation is transpose
template <typename T, std::size_t size>
constexpr Diagonal<T, size, size> InverseRotation(const Matrix<T, size, size>& matrix)
{
    return Transpose(matrix);
}

//  Inverse of normal matrix. Result will always be a full matrix
template <typename TMat>
requires ConceptSquareMatrix<TMat>
constexpr auto InverseGeneral(const TMat& mat)
{
    if constexpr(ConceptConstexprRows<TMat> && ConceptConstexprRows<TMat>)
        return InverseStatic(mat);
    else
        return InverseDyanmic(mat);
}

template <typename TMat>
requires ConceptSquareMatrix<TMat>
constexpr auto InverseStatic(const TMat& mat)
{
    // TODO: several algorithms, selector, estimator for how good it is
    return InverseHotellingBodewig(mat, 1E-10);
}

template <typename TMat>
requires ConceptSquareMatrix<TMat>
constexpr auto InverseHotellingBodewig(const TMat& A, const typename TMat::DataType eps)
{
    // Hotelling-Bodewig algorithm: V_n+1 = V_n * (2*I - A*V_n)
    using T = typename TMat::DataType;
    constexpr std::size_t n = A.GetRows();
    constexpr auto identity = Identity<double, n>{};
    constexpr auto twoI = DiagonalConstant<T, n>(2.);

    // TODO: good initial guess
    auto V = Matrix<T, n, n>(Matmul(1E-10, Transpose(A)));

    // TODO: choose decent max iterations
    std::size_t i = 0;
    for(; i < 1000; ++i)
    {
        const auto tmp = Matmul(A, V);
        V = Matmul(V, Add(twoI, Matmul(-1., tmp)));

        // sum up the difference between tmp and I, break if it is small enough
        double absSum = 0.;
        for(std::size_t i = 0; i < n; ++i)
            for(std::size_t j = 0; j < n; ++j)
                absSum += Abs(tmp.Get(i, j) - identity.Get(i, j));
        if(absSum < eps * n * n)
            break;
    }
    return V;
}

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto InverseDynamic(const TMat& mat)
{
    // TODO: maybe implement this function in a different file,
    // so we do not need to compile it if we don't want it
    return Zeros<double, 1, 1>{};
}

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto InverseQualtiy(const TMat1& matrix, const TMat2& inverse)
{
    // A * A^-1 == I,
    // check how well this inverse approaches Identity
    constexpr auto identity = Identity<double, matrix.GetRows()>{};
    const auto res = Matmul(matrix, inverse);
    double absSum = 0.;
    for(std::size_t i = 0; i < res.GetRows(); ++i)
        for(std::size_t j = 0; j < res.GetRows(); ++j)
            absSum += std::abs(res.Get(i, j) - identity.Get(i, j));

    return absSum / double(res.GetRows() * res.GetRows());
}

// todo: frobenius inner product
template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto FrobeniusInnerProduct(const TMat1& mat1, const TMat2& mat2)
{
    // calculate tr(mat1.T * mat2)
}

// todo: frobenius inner product
template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto FrobeniusInnerProduct(const TMat& mat)
{
    // calculate tr(A.T * A)
    typename TMat::DataType sum = 0.;
    constexpr const std::size_t n = mat.GetRows();
    for(std::size_t i = 0; i < n; ++i)
        for(std::size_t j = 0; j < n; ++j)
            sum += (mat.at(i, j) * mat(j, i));
    return sum;
}

// todo: pre-conditioners

// todo:

// Selector for Inverse algorithm
template <typename TMatrix>
requires ConceptSquareMatrix<TMatrix>
constexpr auto Inverse(const TMatrix& matrix)
{
    // TODO: if is:
    // - rotation
    // - transformation
    // - banded
    // - block

    // TODO: check if user has made a specialization for this specific type of matrix

    // TODO: split Inverse() up into 2 parts:
    // - preconditioner, initial guess, partly dependant on solver
    // - solving algorithm, either iterative/LU-decomposition/something else

    if constexpr(std::is_same_v<TMatrix, Identity<TMatrix::DataType, TMatrix::GetRows()>>)
    {
        return Identity<TMatrix::DataType, TMatrix::GetRows()>{}; // inverse of identity is identity
    }
    else if constexpr(std::is_same_v<TMatrix, Diagonal<TMatrix::DataType, TMatrix::GetRows(), TMatrix::GetColumns()>>)
    {
        return InverseDiagonal(matrix);
    }
    else
    {
        return InverseGeneral(matrix);
    }
}

} // namespace linalg
} // namespace vic