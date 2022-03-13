#pragma once

#include "linalg.h"
#include "traits.h"

namespace vic
{
namespace linalg
{

template <typename TMatrix>
requires ConceptSquareMatrix<TMatrix>
constexpr auto Inverse(const TMatrix& matrix)
{
    // TODO(vicdie): if is:
    // - rotation
    // - transformation
    // - banded
    // - block

    if constexpr(std::is_same_v<TMatrix, Identity<TMatrix::DataType, TMatrix::GetRows()>>)
    {
        return Identity<TMatrix::DataType, TMatrix::GetRows()>{}; // inverse of identity is identity
    }
    else if constexpr(std::is_same_v<TMatrix, Diagonal<TMatrix::DataType, TMatrix::GetRows(), TMatrix::GetColumns()>> && //
                      (TMatrix::GetRows() == TMatrix::GetColumns()))
    {
        return InverseDiagonal(matrix);
    }
    else
    {
        return matrix;
    }
}

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
// requires ConceptSquareMatrix<TMat>
constexpr auto InverseStatic(const TMat& mat)
{
    using T = typename TMat::DataType;
    constexpr std::size_t n = mat.GetRows();
    const auto twoI = Matmul(2., Identity<T, n>{});

    auto V = Matrix<T, n, n>(Matmul(1E-10, Transpose(mat)));

    // Hotelling-Bodewig algorithm: V_n+1 = V_n * (2*I - A*V_n)
    for(std::size_t i = 0; i < 100; ++i)
    {
        const auto tmp1 = Matmul(-1., Matmul(mat, V));
        const auto tmp2 = Add(twoI, tmp1);
        const auto tmp3 = Matmul(V, tmp2);
        V = Matrix<T, n, n>{tmp3};
    }
    return V;
}

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto InverseDynamic(const TMat& mat)
{
    // TODO(vicdie): maybe implement this function in a different file,
    // so we do not need to compile it if we don't want it
    return Zeros<double, 1, 1>{};
}

} // namespace linalg
} // namespace vic