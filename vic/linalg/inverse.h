#pragma once

#include "linalg.h"
#include "traits.h"

namespace vic
{
namespace linalg
{

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
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

} // namespace linalg
} // namespace vic