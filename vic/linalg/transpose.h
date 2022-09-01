#pragma once

// #include "linalg.h"

namespace vic
{
namespace linalg
{

template <typename TMatrix> // requires ConceptMatrix<TMatrix>
constexpr auto Transpose(const TMatrix& matrix)
{
    if constexpr(std::is_same_v<TMatrix, Identity<typename TMatrix::DataType, TMatrix::GetRows()>>)
    {
        return matrix; // transpose of identity is identity
    }
    else if constexpr(std::is_same_v<TMatrix, Diagonal<typename TMatrix::DataType, TMatrix::GetRows(), TMatrix::GetColumns()>> //
                      && (TMatrix::GetRows() == TMatrix::GetColumns()))
    {
        return matrix; // transpose of diag is diag
    }
    else
    {
        return TransposeGeneral<TMatrix>(matrix);
    }
}

template <typename TMatrix> // requires ConceptMatrix<TMatrix>
constexpr auto TransposeGeneral(const TMatrix& matrix)
{
    Matrix<typename TMatrix::DataType, TMatrix::GetColumns(), TMatrix::GetRows()> transpose;
    for(std::size_t i = 0; i < TMatrix::GetRows(); ++i)
        for(std::size_t j = 0; j < TMatrix::GetColumns(); ++j)
            transpose.At(j, i) = matrix.Get(i, j);
    return transpose;
}

} // namespace linalg
} // namespace vic